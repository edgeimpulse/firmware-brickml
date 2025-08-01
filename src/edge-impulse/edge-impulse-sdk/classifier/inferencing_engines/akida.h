/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef EI_CLASSIFIER_INFERENCING_ENGINE_AKIDA_H
#define EI_CLASSIFIER_INFERENCING_ENGINE_AKIDA_H

#if (EI_CLASSIFIER_INFERENCING_ENGINE == EI_CLASSIFIER_AKIDA)

/**
 * @brief if we are not forcing SOFTWARE inference (simulation)
 *  then make sure we will try to use hardware
 *
 */
#ifndef EI_CLASSIFIER_USE_AKIDA_SOFTWARE
#define EI_CLASSIFIER_USE_AKIDA_HARDWARE 1
#endif

/**
 * @brief If more than one device is present in system
 * setting this to device index can select a proper device.
 * e.g.: set to 1 to selct /dev/akida1
 *
 */
#ifndef EI_CLASSIFIER_USE_AKIDA_HARDWARE_NO
#define EI_CLASSIFIER_USE_AKIDA_HARDWARE_NO 0
#endif

#include "model-parameters/model_metadata.h"
#include <thread>
#include "tensorflow-lite/tensorflow/lite/c/common.h"
#include "tensorflow-lite/tensorflow/lite/interpreter.h"
#include "tensorflow-lite/tensorflow/lite/kernels/register.h"
#include "tensorflow-lite/tensorflow/lite/model.h"
#include "tensorflow-lite/tensorflow/lite/optional_debug_tools.h"
#include "edge-impulse-sdk/tensorflow/lite/kernels/custom/tree_ensemble_classifier.h"
#include "edge-impulse-sdk/classifier/ei_model_types.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "tensorflow-lite/tensorflow/lite/kernels/internal/reference/softmax.h"
#undef EI_CLASSIFIER_INFERENCING_ENGINE
#define EI_CLASSIFIER_INFERENCING_ENGINE EI_CLASSIFIER_TFLITE_FULL
#include "tflite_helper.h"
#undef EI_CLASSIFIER_INFERENCING_ENGINE
#define EI_CLASSIFIER_INFERENCING_ENGINE EI_CLASSIFIER_AKIDA
#include <vector>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <limits>
#include <math.h>
#include <algorithm>
#include "pybind11/embed.h"
#include "pybind11/numpy.h"
#include "pybind11/stl.h"

namespace py = pybind11;

std::stringstream engine_info;

static py::module_ akida;
static py::object model;
static py::object model_predict;
static py::object model_forward;
static py::object device;
static bool akida_initialized = false;
static std::vector<size_t> input_shape;
static tflite::RuntimeShape softmax_shape;
static tflite::SoftmaxParams dummy_params;
static int model_input_bits = 0;
static float scale;
static int down_scale;
typedef struct {
    std::unique_ptr<tflite::FlatBufferModel> model;
    std::unique_ptr<tflite::Interpreter> interpreter;
} ei_tflite_state_t;

std::map<uint32_t, ei_tflite_state_t*> ei_tflite_instances;

bool init_akida(const uint8_t *model_arr, size_t model_arr_size, bool debug)
{
    py::module_ sys;
    py::list path;
    constexpr char model_file_path[] = "/tmp/akida_model.fbz";

    if(debug) {
        try {
            sys = py::module_::import("sys");
            path = sys.attr("path");
            ei_printf("DEBUG: sys.path:");
            for (py::handle p: path) {
                ei_printf("\t%s\n", p.cast<std::string>().c_str());
            }
        }
        catch (py::error_already_set &e) {
            ei_printf("ERR: Importing 'sys' library failed:\n%s\n", e.what());
            // as it is only for debug purposes, continue
        }
    }

    try {
        // import Python's akida module
        akida = py::module_::import("akida");
    }
    catch (py::error_already_set &e) {
        ei_printf("ERR: Importing 'akida' library failed:\n%s\n", e.what());
        return false;
    }

    if(debug) {
        std::string ver = akida.attr("__version__").cast<std::string>();
        ei_printf("DEBUG: Akida version: %s\n", ver.c_str());
    }

    py::object Model = akida.attr("Model");

    // deploy akida model file into temporary file
    std::ofstream model_file(model_file_path, std::ios::out | std::ios::binary);
    model_file.write(reinterpret_cast<const char*>(model_arr), model_arr_size);
    if(model_file.bad()) {
        ei_printf("ERR: failed to unpack model ile into %s\n", model_file_path);
        model_file.close();
        return false;
    }
    model_file.close();

    // load model
    try {
        model = Model(model_file_path);
    }
    catch (py::error_already_set &e) {
        ei_printf("ERR: Can't load model file from %s\n", model_file_path);
        ei_printf("ERR: %s\n", e.what());
        return false;
    }

    // get input shape from model
    input_shape = model.attr("input_shape").cast<std::vector<size_t>>();
    //TODO: temporarily only 3D input data is supported (see note in run_nn_inference)
    if(input_shape.size() != 3) {
        ei_printf("ERR: Unsupported input data shape. Expected 3 dimensions got %d\n", (int)input_shape.size());
        return false;
    }
    // extend input by (N, ...) - hardcoded to (1, ...)
    input_shape.insert(input_shape.begin(), (size_t)1);

    // get model input_bits
    std::vector<py::object> layers = model.attr("layers").cast<std::vector<py::object>>();
    auto input_layer = layers[0];
    model_input_bits = input_layer.attr("input_bits").cast<int>();
    if((model_input_bits != 8) && (model_input_bits != 4)) {
        ei_printf("ERR: Unsupported input_bits. Expected 4 or 8 got %d\n", model_input_bits);
        return false;
    }

    // initialize scale coefficients
    if(model_input_bits == 8) {
        scale = 255;
        down_scale = 1;
    }
    else if(model_input_bits == 4) {
        // these values are recommended by BrainChip
        scale = 15;
        down_scale = 16;
    }

    if(debug) {
        ei_printf("INFO: Model input_bits: %d\n", model_input_bits);
        ei_printf("INFO: Scale: %f\n", scale);
        ei_printf("INFO: Down scale: %d\n", down_scale);
    }

#if (defined(EI_CLASSIFIER_USE_AKIDA_HARDWARE) && (EI_CLASSIFIER_USE_AKIDA_HARDWARE == 1))
    // get list of available devices
    py::list devices = akida.attr("devices")();
    if(devices.empty() == true) {
        ei_printf("ERR: Akida device not found!\n");
        return false;
    }

    if(devices.size() > 1) {
        ei_printf("More than one device found! Using /dev/akida%d\n", EI_CLASSIFIER_USE_AKIDA_HARDWARE_NO);
        device = devices[EI_CLASSIFIER_USE_AKIDA_HARDWARE_NO];
    }
    else {
        device = devices[0];
    }

    // TODO: check if selected device is correct (compare versions)
    // power measurement not avaliable on akida1500, commenting out for now
    //device.attr("soc").attr("power_measurement_enabled") = true;


    // map model to the device
    try {
        model.attr("map")(device);
    }
    catch (py::error_already_set &e) {
        ei_printf("ERR: Can't load the ML model onto the Akida SoC\n");
        ei_printf("ERR: %s\n", e.what());
        return false;
    }
#elif (defined(EI_CLASSIFIER_USE_AKIDA_SOFTWARE) && (EI_CLASSIFIER_USE_AKIDA_SOFTWARE == 1))
#warning "Akida model will be run in SIMULATION mode (not on real hardware)!"
#else
#error "Neither EI_CLASSIFIER_USE_AKIDA_HARDWARE or EI_CLASSIFIER_USE_AKIDA_SOFTWARE are defined or set to 1"
#endif

    // init softmax shape
    std::vector<size_t> tmp = model.attr("output_shape").cast<std::vector<size_t>>();
    softmax_shape.BuildFrom(tmp);
    // dumy beta parameter for softmax purposes
    dummy_params.beta = 1;

    // get reference to predict function
    model_predict = model.attr("predict");
    model_forward = model.attr("forward");

    // clear info stream
    engine_info.str("");

    return true;
}

template<typename T>
void debug_print(const std::vector<T> vec, const int val_per_row = 3)
{
    int n = 0;
    for(auto it = vec.begin(); it != vec.end(); it++) {
        ei_printf("%f ", *it);
        if(++n > val_per_row - 1) {
            ei_printf("\n");
            n = 0;
        }
    }
}

/**
 * @brief      Do neural network inferencing over the processed feature matrix
 *
 * @param      impulse  Struct describing impulse architecture
 * @param      fmatrix  Processed matrix
 * @param      result   Output classifier results
 * @param[in]  debug    Debug output enable
 *
 * @return     The ei impulse error.
 */
EI_IMPULSE_ERROR run_nn_inference(
    const ei_impulse_t *impulse,
    ei_feature_t *fmatrix,
    uint32_t learn_block_index,
    uint32_t* input_block_ids,
    uint32_t input_block_ids_size,
    ei_impulse_result_t *result,
    void *config_ptr,
    bool debug)
{
    ei_learning_block_config_tflite_graph_t *block_config = ((ei_learning_block_config_tflite_graph_t*)config_ptr);
    ei_config_akida_graph_t *graph_config = ((ei_config_akida_graph_t*)block_config->graph_config);

    // init Python embedded interpreter (should be called once!)
    static py::scoped_interpreter guard{};

    // check if we've initialized the interpreter and device?
    if (akida_initialized == false) {
        if(init_akida(graph_config->model, graph_config->model_size, debug) == false) {
            return EI_IMPULSE_AKIDA_ERROR;
        }
        akida_initialized = true;
    }

    // according to:
    // https://doc.brainchipinc.com/api_reference/akida_apis.html#akida.Model.predict
    // input type is always uint8
    py::array_t<uint8_t> input_data(input_shape);

    /*
     * convert data to uint8 and copy features into input tensor
     * For images RGB shape is (width, height, colors)
     * For images BW shape is (width, height, 1)
     * For Audio shape is (width, height, 1) - spectrogram
     * TODO: test with other ML models/data types
     * For details see:
     * https://pybind11.readthedocs.io/en/stable/advanced/pycpp/numpy.html#direct-access
     */
    auto r = input_data.mutable_unchecked<4>();
    float temp;

    size_t mtx_size = impulse->dsp_blocks_size + impulse->learning_blocks_size;
    for (size_t i = 0; i < input_block_ids_size; i++) {
#if EI_CLASSIFIER_SINGLE_FEATURE_INPUT == 0
        uint16_t cur_mtx = input_block_ids[i];
        ei::matrix_t* matrix = NULL;

        if (!find_mtx_by_idx(fmatrix, &matrix, cur_mtx, mtx_size)) {
            ei_printf("ERR: Cannot find matrix with id %zu\n", cur_mtx);
            return EI_IMPULSE_INVALID_SIZE;
        }
#else
        ei::matrix_t* matrix = fmatrix[0].matrix;
#endif
        for (py::ssize_t x = 0; x < r.shape(1); x++) {
            for (py::ssize_t y = 0; y < r.shape(2); y++) {
                for(py::ssize_t z = 0; z < r.shape(3); z++) {
                    temp = (matrix->buffer[x * r.shape(2) * r.shape(3) + y * r.shape(3) + z] * scale);
                    temp = std::max(0.0f, std::min(temp, 255.0f));
                    r(0, x, y, z) = (uint8_t)(temp / down_scale);
                }
            }
        }
    }

    // Run inference on Akida
    uint64_t ctx_start_us = ei_read_timer_us();
    py::array_t<float> potentials;
    try {
        potentials = model_predict(input_data);
    }
    catch (py::error_already_set &e) {
        ei_printf("ERR: Inference error:\n%s\n", e.what());
        return EI_IMPULSE_AKIDA_ERROR;
    }
    // TODO: 'forward' is returning int8 or int32, but EI SDK supports int8 or float32 only
    // py::array_t<float> potentials = model_forward(input_data);
    uint64_t ctx_end_us = ei_read_timer_us();

    potentials = potentials.squeeze();

    if(debug) {
        std::string ret_str = py::str(potentials).cast<std::string>();
        ei_printf("Akida raw output:\n%s\n", ret_str.c_str());
    }

    // convert to vector of floats to make further processing much easier
    std::vector<float> potentials_v;// = potentials.cast<std::vector<float>>();

    // TODO: output conversion depending on output shape?
    if (graph_config->object_detection_last_layer == EI_CLASSIFIER_LAST_LAYER_UNKNOWN) {
        potentials_v = potentials.squeeze().cast<std::vector<float>>();
    }
    else {
        // TODO: output from AkidaNet/MobileNet is always N x M x P (3 dimensions)?
        auto q = potentials.unchecked<>();
        for (py::ssize_t x = 0; x < q.shape(0); x++) {
            for (py::ssize_t y = 0; y < q.shape(1); y++) {
                for(py::ssize_t z = 0; z < q.shape(2); z++) {
                    potentials_v.push_back(q(x, y, z));
                }
            }
        }
    }

    if(graph_config->object_detection_last_layer != EI_CLASSIFIER_LAST_LAYER_YOLOV2) {
        // apply softmax, becuase Akida is not supporting this operation
        tflite::reference_ops::Softmax(dummy_params, softmax_shape, potentials_v.data(), softmax_shape, potentials_v.data());
    }

    if(debug == true) {
        ei_printf("After softmax:\n");
        debug_print(potentials_v);
    }

    float active_power = 0;
#if (defined(EI_CLASSIFIER_USE_AKIDA_HARDWARE))
    // the ADK1500 does not have power measurements, commenting out for now
    // TODO: check betweewn Akida1000 and Akida1500 or reanble when available
    // power measurement post-processing
    //float floor_power = device.attr("soc").attr("power_meter").attr("floor").cast<float>();
    //py::array pwr_events = device.attr("soc").attr("power_meter").attr("events")();
    //auto events = pwr_events.mutable_unchecked<py::object>();
    //for (py::ssize_t i = 0; i < events.shape(0); i++) {
    //    active_power += events(i).attr("power").cast<float>();
    //}
    //active_power = (active_power/pwr_events.size()) - floor_power;
#endif

    result->timing.classification_us = ctx_end_us - ctx_start_us;
    result->timing.classification = (int)(result->timing.classification_us / 1000);

    // clear info
    engine_info.str("");
    engine_info << "Power consumption: " << std::fixed << std::setprecision(2) << active_power << " mW\n";
    engine_info << "Inferences per second: " << (1000000 / result->timing.classification_us);

    size_t output_size = potentials_v.size();

    result->_raw_outputs[learn_block_index].matrix = new matrix_t(1, output_size);
    result->_raw_outputs[learn_block_index].blockId = block_config->block_id;

    for (size_t i = 0; i < output_size; i++) {
        result->_raw_outputs[learn_block_index].matrix->buffer[i] = potentials_v[i];
    }

    return EI_IMPULSE_OK;
}

/**
 * Construct a tflite interpreter (creates it if needed)
 */
static EI_IMPULSE_ERROR get_interpreter(ei_learning_block_config_tflite_graph_t *block_config, tflite::Interpreter **interpreter) {
    // not in the map yet...
    if (!ei_tflite_instances.count(block_config->block_id)) {
        ei_config_tflite_graph_t *graph_config = (ei_config_tflite_graph_t*)block_config->graph_config;
        ei_tflite_state_t *new_state = new ei_tflite_state_t();

        auto new_model = tflite::FlatBufferModel::BuildFromBuffer((const char*)graph_config->model, graph_config->model_size);
        new_state->model = std::move(new_model);
        if (!new_state->model) {
            ei_printf("Failed to build TFLite model from buffer\n");
            return EI_IMPULSE_TFLITE_ERROR;
        }

        tflite::ops::builtin::BuiltinOpResolver resolver;
#if EI_CLASSIFIER_HAS_TREE_ENSEMBLE_CLASSIFIER
        resolver.AddCustom("TreeEnsembleClassifier",
            tflite::ops::custom::Register_TREE_ENSEMBLE_CLASSIFIER());
#endif
        tflite::InterpreterBuilder builder(*new_state->model, resolver);
        builder(&new_state->interpreter);

        if (!new_state->interpreter) {
            ei_printf("Failed to construct interpreter\n");
            return EI_IMPULSE_TFLITE_ERROR;
        }

        if (new_state->interpreter->AllocateTensors() != kTfLiteOk) {
            ei_printf("AllocateTensors failed\n");
            return EI_IMPULSE_TFLITE_ERROR;
        }

        int hw_thread_count = (int)std::thread::hardware_concurrency();
        hw_thread_count -= 1; // leave one thread free for the other application
        if (hw_thread_count < 1) {
            hw_thread_count = 1;
        }

        if (new_state->interpreter->SetNumThreads(hw_thread_count) != kTfLiteOk) {
            ei_printf("SetNumThreads failed\n");
            return EI_IMPULSE_TFLITE_ERROR;
        }

        ei_tflite_instances.insert(std::make_pair(block_config->block_id, new_state));
    }

    auto tflite_state = ei_tflite_instances[block_config->block_id];
    *interpreter = tflite_state->interpreter.get();
    return EI_IMPULSE_OK;
}


extern "C" EI_IMPULSE_ERROR run_nn_inference_from_dsp(
    ei_learning_block_config_tflite_graph_t *block_config,
    signal_t *signal,
    matrix_t *output_matrix)
{
    tflite::Interpreter *interpreter;
    auto interpreter_ret = get_interpreter(block_config, &interpreter);
    if (interpreter_ret != EI_IMPULSE_OK) {
        return interpreter_ret;
    }

    TfLiteTensor *input = interpreter->input_tensor(0);
    TfLiteTensor *output = interpreter->output_tensor(0);

    if (!input) {
        return EI_IMPULSE_INPUT_TENSOR_WAS_NULL;
    }
    if (!output) {
        return EI_IMPULSE_OUTPUT_TENSOR_WAS_NULL;
    }

    auto input_res = fill_input_tensor_from_signal(signal, input);
    if (input_res != EI_IMPULSE_OK) {
        return input_res;
    }

    TfLiteStatus status = interpreter->Invoke();
    if (status != kTfLiteOk) {
        ei_printf("ERR: interpreter->Invoke() failed with %d\n", status);
        return EI_IMPULSE_TFLITE_ERROR;
    }

    auto output_res = fill_output_matrix_from_tensor(output, output_matrix);
    if (output_res != EI_IMPULSE_OK) {
        return output_res;
    }

    // on Linux we're not worried about free'ing (for now)

    return EI_IMPULSE_OK;
}

__attribute__((unused)) int extract_tflite_features(signal_t *signal, matrix_t *output_matrix, void *config_ptr, const float frequency) {

    ei_dsp_config_tflite_t *dsp_config = (ei_dsp_config_tflite_t*)config_ptr;

    ei_config_tflite_graph_t ei_config_tflite_graph_0 = {
        .implementation_version = 1,
        .model = dsp_config->model,
        .model_size = dsp_config->model_size,
        .arena_size = dsp_config->arena_size
    };

    const uint8_t ei_output_tensor_indices[1] = { 0 };
    const uint8_t ei_output_tensor_size = 1;

    ei_learning_block_config_tflite_graph_t ei_learning_block_config = {
        .implementation_version = 1,
        .block_id = dsp_config->block_id,
        .output_tensors_indices = ei_output_tensor_indices,
        .output_tensors_size = ei_output_tensor_size,
        .quantized = 0,
        .compiled = 1,
        .graph_config = &ei_config_tflite_graph_0
    };

    auto x = run_nn_inference_from_dsp(&ei_learning_block_config, signal, output_matrix);
    if (x != 0) {
        return x;
    }

    return EIDSP_OK;
}

#endif // EI_CLASSIFIER_INFERENCING_ENGINE == EI_CLASSIFIER_AKIDA
#endif /* EI_CLASSIFIER_INFERENCING_ENGINE_AKIDA_H */
