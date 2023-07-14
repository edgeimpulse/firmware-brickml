/***********************************************************************************************************************
 * Copyright [2017] RELOC s.r.l. - All rights strictly reserved
 * This Software is provided for evaluation purposes; any other use — including reproduction, modification, use for
 * a commercial product, distribution, or republication — without the prior written permission of the Copyright holder
 * is strictly prohibited. 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
 * OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 **********************************************************************************************************************/

#ifndef SF_BNO055_PRIVATE_API_H
#define SF_BNO055_PRIVATE_API_H

/***********************************************************************************************************************
 * Private Instance API Functions. DO NOT USE! Use functions through Interface API structure instead.
 **********************************************************************************************************************/
fsp_err_t SF_BNO055_Open(sf_bno055_ctrl_t * const p_ctrl,
                        sf_bno055_cfg_t const * const p_cfg);
fsp_err_t SF_BNO055_ReadAccelerometer (  sf_bno055_ctrl_t * const p_ctrl,
                                        float * x_value,
                                        float * y_value,
                                        float * z_value);
fsp_err_t SF_BNO055_ReadGyroscope (  sf_bno055_ctrl_t * const p_ctrl,
                                    float * x_value,
                                    float * y_value,
                                    float * z_value);
fsp_err_t SF_BNO055_ReadMagnetometer (   sf_bno055_ctrl_t * const p_ctrl,
                                        float * x_value,
                                        float * y_value,
                                        float * z_value);
fsp_err_t SF_BNO055_ReadEulerAngles (    sf_bno055_ctrl_t * const p_ctrl,
                                        float * pitch_value,
                                        float * roll_value,
                                        float * heading_value);
fsp_err_t SF_BNO055_ReadQuaternions (    sf_bno055_ctrl_t * const p_ctrl,
                                        float * w_value,
                                        float * x_value,
                                        float * y_value,
                                        float * z_value );
fsp_err_t SF_BNO055_ReadTemperature (sf_bno055_ctrl_t * const p_ctrl,
                                    float * temperature);

fsp_err_t SF_BNO055_Reset(sf_bno055_ctrl_t * const p_ctrl);
fsp_err_t SF_BNO055_Close(sf_bno055_ctrl_t * const p_ctrl);
fsp_err_t SF_BNO055_VersionGet(void * const p_version);

#endif /* SF_BNO055_PRIVATE_API_H */
