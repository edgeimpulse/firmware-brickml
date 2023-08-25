set -e

SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"

echo "Removing $SCRIPTPATH/src/edge-impulse//edge-impulse-sdk/"
rm -rf $SCRIPTPATH/src/edge-impulse/edge-impulse-sdk/
echo "Removing $SCRIPTPATH/src/edge-impulse/edge-impulse-sdk/ OK"

echo "Copying new version of SDK"
cp -r $SCRIPTPATH/../edge-impulse-sdk $SCRIPTPATH/src/edge-impulse/edge-impulse-sdk/
echo "Copying new version of SDK OK"

echo "Removing $SCRIPTPATH/src/edge-impulse/firmware-sdk/"
rm -rf $SCRIPTPATH/src/edge-impulse/firmware-sdk/
echo "Removing $SCRIPTPATH/src/edge-impulse/firmware-sdk/ OK"

echo "Copying new version of Firmware SDK"
cp -r $SCRIPTPATH/../firmware-sdk $SCRIPTPATH/src/edge-impulse/firmware-sdk/
echo "Copying new version of Firmware SDK OK"
