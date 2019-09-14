#!/bin/sh
# Packages only the required files for a target architecture
# for compilation on the target system.
# The target system must have a gcc compiler and a means of 
# unpacking a .tar.gz file.

BSEC_DIR='./vendor_src/BSEC_1.4.7.4_Generic_Release'
MQTT_DIR='./vendor_src/MQTT-C-master'
PACK_DIR='./prepack'
COPY_DIR='./prepack/airsense'
TAR_DIR='airsense'

#**************************************************
# Modify the following depending on architecture 
# and BSEC configuration
#
# Version is "normal_version" or "light_version"
VERSION='normal_version'

# Base Architecture e.g. RaspberryPi (see ${BSEC_DIR}/algo/${VERSION}/bin)
BASE_ARCH='RaspberryPI'

# Sub architecture e.g. PiZero_ArmV6-32bits
SUB_ARCH='PiZero_ArmV6-32bits'

# CONFIG is the BSEC configuration file
CONFIG='generic_33v_3s_4d'
#**************************************************

if [ ! -d "${PACK_DIR}" ]; then
  mkdir "${PACK_DIR}"
fi

if [ ! -d "${COPY_DIR}" ]; then
  mkdir "${COPY_DIR}"
else
  rm -r "${COPY_DIR}"
  mkdir "${COPY_DIR}"  
fi

if [ ! -d "${PACK_DIR}/${SUB_ARCH}" ]; then
  mkdir "${PACK_DIR}/${SUB_ARCH}"
else
  rm -r "${PACK_DIR}/${SUB_ARCH}"
  mkdir "${PACK_DIR}/${SUB_ARCH}"
fi

#copy the required files
echo "Copying files to temp directory..."
cp -v "${MQTT_DIR}"/src/*.c "${COPY_DIR}"
cp -v "${MQTT_DIR}"/include/*.h "${COPY_DIR}"
cp -v "${MQTT_DIR}"/examples/templates/posix_sockets.h "${COPY_DIR}"
cp -v "${BSEC_DIR}"/API/bme680* "${COPY_DIR}"
cp -v "${BSEC_DIR}"/examples/bsec_integration* "${COPY_DIR}"
cp -v "${BSEC_DIR}/algo/${VERSION}/bin/${BASE_ARCH}/${SUB_ARCH}"/*.h "${COPY_DIR}"
cp -v "${BSEC_DIR}/algo/${VERSION}/bin/${BASE_ARCH}/${SUB_ARCH}"/libalgobsec.a "${COPY_DIR}"
cp -v "${BSEC_DIR}"/config/"${CONFIG}"/bsec_iaq.config "${COPY_DIR}"
cp -v ./airsense.c  "${COPY_DIR}"
cp -v ./pms5003.*  "${COPY_DIR}"

#Generate make file script
echo "Creating make file script"
FILE="${COPY_DIR}"/make.sh
cat <<EOM >$FILE
if [ ! -d ./bin ]; then
  mkdir ./bin
fi

cc -Wall -Wno-unused-but-set-variable -Wno-unused-parameter \
 -Wno-unused-variable -Wno-duplicate-decl-specifier -Wno-pointer-arith \
 -std=gnu99 -pedantic \
 -isystem"." \
 ./bme680.c \
 ./bsec_integration.c \
 ./mqtt_pal.c \
 ./mqtt.c \
 ./pms5003.c \
 ./airsense.c \
 -L"." -lalgobsec \
 -lm -lrt -lpthread \
 -o bin/airsense

cp ./bsec_iaq.config ./bin
touch ./bin/bsec_iaq.state
EOM
chmod +x "${COPY_DIR}"/make.sh

#create the tar ball
echo "Creating tar ball"
tar -C $PACK_DIR -czvf "${PACK_DIR}/${SUB_ARCH}/airsense.tar.gz" $TAR_DIR

#cleanup
rm -r "${COPY_DIR}"
echo "*************************************************************************"
echo "Packing complete"
echo "\"scp\" the file:"
echo "${PACK_DIR}/${SUB_ARCH}/airsense.tar.gz "
echo "to the destination system."
echo "Expand the file and execute the make.sh script."
echo "Copy the contents of the ./bin directory to the final location on the"
echo "destination system."
echo "*************************************************************************"
