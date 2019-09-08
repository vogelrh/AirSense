#!/bin/sh

set  -eu

. ./make.config

if [ ! -d "${BSEC_DIR}" ]; then
  echo 'BSEC directory missing.'
  exit 1
fi

if [ ! -d "${MQTT_DIR}" ]; then
  echo 'MQTT directory missing.'
  exit 1
fi

if [ ! -d "${CONFIG_DIR}" ]; then
  mkdir "${CONFIG_DIR}"
fi

STATEFILE="${CONFIG_DIR}/bsec_iaq.state"
if [ ! -f "${STATEFILE}" ]; then
  touch "${STATEFILE}"
fi

BIN_DIR="${CONFIG_DIR}/bin"
if [ ! -d "${BIN_DIR}" ]; then
  mkdir "${BIN_DIR}"
fi

echo 'Patching...'
dir="${BSEC_DIR}/examples"
patch='patches/eCO2+bVOCe.diff'
if patch -N --dry-run --silent -d "${dir}/" \
  < "${patch}" 2>/dev/null
then
  patch -d "${dir}/" < "${patch}"
else
  echo 'Already applied.'
fi

echo 'Compiling...'
cc -Wall -Wno-unused-but-set-variable -Wno-unused-variable -static \
  -std=c99 -pedantic \
  -iquote"${BSEC_DIR}"/API \
  -iquote"${BSEC_DIR}"/algo/${ARCH} \
  -iquote"${BSEC_DIR}"/examples \
  -isystem"${MQTT_DIR}"/include \
  -isystem"${MQTT_DIR}"/examples/templates \
  "${BSEC_DIR}"/API/bme680.c \
  "${BSEC_DIR}"/examples/bsec_integration.c \
  "${MQTT_DIR}"/src/mqtt_pal.c \
  "${MQTT_DIR}"/src/mqtt.c \
  ./pms5003.c \
  ./airsense.c \
  -L"${BSEC_DIR}"/algo/"${ARCH}" -lalgobsec \
  -lm -lrt -lpthread \
  -o "${BIN_DIR}/airsense"
echo 'Compiled.'

cp "${BSEC_DIR}"/config/"${CONFIG}"/bsec_iaq.config "${CONFIG_DIR}"/
echo 'Copied config.'

