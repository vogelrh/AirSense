# AirSense

A Linux C-based application that periodically reads air quality data from two sensors (Bosh BME680 & Plantower PMS5003) and
sends the data across a MQTT channel.

The app reads the BME680 sensor with the BSEC library on Linux (e.g. Raspberry Pi) and reads the PMS5003 sensor
with a custom library included in the project.

## Credit
This project is a fork of [alexh.name](https://github.com/alexh-name/bsec_bme680_linux) with the addition of a MQTT libary
by [Liam Bindle](https://github.com/LiamBindle/MQTT-C) and a PMS5003 sensor library developed by myself.


## Intro

This app is designed to run on a Linxu SOC such as the Raspberry Pi Zero and will continously send air quality data from the 
[BME680 sensor](https://www.bosch-sensortec.com/en/bst/products/all_products/bme680) and the [Plantower PMS5003](http://www.aqmd.gov/docs/default-source/aq-spec/resources-page/plantower-pms5003-manual_v2-3.pdf) sensor across a MQTT channel.

For the BME680 sensor, it utilizes the
[BSEC library](https://www.bosch-sensortec.com/bst/products/all_products/bsec),
which reads calibrated environment values including an actual Indoor Air Quality (IAQ) score, and it makes use of a
[Bosch's provided driver](https://github.com/BoschSensortec/BME680_driver).

## Prerequisites

After cloning this project, first create a directory `vendor_src`. Then download the
[dBSEC software package from Bosch](https://www.bosch-sensortec.com/bst/products/all_products/bsec)
and unpack it into `./vendor_src`. Nex download the [MQTT library](https://github.com/LiamBindle/MQTT-C) from github as a zip file and also unpack it into the `.vendor_src` directory.

## Configure and Compile

Optionally make changes to `make.config`. Change the `ARCH` parameter for different host architectures, while the `CONFIG` parameter determines the library configuration. (See the BSEC Integration Guide for more details).

Depending on how your sensor is embedded it might be surrounded by other components giving off heat. Use an offset in °C in `airsense.c` to compensate. The default is 5 °C:
```
#define temp_offset (5.0f)
```
XXX LEFT OFF HERE

To compile: `./make.sh`

## MQTT-C library


## Usage

Output will be similar to this:

```
$ ./bsec_bme680
2017-12-27 18:47:21,[IAQ (1)]: 33.96,[T degC]: 19.61,[H %rH]: 46.41,[P hPa]: 983.39,[G Ohms]: 540924.00,[S]: 0
2017-12-27 18:47:24,[IAQ (1)]: 45.88,[T degC]: 19.61,[H %rH]: 46.41,[P hPa]: 983.41,[G Ohms]: 535321.00,[S]: 0
2017-12-27 18:47:26,[IAQ (1)]: 40.65,[T degC]: 19.60,[H %rH]: 46.45,[P hPa]: 983.39,[G Ohms]: 537893.00,[S]: 0
2017-12-27 18:47:29,[IAQ (1)]: 30.97,[T degC]: 19.60,[H %rH]: 46.42,[P hPa]: 983.41,[G Ohms]: 542672.00,[S]: 0
```
* IAQ (n) - Accuracy of the IAQ score from 0 (low) to 3 (high).
* S: n - Return value of the BSEC library

It can easily be modified in the `output_ready` function.

The BSEC library is supposed to create an internal state of calibration with
increasing accuracy over time. Each 10.000 samples it will save the internal
calibration state to `./bsec_iaq.state` (or wherever you specify the config
directory to be) so it can pick up where it was after interruption.

## Further

You can find a growing list of tools to further use and visualize the data
[here](https://github.com/alexh-name/bme680_outputs).

## Troubleshooting

### bsec_bme680 just quits without a message

Your bsec_iaq.state file might be corrupt or incompatible after an update of the
BSEC library. Try (re)moving it.

