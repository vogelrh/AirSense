# AirSense

A Linux C-based application that periodically reads air quality data from two sensors (Bosh BME680 & Plantower PMS5003) and
sends the data across a MQTT channel as a JSON object.

The app reads the BME680 sensor with the BSEC library on Linux (e.g. Raspberry Pi) and reads the PMS5003 sensor with a custom library included in the project. 
Optionally the app can be configured to only output data from the BME680 sensor.

## Credit
This project is a fork of an [alexh.name](https://github.com/alexh-name/bsec_bme680_linux) project with the addition of a MQTT libary by [Liam Bindle](https://github.com/LiamBindle/MQTT-C) and a PMS5003 sensor library developed by myself.


## Intro

This app is designed to run on a Linxu SoC such as the Raspberry Pi Zero and will continously send air quality data from the 
[BME680 sensor](https://www.bosch-sensortec.com/en/bst/products/all_products/bme680) and the [Plantower PMS5003](http://www.aqmd.gov/docs/default-source/aq-spec/resources-page/plantower-pms5003-manual_v2-3.pdf) sensor across a MQTT channel.

For the BME680 sensor, it utilizes the
[BSEC library](https://www.bosch-sensortec.com/bst/products/all_products/bsec),
which reads calibrated environment values including an actual Indoor Air Quality (IAQ) score, and it makes use of a
[Bosch's provided driver](https://github.com/BoschSensortec/BME680_driver).

## Prerequisites

After cloning this project, first create a directory `vendor_src`. Then download the
[dBSEC software package from Bosch](https://www.bosch-sensortec.com/bst/products/all_products/bsec)
and unpack it into `./vendor_src`. Next download the [MQTT library](https://github.com/LiamBindle/MQTT-C) from github as a zip file and also unpack it into the `.vendor_src` directory.

## Configure and Compile

Optionally make changes to `make.config`. Change the `ARCH` parameter for different host architectures, while the `CONFIG` parameter determines the library configuration. (See the BSEC Integration Guide included in the BSEC source files for more details on the `CONFIG` options).

To compile the code locally: `./make.sh`

## Compiling for a Target System
If your development system which you downloaded this code to is not 
going to be your target deployment system, rather than dealing with a complex cross-compiler environment, I take a simplistic approach.

There is a script in this project `prepack.sh` which will bundle all of the required files for the target system into a tar.gz file which can then be sent to the target system, expanded and then compiled on the target system. Of course, for this to work you will need two things:
  * A GCC compiler on the target system
  * A BSEC library file for the target system.

The bundled code will be placed in a subdirectory of the ./prepack directory. For example if you are bundling for a Pi Zero target system, after configuring and running the `prepack.sh` script, you will find the bundle in the directory `./prepack/PiZero_ArmV6-32bits`.

If you have ssh set up on the target system you can send the file to the target using the `scp` command.

### Configuring `prepack.sh` for a target system.
There are four script variables near the top of `prepack.sh` that have to be configured for a specific target system. These variables tell the script where to find the precompiled BSEC libary for the target system and the BSEC configuration to use. The variables are: 

  * `VERSION` - can be either `normal_version` or `light_version`.
  * `BASE_ARCH` - the high-level target e.g. RaspberryPi, armcc, avr, etc.
  * `SUB_ARCH` - the specific system architecture e.g. PiThree_ArmV8-a-64bits or PiZero_ArmV6-32bits.
  * `CONFIG` - the specifc BSEC configuration file to use.

See the documentation enclosed with the BSEC code for more detail.

One final note, be sure to either execute the `make.sh` file or manually run the lines of code that patches the BSEC example code before you run the `prepack.sh` script.

## Usage

The program is designed to run continously (ideally at boot). There are a number of command line options that can be used to configure the program. If you do not specify a particular option, a default value will be used. The options and their defaults are as follows:

| Option |   Value    |    Description                   |
|:-------:|:----------:|----------------------------------|
| -s | | If specified then the secondary I2C address will be used |
| -d | | Debug mode, writes information to `stdout`. |
| -u | | Disable the PMS5003 sensor. Only output data from the BSE630. |
| -b | *address* | The address of the MQTT broker / server. Default: `test.mosquitto.org` |
| -p | *port* | The port number of the MQTT broker / server. Default: `1883` |
| -t | *topic* | The MQTT topic to publish to. Default: `AirSenseData` |
| -i| *sensor id* | The id of the sensor system sending the data. This is transmitted with the data to identify the source of the data  Default: *system machine name* |
| -m | *number* | The message output rate as a multiple of the intrinsic BSEC sampling rate. The intrinsic sampling rate is determined by the BSEC configuration file used. The default value is `2` which is twice the intrinsic rate. |
| -o | temperature | An offset temperature value. This value is applied to the temperature reading to compensate for any heat given off by the electronics around the sensor. Default: `5.0` °C. |

### Program Output

Any error messages will be written to  `stderr`. Nothing will be written to `stdout` unless the `-d` option is specifed. All sensor data will be sent across the MQTT channel as a string representation of a JSON object. *Programming note: the JSON string will contain a C string terminator (zero) at the end and must be removed from the receiving end before parsing the JSON*

The JSON Object will be similar to this:

```
{
  "sensor_id": "masterpi",
  "time_stamp": "2019-09-14 08:06:12",
  "IAQ": 95.86,
  "iaq_accuracy": 3,
  "temperature": 24.13,
  "humidity": 49.76,
  "pressure": 991.71,
  "gas_resistance": 757746,
  "bVOCe": 1.23,
  "eCO2": 845.39,
  "bse_status": 0,
  "pm1cf": 4,
  "pm2_5cf": 6,
  "pm10cf": 8,
  "pm1at": 4,
  "pm2_5at": 6,
  "pm10at": 8,
  "gt0_3": 972,
  "gt0_5": 265,
  "gt1": 44,
  "gt2_5": 4,
  "gt5": 3,
  "gt10": 1,
  "pms_status": 0
}
```
If the -u flag is set then the output would be similar to the following:
```
{
  "sensor_id": "slavepi01",
  "time_stamp": "2019-09-14 08:06:13",
  "IAQ": 129.66,
  "iaq_accuracy": 1,
  "temperature": 18.73,
  "humidity": 75.14,
  "pressure": 992.95,
  "gas_resistance": 189115,
  "bVOCe": 1,
  "eCO2": 707.17,
  "bse_status": 0
}
```
Where:
* IAQ - Index for Air Quality between 0 (Excellent) and 500 (Extremely Bad). 
* iaq_accuracy - Accuracy of the IAQ score from 0 (low) to 3 (high).
* tem
* bse_status - Return value of the BSEC library

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

