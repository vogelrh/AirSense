/* Copyright (C) 2019 Bob Vogel */
/* Copyright (C) 2017 alexh.name */
/* I2C code by twartzek 2017 */
/* MQTT code by Liam Bindle 2018 */

/*
 * Read the BME680 sensor with the BSEC library by running an endless loop in
 * the bsec_iot_loop() function under Linux.
 *
 */

/*#define _POSIX_C_SOURCE 200809L*/
#define _XOPEN_SOURCE 700

/* header files */

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <time.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>
#include <mqtt.h>
#include <posix_sockets.h>
#include "bsec_integration.h"
#include "pms5003.h"

/* definitions  Ultimately we want these in a config file*/

//#define DESTZONE "TZ=EDT"
#define DEF_TEMP_OFFSET (5.0f)
#define sample_rate_mode (BSEC_SAMPLE_RATE_LP)
#define DEF_SENSOR_ID "PiAirQ01"
#define DEF_ADDR "test.mosquitto.org"
#define DEF_PORT "1883"
#define DEF_TOPIC "AirSenseData"
#define DEF_SAMPLE_MULTIPLIER 2 //sample rate = DEF_SAMPLE_MULTIPLIER * 3 seconds (intrinsic lib sample rate)

int g_i2cFid; // I2C Linux device handle
int i2c_address;
const char* sensor_id;
const char* topic;
char *filename_state = "bsec_iaq.state";
char *filename_iaq_config = "bsec_iaq.config";
struct mqtt_client client; //MQTT client
int sample_count;
int sample_multiplier;
int debug = 0;
int disable5003 = 0;
int sockfd;
float temp_offset;
/*
 * System specific implementation of sleep function
 *
 * param[in]       t_ms    time in milliseconds
 *
 * return          none
 */
void _sleep(uint32_t t_ms)
{
  struct timespec ts;
  ts.tv_sec = 0;
  /* mod because nsec must be in the range 0 to 999999999 */
  ts.tv_nsec = (t_ms % 1000) * 1000000L;
  nanosleep(&ts, NULL);
}
// open the Linux device
void i2cOpen()
{
  g_i2cFid = open("/dev/i2c-1", O_RDWR);
  if (g_i2cFid < 0) {
    perror("i2cOpen");
    exit(1);
  }
}

// close the Linux device
void i2cClose()
{
  close(g_i2cFid);
}

// set the I2C slave address for all subsequent I2C device transfers
void i2cSetAddress(int address)
{
  if (ioctl(g_i2cFid, I2C_SLAVE, address) < 0) {
    perror("i2cSetAddress");
    exit(1);
  }
}
/**
 * @brief Safelty closes the sockfd, I2C and cancels the client_daemon before exit. 
 */
void exit_airsense(int status, int sockfd, pthread_t *client_daemon)
{
    if (sockfd != -1) close(sockfd);
    if (client_daemon != NULL) pthread_cancel(*client_daemon);
    i2cClose();
    pms_close();
    fprintf(stderr,"Exiting with status %d", status);
    exit(status);
}
/* MQTT functions */

/**
 * @brief Liam Bindle's client's refresher function. This function triggers back-end routines to 
 *        handle ingress/egress traffic to the broker.
 * 
 * @note All this function needs to do is call \ref __mqtt_recv and 
 *       \ref __mqtt_send every so often. I've picked 100 ms meaning that 
 *       client ingress/egress traffic will be handled every 100 ms.
 *       Also replaced usleep with the local function _sleep
 */
void* client_refresher(void* client)
{
    while(1) 
    {
        mqtt_sync((struct mqtt_client*) client);
        _sleep(50);
    }
    return NULL;
}
void publish_callback(void** unused, struct mqtt_response_publish *published) 
{
    /* not used for this app */
    if (debug) {
      printf("** publish_callback called **\nDup: %d\nQOS level: %d\nRetain: %d\n", 
             published->dup_flag, published->qos_level, published->retain_flag);
      printf("Topic: %.*s\n****\n", published->topic_name_size, (const char*)published->topic_name);       
    }
}

/* BME680 functions */

/*
 * Write operation in either I2C or SPI
 *
 * param[in]        dev_addr        I2C or SPI device address
 * param[in]        reg_addr        register address
 * param[in]        reg_data_ptr    pointer to the data to be written
 * param[in]        data_len        number of bytes to be written
 *
 * return          result of the bus communication function
 */
int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr,
                 uint16_t data_len)
{
  int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

  uint8_t reg[16];
  reg[0]=reg_addr;
  int i;

  for (i=1; i<data_len+1; i++)
    reg[i] = reg_data_ptr[i-1];

  if (write(g_i2cFid, reg, data_len+1) != data_len+1) {
    perror("user_i2c_write");
    rslt = 1;
    exit(1);
  }

  return rslt;
}

/*
 * Read operation in either I2C or SPI
 *
 * param[in]        dev_addr        I2C or SPI device address
 * param[in]        reg_addr        register address
 * param[out]       reg_data_ptr    pointer to the memory to be used to store
 *                                  the read data
 * param[in]        data_len        number of bytes to be read
 *
 * return          result of the bus communication function
 */
int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr,
                uint16_t data_len)
{
  int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

  uint8_t reg[1];
  reg[0]=reg_addr;

  if (write(g_i2cFid, reg, 1) != 1) {
    perror("user_i2c_read_reg");
    rslt = 1;
  }

  if (read(g_i2cFid, reg_data_ptr, data_len) != data_len) {
    perror("user_i2c_read_data");
    rslt = 1;
  }

  return rslt;
}

/*
 * Capture the system time in microseconds
 *
 * return          system_current_time    system timestamp in microseconds
 */
int64_t get_timestamp_us()
{
  struct timespec spec;
  //clock_gettime(CLOCK_REALTIME, &spec);
  /* MONOTONIC in favor of REALTIME to avoid interference by time sync. */
  clock_gettime(CLOCK_MONOTONIC, &spec);

  int64_t system_current_time_ns = (int64_t)(spec.tv_sec) * (int64_t)1000000000
                                   + (int64_t)(spec.tv_nsec);
  int64_t system_current_time_us = system_current_time_ns / 1000;

  return system_current_time_us;
}

/*
 * Callback handling of the BSE680 ready outputs. Sends data over MQTT channel in JSON format.
 * After retrieving sensor data from the BSE680, it will read the PMS5003 data from the UART.
 *
 * param[in]       timestamp       time in microseconds
 * param[in]       iaq             IAQ signal
 * param[in]       iaq_accuracy    accuracy of IAQ signal
 * param[in]       temperature     temperature signal
 * param[in]       humidity        humidity signal
 * param[in]       pressure        pressure signal
 * param[in]       raw_temperature raw temperature signal
 * param[in]       raw_humidity    raw humidity signal
 * param[in]       gas             raw gas sensor signal
 * param[in]       bsec_status     value returned by the bsec_do_steps() call
 * param[in]       static_iaq      unscaled indoor-air-quality estimate
 * param[in]       co2_equivalent  CO2 equivalent estimate [ppm]
 * param[in]       breath_voc_equivalent  breath VOC concentration estimate [ppm]
 *
 * return          none
 */
void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy,
                  float temperature, float humidity, float pressure,
                  float raw_temperature, float raw_humidity, float gas,
                  bsec_library_return_t bsec_status,
                  float static_iaq, float co2_equivalent,
                  float breath_voc_equivalent)
{
  if (debug) {
    printf("Output ready\n");
  }
  // Is it time to output?
  sample_count++;
  if (sample_count < sample_multiplier) {
    return;
  }
  sample_count = 0;
  time_t t = time(NULL);
  struct tm tm = *localtime(&t);
  int pstat;
  float di = temperature - 0.55 * (1 - 0.01 * humidity) * (temperature - 14.5);

  PMS5003_DATA  pms = {0};
  if (!disable5003) {
    pstat = read_pms5003_data(&pms);
    if (pstat != UART_OK) {
      output_uart_code(pstat);
    }
  }
  /*
   * Send the data out the MQTT channel
   * 
   * Note: to create the JSON string I wanted to use the safer asprintf function. 
   * However on the Raspian (Jessie) the function does not work correctly
   * for the format string shown below. The numeric values where wrong after the time parameters.
   * The exact same format string works fine with sprintf.
   */ 
  char message[2048]; //over kill
  if (disable5003) {
    sprintf(message,
           "{\"sensor_id\":\"%s\",\"time_stamp\":\"%d-%02d-%02d %02d:%02d:%02d\",\"IAQ\":%.2f,\"iaq_accuracy\":%d,\"T\":%.2f,\"RH\":%.2f,\"P\":%.2f,\"DI\":%.0f,\"gas\":%.0f,\"bVOCe\":%.2f,\"eCO2\":%.2f,\"bstat\":%d}",
                      sensor_id, tm.tm_year + 1900,tm.tm_mon + 1,tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, 
                      iaq, iaq_accuracy, temperature, humidity, pressure / 100, di, gas, breath_voc_equivalent, co2_equivalent, bsec_status);

  } else {
    sprintf(message,
           "{\"sensor_id\":\"%s\",\"time_stamp\":\"%d-%02d-%02d %02d:%02d:%02d\",\"IAQ\":%.2f,\"iaq_accuracy\":%d,\"T\":%.2f,\"RH\":%.2f,\"P\":%.2f,\"DI\":%.0f,\"gas\":%.0f,\"bVOCe\":%.2f,\"eCO2\":%.2f,\"bstat\":%d,\"pm1cf\":%d,\"pm2_5cf\":%d,\"pm10cf\":%d,\"pm1at\":%d,\"pm2_5at\":%d,\"pm10at\":%d,\"gt0_3\":%d,\"gt0_5\":%d,\"gt1\":%d,\"gt2_5\":%d,\"gt5\":%d,\"gt10\":%d,\"pstat\":%d}",
                      sensor_id, tm.tm_year + 1900,tm.tm_mon + 1,tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, 
                      iaq, iaq_accuracy, temperature, humidity, pressure / 100, di, gas, breath_voc_equivalent, co2_equivalent, bsec_status,
                      pms.pm1cf, pms.pm2_5cf, pms.pm10cf, pms.pm1at, pms.pm2_5at, pms.pm10at, pms.gt0_3,
                      pms.gt0_5, pms.gt1, pms.gt2_5, pms.gt5, pms.gt10, pstat);

  }
  int mcnt = strlen(message);
  if (debug) {
    printf("Ready to publish\n");
  }
  int stat = (int)mqtt_publish(&client, topic, message, mcnt + 1, MQTT_PUBLISH_QOS_1);
  if (debug) {
    printf("Publish status: %d\n", stat);
  }
  // check for errors
  if (client.error != MQTT_OK) {
    fprintf(stderr, "MQTT error: %s\n", mqtt_error_str(client.error));
    exit_airsense(EXIT_FAILURE, sockfd, NULL);   
  }

  /* for debuging */
  if (debug) {
    printf(message);
    printf("\r\n");
    fflush(stdout);
  }
}

/*
 * Load binary file from non-volatile memory into buffer
 *
 * param[in,out]   state_buffer    buffer to hold the loaded data
 * param[in]       n_buffer        size of the allocated buffer
 * param[in]       filename        name of the file on the NVM
 * param[in]       offset          offset in bytes from where to start copying
 *                                  to buffer
 * return          number of bytes copied to buffer or zero on failure
 */
uint32_t binary_load(uint8_t *b_buffer, uint32_t n_buffer, char *filename,
                     uint32_t offset)
{
  int32_t copied_bytes = 0;
  int8_t rslt = 0;

  struct stat fileinfo;
  rslt = stat(filename, &fileinfo);
  if (rslt != 0) {
    fprintf(stderr,"stat'ing binary file %s: ",filename);
    perror("");
    return 0;
  }

  uint32_t filesize = fileinfo.st_size - offset;

  if (filesize > n_buffer) {
    fprintf(stderr,"%s: %d > %d\n", "binary data bigger than buffer", filesize,
            n_buffer);
    return 0;
  } else {
    FILE *file_ptr;
    file_ptr = fopen(filename,"rb");
    if (!file_ptr) {
      perror("fopen");
      return 0;
    }
    fseek(file_ptr,offset,SEEK_SET);
    copied_bytes = fread(b_buffer,sizeof(char),filesize,file_ptr);
    if (copied_bytes == 0) {
      fprintf(stderr,"%s empty\n",filename);
    }
    fclose(file_ptr);
    return copied_bytes;
  }
}

/*
 * Load previous library state from non-volatile memory
 *
 * param[in,out]   state_buffer    buffer to hold the loaded state string
 * param[in]       n_buffer        size of the allocated state buffer
 *
 * return          number of bytes copied to state_buffer or zero on failure
 */
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer)
{
  int32_t rslt = 0;
  rslt = binary_load(state_buffer, n_buffer, filename_state, 0);
  return rslt;
}

/*
 * Save library state to non-volatile memory
 *
 * param[in]       state_buffer    buffer holding the state to be stored
 * param[in]       length          length of the state string to be stored
 *
 * return          none
 */
void state_save(const uint8_t *state_buffer, uint32_t length)
{
  FILE *state_w_ptr;
  state_w_ptr = fopen(filename_state,"wb");
  fwrite(state_buffer,length,1,state_w_ptr);
  fclose(state_w_ptr);
}

/*
 * Load library config from non-volatile memory
 *
 * param[in,out]   config_buffer    buffer to hold the loaded state string
 * param[in]       n_buffer         size of the allocated state buffer
 *
 * return          number of bytes copied to config_buffer or zero on failure
 */
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
  int32_t rslt = 0;
  /*
   * Provided config file is 4 bytes larger than buffer.
   * Apparently skipping the first 4 bytes works fine.
   *
   */
  rslt = binary_load(config_buffer, n_buffer, filename_iaq_config, 4);
  return rslt;
}


/* main */

/*
 * Main function which configures BSEC library and then reads and processes
 * the data from both sensors at a specified rate and sends the data across
 * a MQTT channel.
 *
 * Program Arguments (all are optional, defaults will be used.)
 * -s             -- Use the BME680 secondary I2C address.
 * -d             -- Debug mode, writes output to stdout.
 * -u             -- Disable the pms5003 sensor. Only output date from the BSE680.
 * -b <addres>    -- the address of the MQTT broker/server (default: test.mosquitto.org).
 * -p <port>      -- the port number of the MQTT broker/server (default: 1883).
 * -t <topic>     -- the MQTT channel name (default: AirSenseData).
 * -i <sensor id> -- the id of the sensor.
 * -m <int>       -- Message output rate as a multiplier of the intrinsic BSEC sampling rate.
 *                   a value of 1 would send data at the intrinsic sampling rate while as
 *                   value of 2 would send data at twice the intrinsic sampling rate. (default 
 *                   is 2 which is every 6 seconds for the default BSEC configuration)
 * -o <float>     -- an offset temperature in °C. It will compensate for any heat generated by
 *                   components surrounding the BSE680 sensor (default: 5.0).
 * 
 * return      result of the processing. Note only returns on fatal error.
 */
int main(int argc, char **argv)
{
  //putenv(DESTZONE); // Switch to destination time zone
  int opt;
  const char* broker = DEF_ADDR ;
  const char* port = DEF_PORT;
  topic = DEF_TOPIC;
  i2c_address = BME680_I2C_ADDR_PRIMARY;
  size_t len;
  char mname[256];
  gethostname(mname, sizeof(mname));
  sensor_id = mname;
  sample_count = DEF_SAMPLE_MULTIPLIER;
  sample_multiplier = DEF_SAMPLE_MULTIPLIER;
  temp_offset = DEF_TEMP_OFFSET;
  return_values_init ret;

  /*
   * Parse the input parameters
   */
  while((opt = getopt(argc, argv, "sdub:p:t:i:m:o:")) != -1)
    switch(opt) 
    {
      case 's':
        i2c_address = BME680_I2C_ADDR_SECONDARY;
        break;
      case 'd':
        debug = 1;
        break;
      case 'u':
        disable5003 = 1;
        break;
      case 'b':
        broker = optarg;
        break;
      case 'p':
        port = optarg;
        break;
      case 't':
        topic = optarg;
        break;
      case 'i':
        sensor_id = optarg;
        break;
      case 'm':
        sample_multiplier = atoi(optarg);
        break;
      case 'o':
        temp_offset = atof(optarg);
        break;
      case '?':
        if (optopt == 'b' || optopt == 'p' || optopt == 't' || optopt == 'i' || optopt == 'm' || optopt == 'o')
          fprintf(stderr, "Option -%c requires an argument.\n", optopt);
        else if (isprint(optopt))
            fprintf(stderr, "Unknown option `-%c'.\n", optopt);
        else
            fprintf(stderr, "Unknown option character `\\x%x'.\n", optopt);
        
        return 1; //abort on option errors
    }
  if (debug) {
    const char * addr;
    if (i2c_address == BME680_I2C_ADDR_PRIMARY) {
      addr = "Primary";
    } else {
      addr = "Secondary";
    }
    printf("*** Application Starting ***\nI2C Address: %s\nDisable PMS5003: %d\nBroker: %s\nPort: %s",
           addr, disable5003, broker, port);
    printf("Topic: %s\nSensor ID: %s\nSample Multiplier: %d\n*******\n", topic, sensor_id,sample_multiplier);       
  }
  /* 
   * Open the MQTT communications
   */
  if (debug) {
    printf("Initializing MQTT communications\n");
  }
  /* open the non-blocking TCP socket (connecting to the broker) */
  sockfd = open_nb_socket(broker, port);

  if (sockfd == -1) {
      perror("Failed to open socket: ");
      exit_airsense(EXIT_FAILURE, sockfd, NULL);
  }

  /* setup a client */

  uint8_t sendbuf[2048]; /* sendbuf should be large enough to hold multiple whole mqtt messages */
  uint8_t recvbuf[1024]; /* recvbuf should be large enough any whole mqtt message expected to be received */
  mqtt_init(&client, sockfd, sendbuf, sizeof(sendbuf), recvbuf, sizeof(recvbuf), publish_callback);
  mqtt_connect(&client, sensor_id, NULL, NULL, 0, NULL, NULL, 0, 400);

  /* check that we don't have any errors */
  if (client.error != MQTT_OK) {
      fprintf(stderr, "MQTT error: %s\n", mqtt_error_str(client.error));
      exit_airsense(EXIT_FAILURE, sockfd, NULL);
  }

  /* start a thread to refresh the client (handle egress and ingree client traffic) */
  pthread_t client_daemon;
  if(pthread_create(&client_daemon, NULL, client_refresher, &client)) {
      fprintf(stderr, "Failed to start MQTT refresh client daemon.\n");
      exit_airsense(EXIT_FAILURE, sockfd, NULL);
  }

  if (debug) {
    printf("Initializing sensor communications\n");
  }
  /*
   * Open the i2c channel and initialize the BME680
   */
  i2cOpen();
  i2cSetAddress(i2c_address);
  /**
   * Open the UART using default serial0 @9600 baud.
   */
  if (!disable5003) {
    int ustat = pms_init();
    if (ustat != UART_OK) {
      output_uart_code(ustat);
      exit_airsense(EXIT_FAILURE, sockfd, NULL);
    }
  }
  
  if (debug) {
    printf("Initializing BSEC library\n");
  }
  // initalize the bsec library
  ret = bsec_iot_init(sample_rate_mode, temp_offset, bus_write, bus_read,
                      _sleep, state_load, config_load);
  if (ret.bme680_status) {
    /* Could not intialize BME680 */
    return (int)ret.bme680_status;
  } else if (ret.bsec_status) {
    /* Could not intialize BSEC library */
    return (int)ret.bsec_status;
  }

  if (debug) {
    printf("Entering sampling loop\n");
  }
  /* Call to endless loop function which reads and processes data based on
   * sensor settings.
   * State is saved every 10.000 samples, which means every 10.000 * 3 secs
   * = 500 minutes (depending on the config).
   *
   */
  bsec_iot_loop(_sleep, get_timestamp_us, output_ready, state_save, 10000);

  i2cClose();
  return 0;
}

