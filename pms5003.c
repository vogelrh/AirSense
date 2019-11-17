/**
 * Copyright (C) 2019 Robert Vogel
 * 
 * C library for retrieving data from the Plantower PMS5003 particulate sensor.
 * Inspired by the Pimoroni pms5003-python library.
 */
#define _XOPEN_SOURCE 700

#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include "pms5003.h"

/**************************************************
 * Private structs/unions and constants
 **************************************************/
#define PMS5003_EXPECTED_BYTES 28
#define LITTLE_ENDED 1
static const uint16_t PMS5003_SOF = 19778; //the x42, x4d start characters

/**
 * Union for converting two byte reads from the UART to 
 * unsigned short int.
 * NOTE: This will ONLY work on Big Endian systems!!!
 * If using on Little Endian systems then the bytes must
 * be swapped before readin value;
 */
typedef union uart_word {
  uint16_t value;
  uint8_t data[2];
} uart_word;

/**
 * Union for converting the 26 bytes of data fread 
 * from the pms5003 to unsigned int values.
 * NOTE: Same Big Endian issues apply!
 */
typedef union pms5003_data_block {
  struct
  {
    uint16_t pm1cf;
    uint16_t pm2_5cf;
    uint16_t pm10cf;
    uint16_t pm1at;
    uint16_t pm2_5at;
    uint16_t pm10at;
    uint16_t gt0_3;
    uint16_t gt0_5;
    uint16_t gt1;
    uint16_t gt2_5;
    uint16_t gt5;
    uint16_t gt10;
    uint16_t reserved;
    uint16_t cksum;
  } d;
  uint8_t raw_data[PMS5003_EXPECTED_BYTES];
} pms5003_data_block;

/**************************************************
 * Module level variables.
 **************************************************/
int uart0_filestream = -1;
int uart_status = UART_NOT_INITIALIZED;

/**************************************************
 * Private functions
 **************************************************/
/**
 * Sleep function that uses nanosleep
 */
static void msleep(uint32_t ms)
{
  struct timespec ts;
  ts.tv_sec = 0;
  ts.tv_nsec = (ms % 1000) * 1000000L; //force range between 0 and 999999999 
  nanosleep(&ts, NULL);
}
/**
 * Swaps bytes in two byte words in byte array
 */
static void swap_bytes16(uint8_t *array, int size)
{
  uint8_t temp;
  int i;

  for (i = 0; i < size; i = i + 2)
  {
    temp = array[i];
    array[i] = array[i + 1];
    array[i + 1] = temp;
  }
}
static int byte_read(int ufd, uint8_t *buf, size_t count) {
  size_t num_read = 0;
  do {
    int rdlen = read(ufd, (char *) buf + num_read, count - num_read);
    if (rdlen < 1) {   
      return UART_RX_ERROR;
    }
    num_read += rdlen;
  } while (num_read < count);

  return UART_OK;
}
/**
 * Reads two bytes from the UART and populates the specified uart_word
 * union. Note because we are running in no-wait mode, we may not have 
 * any data yet. In that case we make sure the word structure has a
 * zero value.
 * 
 * param[out]       word        A uart_word union recieving the data.
 * 
 * Returns a UART status/error code.
 */
static int read_word(uart_word *word, int swap)
{
  int rx_cnt = byte_read(uart0_filestream, word->data, 2);
  if (rx_cnt != UART_OK) {
    word->value = 0;
    return  rx_cnt;
  }
  // {
  //   return UART_RX_ERROR; // some read error occured.
  // }
  // else if (rx_cnt == 0)
  // { // no data yet.
  //   word->value = 0;
  // }
  // else if (rx_cnt != 2)
  // {
  //   return UART_UNEXPECTED_DATA_ERROR;
  // }

  if (swap)
  {
    swap_bytes16(word->data, 2);
  }
  return UART_OK;
}

/**
 * Read a block of bytes from the PMS5003 and populates the specified
 * pms5003_data_block union. If an error occurs, then an error code is
 * returned. If successful, UART_OK is returned.
 * 
 * param[out]       data        A pms5003_data_block union recieving the data.
 * 
 * Returns a UART status/error code.
 */
static int read_pms_data_block(pms5003_data_block *data)
{
  int i;
  int checksum;
  int rstat;

  // get time at start an loop through reading 2 bytes from uart until
  // we find a start sequence or we exceed the timeout interval.
  clock_t begin;
  double elapse_time;

  // Zero the data sturcture
  for (i = 0; i < PMS5003_EXPECTED_BYTES; i++)
  {
    data->raw_data[i] = 0;
  }
  // Exit if UART is not open
  if (uart_status != UART_OK)
  {
    return UART_NOT_INITIALIZED;
  }

  begin = clock();
  uart_word sof;

  while (1)
  { //wait for SOF word
    elapse_time = (double)(clock() - begin) / CLOCKS_PER_SEC;
    if (elapse_time >= 5.0)
    {
      return UART_TIMEOUT_ERROR;
    }
    rstat = read_word(&sof, 0);
    if (rstat == UART_OK && sof.value == PMS5003_SOF)
    {
      checksum = sof.data[0] + sof.data[1];
      break;
    }
    else if (rstat != UART_OK)
    {    
      return rstat; //error occured
    }
  }

  // Now read frame length word
  uart_word packet_length;
  rstat = read_word(&packet_length, LITTLE_ENDED);
  if (rstat != UART_OK)
  {
    return rstat;
  }

  if (packet_length.value != PMS5003_EXPECTED_BYTES)
  {
    return UART_UNEXPECTED_DATA_ERROR;
  }

  checksum += (packet_length.data[0] + packet_length.data[1]);

  // Now read sensor data, but first wait for rest of data
  //msleep(50); // 50 milliseconds is plenty for 28 bytes at 9600 baud

  int rx_cnt = byte_read(uart0_filestream, data->raw_data, PMS5003_EXPECTED_BYTES);
  if (rx_cnt != UART_OK)
  {
    return rx_cnt; // some read error occured.
  }

  // Swap bytes if little ended system
  if (LITTLE_ENDED)
  {
    swap_bytes16(data->raw_data, PMS5003_EXPECTED_BYTES);
  }

  // Finish calculating the check sum
  for (i = 0; i < PMS5003_EXPECTED_BYTES - 2; i++)
  {
    checksum += data->raw_data[i];
  }
  if (checksum != data->d.cksum)
  {
    return UART_CHECKSUM_ERROR;
  }

  return UART_OK;
}

/**
 * Converts an integer value of the baud rate to the
 * correct constand value needed by the uart configuration.
 */
static int32_t convert_baudrate(int baudrate)
{
  switch (baudrate)
  {
  case 1200:
    return B1200;
  case 2400:
    return B2400;
  case 4800:
    return B4800;
  case 9600:
    return B9600;
  case 19200:
    return B19200;
  case 38400:
    return B38400;
  case 57600:
    return B57600;
  case 115200:
    return B115200;
  case 230400:
    return B230400;
  }

  return -1;
}

/**************************************************
 * ***** Public Facing functions *****
 **************************************************/
int pms_init_override(char *device, int baud)
{
  uart_status = UART_NOT_INITIALIZED;
  if (device == NULL)
  {
    return UART_PARAMETER_ERROR;
  }

  int32_t baud_rate = convert_baudrate(baud);
  if (baud_rate < 0)
  {
    return UART_PARAMETER_ERROR;
  }

  uart0_filestream = open(device, O_RDWR | O_NOCTTY | O_NDELAY); //open in non blocking mode
  if (uart0_filestream < 0)
  {
    return UART_INIT_ERROR;
  }

  // Initialze for 8bit data, 1 stop bit, no parity no checkbit
  struct termios options;
  tcgetattr(uart0_filestream, &options);
  cfsetospeed(&options, baud_rate);
  cfsetispeed(&options, baud_rate);
  //no parity (8N1)
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  //Enable receiver and do not change owner of port
  options.c_cflag |= CLOCAL | CREAD;

  //raw input
  options.c_lflag &= ~(ICANON | ECHO | ECHONL | ISIG | IEXTEN);

  // One byte is enough to return from read
  options.c_cc[VMIN] = 1;
  options.c_cc[VTIME] = 0;

  // Turn off output processing
  options.c_oflag = 0;

  tcflush(uart0_filestream, TCIFLUSH);
  if (tcsetattr(uart0_filestream, TCSANOW, &options) != 0) {
    return UART_INIT_ERROR;
  }

  msleep(500); // slight delay to avoid No such file or directory.
  uart_status = UART_OK;
  return uart_status;
}

int pms_init()
{
  return pms_init_override("/dev/serial0", 9600);
}

void pms_close()
{
  close(uart0_filestream);
  uart_status = UART_NOT_INITIALIZED;
}

void output_uart_code(int error_code)
{
  char *err_msg = NULL;
  int sysErr = 0;
  switch (error_code)
  {
  case UART_OK:
    err_msg = "Status: OK";
    break;
  case UART_NO_DATA:
    err_msg = "Status: No data returned from UART";
    break;
  case UART_NOT_INITIALIZED:
    err_msg = "Status: UART not initialized";
    break;
  case UART_INIT_ERROR:
    sysErr = 1;
    err_msg = "ERROR: UART initialization error";
    break;
  case UART_PARAMETER_ERROR:
    sysErr = 1;
    err_msg = "ERROR: Incorrect UART parmeters provided";
    break;
  case UART_TX_ERROR:
    sysErr = 1;
    err_msg = "ERROR: UART transmit problem";
    break;
  case UART_RX_ERROR:
    sysErr = 1;
    err_msg = "ERROR: UART recieve problem";
    break;
  case UART_UNEXPECTED_DATA_ERROR:
    err_msg = "ERROR: PMS5003 unexpected data. Block size does not match data structure";
    break;
  case UART_CHECKSUM_ERROR:
    err_msg = "ERROR: PMS5003 data checksum error";
    break;
  case UART_TIMEOUT_ERROR:
    err_msg = "ERROR: PMS5003 read timeout error";
    break;
  }
  if (err_msg != NULL)
  {
    if (sysErr)
    {   
      perror(err_msg);
    }
    else
    {    
      fprintf(stderr, "%s\n", err_msg);
    }
  }
}

int read_pms5003_data(PMS5003_DATA *data)
{
  pms5003_data_block rd;
  int status = read_pms_data_block(&rd); //read the sensor

  if (status == UART_OK)
  { //transfer data to final structure
    data->pm1cf = rd.d.pm1cf;
    data->pm2_5cf = rd.d.pm2_5cf;
    data->pm10cf = rd.d.pm10cf;
    data->pm1at = rd.d.pm1at;
    data->pm2_5at = rd.d.pm2_5at;
    data->pm10at = rd.d.pm10at;
    data->gt0_3 = rd.d.gt0_3;
    data->gt0_5 = rd.d.gt0_5;
    data->gt1 = rd.d.gt1;
    data->gt2_5 = rd.d.gt2_5;
    data->gt5 = rd.d.gt5;
    data->gt10 = rd.d.gt10;
  }
  return status;
}
