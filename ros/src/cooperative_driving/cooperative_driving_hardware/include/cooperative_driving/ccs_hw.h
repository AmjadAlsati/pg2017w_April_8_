//////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2017, CCS Labs
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the <organization> nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////////
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>

/** Struct containing the X and Y coordinates for tracking the 
* location of the robot.
* 
*/
struct Position
{
  int16_t x;
  int16_t y;
  int16_t alpha;
};


#define ADC_WIDTH 7

/** Struct containing the values of the distance sensors that are received 
* from the microcontroller.
*/
struct Sensors
{
  uint8_t bumpers;
  uint8_t analog[ADC_WIDTH];
};

#define UNUSED(x) (void)x

#define CCS_I2C_BUS_ADDRESS 0x69
#define CCS_LED_REGISTER 0x20
#define CCS_LED_REGISTER_L 2
#define CCS_VELOCITY_REGISTER 0x1A
#define CCS_VELOCITY_REGISTER_L 2
#define CCS_POSITION_REGISTER 0x1B
#define CCS_POSITION_REGISTER_L sizeof(struct Position)
#define CCS_SENSOR_REGISTER 0x10
#define CCS_SENSOR_REGISTER_L sizeof(struct Sensors)

/** Writes the LED values to the microcontroller.
* 
* \param[in] i2cdev     i2c device id obtained by the code
* \param[in] red        Numeric value in 0-255 to be written to red LED
* \param[in] blue       Numeric value in 0-255 to be written to blue LED
* 
* \return Return status of writing for arm architecture. Else return zero.
*/
static inline int ccs_write_led(int i2cdev, uint8_t red, uint8_t blue)
{
#ifdef __arm__
  uint8_t values[] = { red, blue };
  return i2c_smbus_write_i2c_block_data(i2cdev, CCS_LED_REGISTER, CCS_LED_REGISTER_L, values);
#else
  UNUSED(i2cdev);
  UNUSED(red);
  UNUSED(blue);
  return 0;
#endif
}

/** Reads the current LED states from the microcontroller.
*
* \param[in] i2cdev     i2c device id obtained by the code
* \param[out] red       Current value of the red LED as returned by microcontroller
* \param[out] blue      Current value of the blue LED as returned by microcontroller
*
* \return Return status of reading for arm architecture. Else return zero.
* \todo Sometimes Not reading any values. Maybe the MC-firmware buggy?
*/
static inline int ccs_read_led(int i2cdev, uint8_t *red, uint8_t *blue)
{  
#ifdef __arm__
  uint8_t values[CCS_LED_REGISTER_L];
  int ret = i2c_smbus_read_i2c_block_data(i2cdev, CCS_LED_REGISTER, CCS_LED_REGISTER_L, values);
  *red = values[0];
  *blue = values[1];
  return ret;
#else
  UNUSED(i2cdev);
  *red = 0;
  *blue = 0;
  return 0;
#endif
}

/** Writes the Engine velocities to the microcontroller.
*
* \param[in] i2cdev     i2c device id obtained by the code
* \param[in] left       Velocity in 0-255 for the left engine
* \param[in] right      Velocity in 0-255 for the right engine
*
* \return Return status of writing for arm architecture. Else return zero.
*/
static inline int ccs_write_velocities(int i2cdev, int8_t left, int8_t right)
{
#ifdef __arm__
  uint8_t values[] = { (uint8_t)left, (uint8_t)right };
  return i2c_smbus_write_i2c_block_data(i2cdev, CCS_VELOCITY_REGISTER, CCS_VELOCITY_REGISTER_L, values);
#else
  UNUSED(i2cdev);
  UNUSED(left);
  UNUSED(right);
  return 0;
#endif
}

/** Reads the current engine velocity as calculated by the
* microcontroller.
*
* \param[in] i2cdev      i2c device id obtained by the code
* \param[out] left       Velocity in 0-255 for the left engine as calculated by the microcontroller
* \param[out] right      Velocity in 0-255 for the right engine as calculated by the microcontroller
*
* \return Return status of reading for arm architecture. Else return zero.
*/
static inline int ccs_read_velocities(int i2cdev, int8_t *left, int8_t *right)
{
#ifdef __arm__
  uint8_t values[CCS_VELOCITY_REGISTER_L];
  int ret = i2c_smbus_read_i2c_block_data(i2cdev, CCS_VELOCITY_REGISTER, CCS_VELOCITY_REGISTER_L, values);
  *left = (int8_t)values[0];
  *right = (int8_t)values[1];
  return ret;
#else
  UNUSED(i2cdev);
  *left = 0;
  *right = 0;
  return 0;
#endif
}

/** Reads the current position of the robot as calculated by
* the microcontroller odometry, represented as X,Y coordinates.
*
* \param[in] i2cdev      i2c device id obtained by the code
* \param[out] position   Current position of the robot as calculated by the odometry in the microcontroller 
*
* \return Return status of reading for arm architecture. Else return zero.
*/

static inline int ccs_read_position(int i2cdev, struct Position *position)
{
#ifdef __arm__
  return i2c_smbus_read_i2c_block_data(i2cdev, CCS_POSITION_REGISTER, CCS_POSITION_REGISTER_L, (uint8_t *)position);
#else
  UNUSED(i2cdev);
  position->x = 0;
  position->y = 0;
  position->alpha = 0;
  return 0;
#endif
}

/** Reads the sensor values from the microcontroller
* 
* \param[in] i2cdev      i2c device id obtained by the code
* \param[out] sensors    Readings of the sensor values in a struct
*
* \return Return status of reading for arm architecture. Else return zero.
*/
static inline int ccs_read_sensors(int i2cdev, struct Sensors *sensors)
{
#ifdef __arm__
  return i2c_smbus_read_i2c_block_data(i2cdev, CCS_SENSOR_REGISTER, CCS_SENSOR_REGISTER_L, (uint8_t *)sensors);
#else
  UNUSED(i2cdev);
  sensors->analog[0] = 0;
  sensors->analog[1] = 1;
  sensors->analog[2] = 2;
  sensors->analog[3] = 3;
  sensors->analog[4] = 4;
  sensors->analog[5] = 5;
  sensors->analog[6] = 6;
  return 0;
#endif
}

/** Gets access to the i2c device by obtaining a file descriptor id
*
* \param[out] i2cdevice  A file descriptor of the i2c device 
*
* \return Return the file descriptor if successful or failure depending on the outcome of system call for arm architecture. Else return zero.
*/
static inline int ccs_open(const char *i2c_device)
{
#ifdef __arm__
  int fd = open(i2c_device, O_RDWR);
  if (fd < 0)
  {
    return fd;
  }
  if (ioctl(fd, I2C_SLAVE, CCS_I2C_BUS_ADDRESS) < 0)
  {
    return -1;
  }
  return fd;
#else
  UNUSED(i2c_device);
  return 0;
#endif
}

/** Proper closure of the i2c device by making a close() call
*
* \param[in] fd  A file descriptor of the i2c device to be closed 
*
* \return Return status of close() system call for arm architecture. Else return zero.
*/
static inline int ccs_close(int fd)
{
#ifdef __arm__
  ccs_write_velocities(fd, 0, 0);
  return close(fd);
#else
  UNUSED(fd);
  return 0;
#endif
}
