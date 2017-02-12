#include "Repetier.h"

#if Z_PROBE_ACCELEROMETER

// TODO: Move this to Configuration.h
#define ZPROBE_SENSITIVITY       30
#define ZPROBE_TIMELIMIT         10
#define ZPROBE_TIMELAT           20
#define ZPROBE_TIMEWIN           255

bool accelerometer_initialized = false;

#if Z_PROBE_LIS3DH || Z_PROBE_HE280

#if Z_PROBE_LIS3DH
#define ACCEL_I2C_DEVADDR           (0x18 << 1)
#define ACCEL_I2C_DEVICEID          0x33
#elif Z_PROBE_HE280
#define ACCEL_I2C_DEVADDR           (0x19 << 1)
#define ACCEL_I2C_DEVICEID          0x6A
#endif // Z_PROBE_LIS3DH || Z_PROBE_HE280

// LIS3DH HW regs
#define LIS3DH_WHOAMI            0x0F
#define LIS3DH_REG_CTRL1         0x20
#define LIS3DH_REG_CTRL3         0x22
#define LIS3DH_REG_CTRL4         0x23
#define LIS3DH_REG_CTRL5         0x24
#define LIS3DH_REG_CLICKCFG      0x38
#define LIS3DH_REG_CLICKSRC      0x39
#define LIS3DH_REG_CLICKTHS      0x3A
#define LIS3DH_REG_TIMELIMIT     0x3B
#define LIS3DH_REG_TIMELATENCY   0x3C
#define LIS3DH_REG_TIMEWINDOW    0x3D

static void accelerometer_write8(uint8_t reg, uint8_t val)
{
  HAL::i2cStartWait(ACCEL_I2C_DEVADDR | I2C_WRITE);
  HAL::i2cWrite(reg);
  HAL::i2cWrite(val);
  HAL::i2cStop();
}

static uint8_t accelerometer_read8(uint8_t reg)
{
  HAL::i2cStartWait(ACCEL_I2C_DEVADDR | I2C_WRITE);
  HAL::i2cWrite(reg);
  HAL::i2cStop();
  HAL::i2cStartWait(ACCEL_I2C_DEVADDR | I2C_READ);
  ret = HAL::i2cReadNak();
  HAL::i2cStop();
  return ret;
}

bool accelerometer_init()
{
  if (accelerometer_read8(ACCEL_I2C_WHOAMI) != ACCEL_I2C_DEVICEID) {
    Com::printErrorFLN(PSTR("Failed to find accelerometer probe!"));
    return false;
  }
  // Enable only the Z-Axis, set 400Hz data rate, no lpwr mode
  accelerometer_write8(LIS3DH_REG_CTRL1, 0b01110100);
  // BDU, 8G range, high res
  accelerometer_write8(LIS3DH_REG_CTRL4, 0b10101000);

  // interrupt on Z-Axis passing threshold
  accelerometer_write8(LIS3DH_REG_CLICKCFG, 0b00110000);

  accelerometer_write8(LIS3DH_REG_CLICKTHS,    ZPROBE_SENSITIVITY); // arbitrary
  accelerometer_write8(LIS3DH_REG_TIMELIMIT,   ZPROBE_TIMELIMIT);   // arbitrary
  accelerometer_write8(LIS3DH_REG_TIMELATENCY, ZPROBE_TIMELAT);     // arbitrary
  accelerometer_write8(LIS3DH_REG_TIMEWINDOW,  ZPROBE_TIMEWIN);     // arbitrary

  accelerometer_initialized = true;
  return true;
}

bool accelerometer_clicked()
{
  return accelerometer_initialized &&
    ((accelerometer_read8(LIS3DH_REG_CLICKSRC) & 0x30) != 0);
}

#else
#error Unsupported accelerometer chosen!
#endif

#endif // Z_PROBE_ACCELEROMETER
