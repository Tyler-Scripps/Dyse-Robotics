include "MPU6050.h"

	////////// Initializers //////////

MPU6050::MPU6050(int ado)
{
	if (ado == 0)
		address = 0x68;
	else
		address = 0x69;

	test_WhoAmI();
}

// MPU6050::begin(mpu6050_gyro_range gRange, mpu6050_accel_range aRange)
// {
// 	Wire.begin()
// }

	////////// Diagnostic Functions //////////

uint8_t MPU6050::getStatus()
{
	return status;
}

	////////// Low-level I/O //////////

uint8_t MPU6050::readRegister(uint8_t reg)
{
	Wire.beginTransmission(address);
	Wire.write(reg);
	status = Wire.endTransmission(false);
	Wire.requestFrom(address, 1);
	while (!Wire.available());
	return Wire.read();
}

void MPU6050::writeRegister(uint8_t reg, uint8_t value)
{
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.write(value);
	status = Wire.endTransmission();
}

	////////// Tests //////////

bool MPU6050::test_WhoAmI()
{
	uint8_t val = readByte(address, WHO_AM_I_MPU6050);
	status = !((0x68 == val) && (0x69 == val))
}