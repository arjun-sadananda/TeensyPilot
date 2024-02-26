#include "MotionSense.h"
#include "SensorRegisters.h"
// #include <util/crc16.h>

#define NXP_MOTION_CAL_EEADDR  60
#define NXP_MOTION_CAL_SIZE    68

#define MAG_ADDR 0x0D
#define MPU_ADDR 0x68
#define BARO_ADDR 0x76

#define MPU_POWER_REG 0x6B
#define MPU_START 0x00
#define MPU_CONFIG_REG 0x1A  //Register 26 - Configuration
#define MPU_SET_DLPF 0x05  //DLPF settings for gyro and accel
                            //0x05 Accel and Gyro 1kHz: Bandwidth 10Hz Delay 13.8/13.4(and disables FSYNC?)
                            //0x00 Accel 260Hz bandwidth 0ms delay, No DLPF

static inline uint16_t _crc16_update(uint16_t crc, uint8_t data)
{
	unsigned int i;

	crc ^= data;
	for (i = 0; i < 8; ++i) {
		if (crc & 1) {
			crc = (crc >> 1) ^ 0xA001;
		} else {
			crc = (crc >> 1);
		}
	}
	return crc;
}

bool NXPMotionSense::begin()
{
	unsigned char buf[NXP_MOTION_CAL_SIZE];
	uint8_t i;
	uint16_t crc;

	Wire.begin();
	Wire.setClock(400000);

	memset(gyro_accel_raw, 0, sizeof(gyro_accel_raw));
	memset(mag_raw, 0, sizeof(mag_raw));

	//Serial.println("init hardware");
	while (!QMC5883_begin()) {
		Serial.println("config error FXOS8700");
		delay(1000);
	}
	while (!MPU6050_begin()) {
		Serial.println("config error FXAS21002");
		delay(1000);
	}
	// while (!BMP280_begin()) {
	// 	Serial.println("config error MPL3115");
	// 	delay(1000);
	// }
	//Serial.println("init done");

	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		buf[i] = EEPROM.read(NXP_MOTION_CAL_EEADDR + i);
	}
	crc = 0xFFFF;
	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		crc = _crc16_update(crc, buf[i]);
	}
	if (crc == 0 && buf[0] == 117 && buf[1] == 84) {
		memcpy(cal, buf+2, sizeof(cal));
	} else {
		memset(cal, 0, sizeof(cal));
		cal[9] = 50.0f;
	}
	// cal[0]=0;
	// cal[1]=0;
	// cal[2]=0;
	// cal[3]=0;
	// cal[4]=0;
	// cal[5]=0;
	// cal[6]=227.8;//93.62;
	// cal[7]=173.23;//153.45;
	// cal[8]=-2.03;//125.96;
	// cal[9]=58.6;//65.58;//field strength
	// cal[10]=1.106;//1.185;
	// cal[11]=.969;//0.772;
	// cal[12]=.937;//1.31;
	// cal[13]=0.048;//0.004;
	// cal[14]=.013;//-0.509;
	// cal[15]=-.032;//-0.007;

	return true;

}


void NXPMotionSense::update()
{
	static elapsedMillis msec;
	int32_t alt;

	if (MPU_read(gyro_accel_raw)) { // accel + mag
		//Serial.println("accel+mag");
	}
	// if (BMP280_read(&alt, &temperature_raw)) { // alt
	// 	//Serial.println("alt");
	// }
	if (MAG_read(mag_raw)) {  // gyro
		//Serial.println("gyro");
		newdata = 1;
	}
}


static bool write_reg(uint8_t i2c, uint8_t addr, uint8_t val)
{
	Wire.beginTransmission(i2c);
	Wire.write(addr);
	Wire.write(val);
	return Wire.endTransmission() == 0;
}

static bool read_regs(uint8_t i2c, uint8_t addr, uint8_t *data, uint8_t num)
{
	Wire.beginTransmission(i2c);
	Wire.write(addr);
	if (Wire.endTransmission(false) != 0) return false;
	Wire.requestFrom(i2c, num);
	if (Wire.available() != num) return false;
	while (num > 0) {
		*data++ = Wire.read();
		num--;
	}
	return true;
}

static bool read_regs(uint8_t i2c, uint8_t *data, uint8_t num)
{
	Wire.requestFrom(i2c, num);
	if (Wire.available() != num) return false;
	while (num > 0) {
		*data++ = Wire.read();
		num--;
	}
	return true;
}

bool NXPMotionSense::QMC5883_begin()
{
	// const uint8_t i2c_addr=FXOS8700_I2C_ADDR0;
	// uint8_t b;

	//Serial.println("QMC5883_begin");
	// detect if chip is present
	// if (!read_regs(i2c_addr, FXOS8700_WHO_AM_I, &b, 1)) return false;
	// //Serial.printf("FXOS8700 ID = %02X\n", b);
	// if (b != 0xC7) return false;
	// // place into standby mode
	// if (!write_reg(i2c_addr, FXOS8700_CTRL_REG1, 0)) return false;
	// // configure magnetometer
	// if (!write_reg(i2c_addr, FXOS8700_M_CTRL_REG1, 0x1F)) return false;
	// if (!write_reg(i2c_addr, FXOS8700_M_CTRL_REG2, 0x20)) return false;
	// // configure accelerometer
	// if (!write_reg(i2c_addr, FXOS8700_XYZ_DATA_CFG, 0x01)) return false; // 4G range
	// if (!write_reg(i2c_addr, FXOS8700_CTRL_REG2, 0x02)) return false; // hires
	// if (!write_reg(i2c_addr, FXOS8700_CTRL_REG1, 0x15)) return false; // 100Hz A+M
	//Serial.println("FXOS8700 Configured");
	Wire.beginTransmission(MAG_ADDR);
	Wire.write(0x0B);  // SET/RESET Period Register
	Wire.write(0x01);  // 
	Wire.endTransmission();

	Wire.beginTransmission(MAG_ADDR);
	Wire.write(0x09);  // Control Register
	Wire.write(0x1D);  //OSR RNG ODR Mode 
	Wire.endTransmission();
	return true;
}

bool NXPMotionSense::MPU6050_begin()
{
        const uint8_t i2c_addr=MPU_ADDR;
        // uint8_t b;

	// if (!read_regs(i2c_addr, FXAS21002_WHO_AM_I, &b, 1)) return false;
	// //Serial.printf("FXAS21002 ID = %02X\n", b);
	// if (b != 0xD7) return false;

	// place into standby mode
	// if (!write_reg(i2c_addr, FXAS21002_CTRL_REG1, 0)) return false;
	// // switch to active mode, 100 Hz output rate
	// if (!write_reg(i2c_addr, FXAS21002_CTRL_REG0, 0x00)) return false;
	// if (!write_reg(i2c_addr, FXAS21002_CTRL_REG1, 0x0E)) return false;
	    Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x6B);  // Register 107 - Power Management 1
        Wire.write(0x00);  // To start the device
        Wire.endTransmission();

        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x1A);  //Register 26 - Configuration
        Wire.write(0x05);  //DLPF settings for gyro and accel
                            //0x05 Accel and Gyro 1kHz: Bandwidth 10Hz Delay 13.8/13.4(and disables FSYNC?)
                            //0x00 Accel 260Hz bandwidth 0ms delay, No DLPF
        Wire.endTransmission();
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x1B);  //Register 27 - Gyro Config
        Wire.write(0x08);   //Full Scale Range +/- 500deg/sec -> Sensitivity of the Gyro to 65.5 LSB/deg/sec 2^16/500=131 32.768
        Wire.endTransmission();
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x1C);  //Register 28 - Accel Config
        Wire.write(0x10);  //Full Scale Range +/- 8g         -> Sensitivity of the Accel to 4096 LSB/g
        Wire.endTransmission();

	//Serial.println("FXAS21002 Configured");
	return true;
}

bool NXPMotionSense::MPU_read(int16_t *data)  // accel + mag
{
	// static elapsedMicros usec_since;
	// static int32_t usec_history=5000;
	// const uint8_t i2c_addr=MPU_ADDR;
	// uint8_t buf[13];

	// int32_t usec = usec_since;
	// if (usec + 100 < usec_history) return false;

	// //I2C_MST_STATUS 0x36
	// // if (!read_regs(i2c_addr, FXOS8700_STATUS, buf, 1)) return false;
	// if (!read_regs(i2c_addr, 0x36, buf, 1)) return false;
	// if (buf[0] == 0) return false;

	// usec_since -= usec;
	// int diff = (usec - usec_history) >> 3;
	// if (diff < -15) diff = -15;
	// else if (diff > 15) diff = 15;
	// usec_history += diff;

	// if (!read_regs(i2c_addr, FXOS8700_OUT_X_MSB, buf+1, 12)) return false;
	// //if (!read_regs(i2c_addr, buf, 13)) return false;

	// data[0] = (int16_t)((buf[1] << 8) | buf[2]);
	// data[1] = (int16_t)((buf[3] << 8) | buf[4]);
	// data[2] = (int16_t)((buf[5] << 8) | buf[6]);
	// data[3] = (int16_t)((buf[7] << 8) | buf[8]);
	// data[4] = (int16_t)((buf[9] << 8) | buf[10]);
	// data[5] = (int16_t)((buf[11] << 8) | buf[12]);
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x43);                                                //Register 67-72 - Gyroscope Measurements
	Wire.endTransmission();
	Wire.requestFrom(MPU_ADDR, 6);
	data[0] = Wire.read() << 8 | Wire.read();                  //67 GYRO_XOUT[15:8] and 68 GYRO_XOUT[7:0]
	data[1] = Wire.read() << 8 | Wire.read();                  //67 GYRO_XOUT[15:8] and 68 GYRO_XOUT[7:0]
	data[2] = Wire.read() << 8 | Wire.read();                  //67 GYRO_XOUT[15:8] and 68 GYRO_XOUT[7:0]
	Wire.beginTransmission(MPU_ADDR);

	Wire.write(0x3B);  //Register 59-64 - Accelerometer Measurements
	Wire.endTransmission();
	Wire.requestFrom(MPU_ADDR, 6);
	data[3] = Wire.read() << 8 | Wire.read();  //59 ACCEL_XOUT[15:8] and 60 ACCEL_XOUT[7:0]
	data[4] = Wire.read() << 8 | Wire.read();  //61 ACCEL_YOUT[15:8] and 62 ACCEL_YOUT[7:0]
	data[5] = Wire.read() << 8 | Wire.read();  //63 ACCEL_ZOUT[15:8] and 64 ACCEL_ZOUT[7:0]
	return true;
}

bool NXPMotionSense::MAG_read(int16_t *data) // gyro
{
	static elapsedMicros usec_since;
	static int32_t usec_history=10000;
	const uint8_t i2c_addr=FXAS21002_I2C_ADDR0;
	uint8_t buf[7];

	int32_t usec = usec_since;
	if (usec + 100 < usec_history) return false;

	// if (!read_regs(i2c_addr, FXAS21002_STATUS, buf, 1)) return false;
	// if (buf[0] == 0) return false;

	usec_since -= usec;
	int diff = (usec - usec_history) >> 3;
	if (diff < -15) diff = -15;
	else if (diff > 15) diff = 15;
	usec_history += diff;
	//Serial.println(usec);

	// if (!read_regs(i2c_addr, FXAS21002_STATUS, buf, 7)) return false;
	// //if (!read_regs(i2c_addr, buf, 7)) return false;

	// data[0] = (int16_t)((buf[1] << 8) | buf[2]);
	// data[1] = (int16_t)((buf[3] << 8) | buf[4]);
	// data[2] = (int16_t)((buf[5] << 8) | buf[6]);
	Wire.beginTransmission(MAG_ADDR);
	Wire.write(0x00);
	int err = Wire.endTransmission();
	if(!err){
		Wire.requestFrom(MAG_ADDR, 6);
		data[0] = (int16_t)(Wire.read()|Wire.read()<<8);
		data[1] = (int16_t)(Wire.read()|Wire.read()<<8);
		data[2] = (int16_t)(Wire.read()|Wire.read()<<8);
		return true;
	}
}

// bool NXPMotionSense::BMP280_begin() // pressure
// {
//         const uint8_t i2c_addr=MPL3115_I2C_ADDR;
//         uint8_t b;
// 
// 	if (!read_regs(i2c_addr, MPL3115_WHO_AM_I, &b, 1)) return false;
// 	//Serial.printf("MPL3115 ID = %02X\n", b);
// 	if (b != 0xC4) return false;
// 
// 	// place into standby mode
// 	if (!write_reg(i2c_addr, MPL3115_CTRL_REG1, 0)) return false;
// 
// 	// switch to active, altimeter mode, 512 ms measurement, polling mode
// 	if (!write_reg(i2c_addr, MPL3115_CTRL_REG1, 0xB9)) return false;
// 	// enable events
// 	if (!write_reg(i2c_addr, MPL3115_PT_DATA_CFG, 0x07)) return false;
// 
// 	//Serial.println("MPL3115 Configured");
// 	return true;
// }

// bool NXPMotionSense::BMP280_read(int32_t *altitude, int16_t *temperature)
// {
// 	static elapsedMicros usec_since;
// 	static int32_t usec_history=980000;
// 	const uint8_t i2c_addr=MPL3115_I2C_ADDR;
// 	uint8_t buf[6];
// 
// 	int32_t usec = usec_since;
// 	if (usec + 500 < usec_history) return false;
// 
// 	if (!read_regs(i2c_addr, FXAS21002_STATUS, buf, 1)) return false;
// 	if (buf[0] == 0) return false;
// 
// 	if (!read_regs(i2c_addr, buf, 6)) return false;
// 
// 	usec_since -= usec;
// 	int diff = (usec - usec_history) >> 3;
// 	if (diff < -1000) diff = -1000;
// 	else if (diff > 1000) diff = 1000;
// 	usec_history += diff;
// 
// 	int32_t a = ((uint32_t)buf[1] << 12) | ((uint16_t)buf[2] << 4) | (buf[3] >> 4);
// 	if (a & 0x00080000) a |= 0xFFF00000;
// 	*altitude = a;
// 	*temperature = (int16_t)((buf[4] << 8) | buf[5]);
// 
// 	//Serial.printf("%02X %d %d: ", buf[0], usec, usec_history);
// 	//Serial.printf("%6d,%6d", a, *temperature);
// 	//Serial.println();
// 	return true;
// }

bool NXPMotionSense::writeCalibration(const void *data)
{
	const uint8_t *p = (const uint8_t *)data;
	uint16_t crc;
	uint8_t i;

	if (p[0] != 117 || p[1] != 84) return false;
	crc = 0xFFFF;
	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		crc = _crc16_update(crc, p[i]);
	}
	if (crc != 0) return false;
	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		EEPROM.write(NXP_MOTION_CAL_EEADDR + i, p[i]);
	}
	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		if (EEPROM.read(NXP_MOTION_CAL_EEADDR + i) != p[i]) return false;
	}
	memcpy(cal, ((const uint8_t *)data)+2, sizeof(cal));
	return true;
}

