#include <stdio.h>
#include "semihosting.h"
#include "stm32f10x.h"
#include "delay.h"
#include "i2c.h"
#include "mpu9250.h"
#include "math.h"
#include "kalman.h"
#include "filters.h"

#define MPU9250_ADDRESS 0x68

enum Ascale {
	AFS_2G = 0,
	AFS_4G,
	AFS_8G,
	AFS_16G
};

enum Gscale {
	GFS_250DPS = 0,
	GFS_500DPS,
	GFS_1000DPS,
	GFS_2000DPS
};

enum Mscale {
	MFS_14BITS = 0, // 0.6 mG per LSB
	MFS_16BITS      // 0.15 mG per LSB
};

// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;

// Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mscale = MFS_16BITS;

// 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
uint8_t Mmode = 0x02;

// Scale resolutions per LSB for the sensors
float aRes, gRes, mRes;

// Low-Pass
int32_t oldAccel[3] = {0, 0, 0};

// Bias corrections
float gyroBias[3] 	= {0, 0, 0};
float accelBias[3] 	= {0, 0, 0};
float magBias[3] 	= {0, 0, 0};
float magScale[3]  	= {0, 0, 0};

void getMres()
{
  switch (Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
      mRes = 10.0f * 4912.0f / 8190.0f; // Proper scale to return milliGauss
      break;
    case MFS_16BITS:
      mRes = 10.0f * 4912.0f / 32760.0f; // Proper scale to return milliGauss
      break;
  }
}

void getGres()
{
	switch (Gscale)
	{
		// Possible gyro scales (and their register bit settings) are:
		// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS (11).
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
		// 2-bit value:
		case GFS_250DPS: // 250.0f / 32768.0f
			gRes = 250.0f / 32768.0f;
		break;
		case GFS_500DPS: // 500.0f / 32768.0f
			gRes = 655;
		break;
		case GFS_1000DPS: // 1000.0f / 32768.0f
			gRes = 328;
		break;
		case GFS_2000DPS: // 2000.0f / 32768.0f
			gRes = 164;
		break;
	}
}

// Possible accelerometer scales (and their register bit settings) are:
// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
void getAres() {
  switch (Ascale)
  {
    case AFS_2G: // 2.0 / 32768.0
          aRes = 2.0 / 32768.0;
          break;
    case AFS_4G:
          aRes = 4.0 / 32768.0;
          break;
    case AFS_8G:
          aRes = 8.0 / 32768.0;
          break;
    case AFS_16G:
          aRes = 16.0 / 32768.0;
          break;
  }
}

int32_t lowPass(int32_t input, int32_t output, int32_t alpha, int32_t mult) {
	int32_t ret = input;

	ret = (output * mult + alpha * (input - output)) / mult;

	return ret;
}

void mpu9250_init() {
	uint8_t data;

	// Clear sleep bit (enable all sensors) to wake up the sensor
	i2c_write_register(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x00);
	printf("Clearing sleep bit...\n\r");
	DelayUs(200);

	// Manufacture gave me the wrong chip. MPU9255 instead of MPU9250
	// Ohh well, it's an upgrade
	uint8_t whoami;
	i2c_read_register(MPU9250_ADDRESS, MPU9250_WHO_AM_I, &whoami);
	printf("I am 0x%x\n\r", whoami);
	DelayUs(200);

	// Get stable time source
	i2c_write_register(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x01);

	// Configure Gyro and Accelerometer
	i2c_write_register(MPU9250_ADDRESS, MPU9250_CONFIG, 0x03);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	i2c_write_register(MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, 0x04);

	// Set gyroscope full scale range
	i2c_read_register(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, &data);
	data = data & ~0x02; // Clear Fchoice bits [1:0]
	data = data & ~0x18; // Clear AFS bits [4:3]
	data = data | Gscale << 3; // Set full scale range for the gyro
	i2c_write_register(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, data);

	// Set accelerometer full-scale range configuration
	i2c_read_register(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, &data);
	data = data & ~0x18;  // Clear AFS bits [4:3]
	data = data | Ascale << 3; // Set full scale range for the accelerometer
	i2c_write_register(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, data);

	// Set accelerometer sample rate configuration
	i2c_read_register(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2, &data);
	data = data & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	data = data | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	i2c_write_register(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2, data);
	i2c_write_register(MPU9250_ADDRESS, MPU9250_INT_PIN_CFG, 0x22);
	i2c_write_register(MPU9250_ADDRESS, MPU9250_INT_ENABLE, 0x01);

	getAres(); getGres(); getMres();

	DelayUs(100);
}

void mpu9250_readGyroData(int16_t* dest)
{
  uint8_t rawData[6];
  int16_t values[3];

  i2c_readmulti_register(MPU9250_ADDRESS, MPU9250_GYRO_XOUT_H, &rawData[0], 6);
  values[0] = ((int16_t)rawData[0] << 8) | rawData[1];
  values[1] = ((int16_t)rawData[2] << 8) | rawData[3];
  values[2] = ((int16_t)rawData[4] << 8) | rawData[5];

  values[0] = values[0] * 1000 * gRes;
  values[1] = values[1] * 1000 * gRes;
  values[2] = values[2] * 1000 * gRes;

  dest = values;

  //printf("%d\r", dest[2]);

  //printf("Gyro Data (g) X: %d, Y: %d, Z: %d\r", dest[0], dest[1], dest[2]);
}


void mpu9250_readAccelData(int16_t* dest) {
	uint8_t rawData[6];
	int16_t values[3];

	i2c_readmulti_register(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, &rawData[0], 6);

	/*i2c_read_register(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, &rawData[0]);
	i2c_read_register(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_L, &rawData[1]);
	i2c_read_register(MPU9250_ADDRESS, MPU9250_ACCEL_YOUT_H, &rawData[2]);
	i2c_read_register(MPU9250_ADDRESS, MPU9250_ACCEL_YOUT_L, &rawData[3]);
	i2c_read_register(MPU9250_ADDRESS, MPU9250_ACCEL_ZOUT_H, &rawData[4]);
	i2c_read_register(MPU9250_ADDRESS, MPU9250_ACCEL_ZOUT_L, &rawData[5]);*/

	values[0] = ((int16_t)rawData[0] << 8) | rawData[1];
	values[1] = ((int16_t)rawData[2] << 8) | rawData[3];
	values[2] = ((int16_t)rawData[4] << 8) | rawData[5];

	values[0] = values[0] * 1000 * aRes;
	values[1] = values[1] * 1000 * aRes;
	values[2] = values[2] * 1000 * aRes;

	dest = values;

	int32_t x = lowPass(values[0], oldAccel[0], 15, 100);
	oldAccel[0] = x;

	int32_t y = lowPass(values[1], oldAccel[1], 15, 100);
	oldAccel[1] = y;

	int32_t z = lowPass(values[2], oldAccel[2], 15, 100);
	oldAccel[2] = z;

	printf("%d %d %d\r", x, y, z);

	//printf("Accel Data (g) X: %d, Y: %d, Z: %d\r", dest[0], dest[1], dest[2]);
}

void mpu9250_read_accel_bias(int32_t* accel_bias) {
	unsigned char data[6] = {0, 0, 0, 0, 0, 0};
	i2c_readmulti_register(MPU9250_ADDRESS, MPU9250_XA_OFFSET_H, &data[0], 2); // Read factory accelerometer trim values
	i2c_readmulti_register(MPU9250_ADDRESS, MPU9250_YA_OFFSET_H, &data[2], 2);
	i2c_readmulti_register(MPU9250_ADDRESS, MPU9250_ZA_OFFSET_H, &data[4], 2);

	accel_bias[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	accel_bias[1] = (int32_t) (((int16_t)data[2] << 8) | data[3]);
	accel_bias[2] = (int32_t) (((int16_t)data[4] << 8) | data[5]);
}

void mpu9250_set_accel_bias(int32_t* accel_bias) {
	uint8_t data[6] = {0, 0, 0, 0, 0, 0};
	int32_t accel_bias_reg[3] = {0, 0, 0};

	mpu9250_read_accel_bias(accel_bias_reg);

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	// Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[0] -= (accel_bias[0] & ~1); // ~1 preserves the temperature bit
	accel_bias_reg[1] -= (accel_bias[1] & ~1); // ~1 preserves the temperature bit
	accel_bias_reg[2] -= (accel_bias[2] & ~1); // ~1 preserves the temperature bit

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1])      & 0xFF;
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2])      & 0xFF;

	i2c_write_register(MPU9250_ADDRESS, MPU9250_XA_OFFSET_H, data[0]);
	i2c_write_register(MPU9250_ADDRESS, MPU9250_XA_OFFSET_L, data[1]);
	i2c_write_register(MPU9250_ADDRESS, MPU9250_YA_OFFSET_H, data[2]);
	i2c_write_register(MPU9250_ADDRESS, MPU9250_YA_OFFSET_L, data[3]);
	i2c_write_register(MPU9250_ADDRESS, MPU9250_ZA_OFFSET_H, data[4]);
	i2c_write_register(MPU9250_ADDRESS, MPU9250_ZA_OFFSET_L, data[5]);
}

void mpu9250_calibrate(float* accelDest, float* gyroDest) {
	uint8_t data[12];
	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
	uint16_t ii, packet_count, fifo_count;
	uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g
	int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
	int32_t gyro_temp[3] = {0, 0, 0}, accel_temp[3] = {0, 0, 0};

	// Reset device
	i2c_write_register(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x80);
	DelayMs(50);

	// get stable time source; Auto select clock source to be P LL gyroscope reference if ready
	// else use the internal oscillator, bits 2:0 = 001
	i2c_write_register(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x01);
	i2c_write_register(MPU9250_ADDRESS, MPU9250_PWR_MGMT_2, 0x00);
	DelayMs(15);

	// Configure device for bias calculation
	i2c_write_register(MPU9250_ADDRESS, MPU9250_INT_ENABLE, 0x00);   // Disable all interrupts
	i2c_write_register(MPU9250_ADDRESS, MPU9250_FIFO_EN, 0x00);      // Disable FIFO
	i2c_write_register(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x00);   // Turn on internal clock source
	i2c_write_register(MPU9250_ADDRESS, MPU9250_I2C_MST_CTRL, 0x00); // Disable I2C master
	i2c_write_register(MPU9250_ADDRESS, MPU9250_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	i2c_write_register(MPU9250_ADDRESS, MPU9250_USER_CTRL, 0x0C);    // Reset FIFO and DMP
	DelayMs(15);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	i2c_write_register(MPU9250_ADDRESS, MPU9250_CONFIG, 0x01);       // Set low-pass filter to 188 Hz
	i2c_write_register(MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, 0x00);   // Set sample rate to 1 kHz
	i2c_write_register(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	i2c_write_register(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
	DelayMs(15);

	i2c_write_register(MPU9250_ADDRESS, MPU9250_USER_CTRL, 0x40);   // Enable FIFO
	i2c_write_register(MPU9250_ADDRESS, MPU9250_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	DelayMs(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	i2c_write_register(MPU9250_ADDRESS, MPU9250_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	i2c_readmulti_register(MPU9250_ADDRESS, MPU9250_FIFO_COUNTH, &data[0], 2); // read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count / 12;// How many sets of full gyro and accelerometer data for averaging

	// Get the average
	for (ii = 0; ii < packet_count; ii++) {
		i2c_readmulti_register(MPU9250_ADDRESS, MPU9250_FIFO_R_W, &data[0], 12); // read data for averaging

		accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]); // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]);
		accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]);

		gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]);
		gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]);
		gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]);

		accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];
	}

	accel_bias[0] /= packet_count;
	accel_bias[1] /= packet_count;
	accel_bias[2] /= packet_count;

	accel_bias[0] = (int32_t)(accel_bias[0] << 16) / accelsensitivity * 2048;
	accel_bias[1] = (int32_t)(accel_bias[1] << 16) / accelsensitivity * 2048;
	accel_bias[2] = (int32_t)(accel_bias[2] << 16) / accelsensitivity * 2048;


	 // Remove gravity from the z-axis accelerometer bias calculation
	 /*if(accel_bias[2] > 0L) {
		 accel_bias[2] -= 1000;
	 }  else {
		 accel_bias[2] += 1000;
	 }*/

	// The accel_bias values are stored in +/-8g format
	for (int i = 0; i < 3; ++i) {
		accel_bias[i] = accel_bias[i] >> 16;
	}

	mpu9250_set_accel_bias(accel_bias);

	gyro_bias[0]  = (int32_t)(gyro_bias[0] << 16) / gyrosensitivity / packet_count;
	gyro_bias[1]  = (int32_t)(gyro_bias[1] << 16) / gyrosensitivity / packet_count;
	gyro_bias[2]  = (int32_t)(gyro_bias[2] << 16) / gyrosensitivity / packet_count;


	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;

	// Push gyro biases to hardware registers
	//i2c_write_register(MPU9250_ADDRESS, MPU9250_XG_OFFSET_H, data[0]);
	//i2c_write_register(MPU9250_ADDRESS, MPU9250_XG_OFFSET_L, data[1]);
	//i2c_write_register(MPU9250_ADDRESS, MPU9250_YG_OFFSET_H, data[2]);
	//i2c_write_register(MPU9250_ADDRESS, MPU9250_YG_OFFSET_L, data[3]);
	//i2c_write_register(MPU9250_ADDRESS, MPU9250_ZG_OFFSET_H, data[4]);
	//i2c_write_register(MPU9250_ADDRESS, MPU9250_ZG_OFFSET_L, data[5]);

	// Output scaled gyro biases for display in the main program
	gyroDest[0] = (float) gyro_bias[0];
	gyroDest[1] = (float) gyro_bias[1];
	gyroDest[2] = (float) gyro_bias[2];

	// Output scaled accelerometer biases for display in the main program
	accelDest[0] = (float)accel_bias[0];
	accelDest[1] = (float)accel_bias[1];
	accelDest[2] = (float)accel_bias[2];
}

int16_t readTemperature()
{
	uint8_t rawData[2];  // x/y/z gyro register data stored here
	i2c_readmulti_register(MPU9250_ADDRESS, MPU9250_TEMP_OUT_H, &rawData[0], 2);  // Read the two raw data registers sequentially into data array

	uint16_t temp = ((int16_t)rawData[0] << 8) | rawData[1];  // Turn the MSB and LSB into a 16-bit value
	uint16_t celsius = temp / 334 + 21.00; // Convert raw data to celsius
	uint16_t fahr = (celsius * 10 * 18) / 100 + 32; // Convert celsius to Farhenheit

	return fahr;
}

void mpu9250_self_test() {
	uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
	uint8_t selfTest[6] = {0, 0, 0, 0, 0, 0};
	float factoryTrim[6] = {0, 0, 0, 0, 0, 0};
	int16_t gAvg[3] = {0, 0, 0};
	int16_t aAvg[3] = {0, 0, 0};
	int16_t aSTAvg[3] = {0, 0, 0};
	int16_t gSTAvg[3] = {0, 0, 0};
	uint8_t FS = 0;

	i2c_write_register(MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, 0x00); 		// Set the gyro rate to 1 kHz
	i2c_write_register(MPU9250_ADDRESS, MPU9250_CONFIG, 0x02);			// Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	i2c_write_register(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, FS<<3); 	// Set full scale range for the gyro to 250 dps
	i2c_write_register(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2, 0x02); 	// Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	i2c_write_register(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, FS<<3); // Set full scale range for the accelerometer to 2g

	// Get the current values gyro and accelerometer
	for (int i = 0; i < 200; i++) {
		// Read the six raw data register into data array
		i2c_readmulti_register(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, &rawData[0], 6);
		aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
		aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

		// Read the six raw data register into data array
		i2c_readmulti_register(MPU9250_ADDRESS, MPU9250_GYRO_XOUT_H, &rawData[0], 6);
		gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
		gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
	}

	for (int i = 0; i < 3; i++) {  // Get average of 200 values and store as average current readings
		aAvg[i] /= 200;
		gAvg[i] /= 200;
	}

	printf("Accel: %d, Y: %d, Z: %d\n\r", aAvg[0], aAvg[1], aAvg[2]);
	printf("Gyro: %d, Y: %d, Z: %d\n\r", gAvg[0], gAvg[1], gAvg[2]);

	// Configure the accelerometer for self-test
	i2c_write_register(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
	i2c_write_register(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	DelayUs(1000);  // Delay a while to let the device stabilize

	for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer
		i2c_readmulti_register(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, &rawData[0], 6);  // Read the six raw data registers into data array
		aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

		i2c_readmulti_register(MPU9250_ADDRESS, MPU9250_GYRO_XOUT_H, &rawData[0], 6);  // Read the six raw data registers sequentially into data array
		gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
	}

	for (int ii = 0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}

	printf("Accel: %d, Y: %d, Z: %d\n\r", aSTAvg[0], aSTAvg[1], aSTAvg[2]);
	printf("Gyro: %d, Y: %d, Z: %d\n\r", gSTAvg[0], gSTAvg[1], gSTAvg[2]);

	// Configure the gyro and accelerometer for normal operation
	i2c_write_register(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0x00);
	i2c_write_register(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG,  0x00);
	DelayUs(1000);  // Delay a while to let the device stabilize

	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	i2c_read_register(MPU9250_ADDRESS, MPU9250_SELF_TEST_X_ACCEL, &selfTest[0]); // X-axis accel self-test results
	i2c_read_register(MPU9250_ADDRESS, MPU9250_SELF_TEST_Y_ACCEL, &selfTest[1]); // Y-axis accel self-test results
	i2c_read_register(MPU9250_ADDRESS, MPU9250_SELF_TEST_Z_ACCEL, &selfTest[2]); // Z-axis accel self-test results
	i2c_read_register(MPU9250_ADDRESS, MPU9250_SELF_TEST_X_GYRO, &selfTest[3]);  // X-axis gyro self-test results
	i2c_read_register(MPU9250_ADDRESS, MPU9250_SELF_TEST_Y_GYRO, &selfTest[4]);  // Y-axis gyro self-test results
	i2c_read_register(MPU9250_ADDRESS, MPU9250_SELF_TEST_Z_GYRO, &selfTest[5]);  // Z-axis gyro self-test results

	// Retrieve factory self-test value from self-test code reads
	factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
	factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
	factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
	factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
	factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
	factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim
	// of the Self-Test Response
	// To get percent, must multiply by 100
	// Report percent differences

	int32_t dest[6];

	dest[0] = 100.0 * ((float)(aSTAvg[0] - aAvg[0])) / factoryTrim[0];
	dest[1] = 100.0 * ((float)(aSTAvg[1] - aAvg[1])) / factoryTrim[1];
	dest[2] = 100.0 * ((float)(aSTAvg[2] - aAvg[2])) / factoryTrim[2];
	printf("Accel Percentage: %d, Y: %d, Z: %d\n\r", (int)dest[0], (int)dest[1], (int)dest[2]);

	dest[3] = 100.0 * ((float)(gSTAvg[3] - gAvg[3])) / factoryTrim[3];
	dest[4] = 100.0 * ((float)(gSTAvg[4] - gAvg[4])) / factoryTrim[4];
	dest[5] = 100.0 * ((float)(gSTAvg[5] - gAvg[5])) / factoryTrim[5];
	printf("Gyro Percentage: %d, Y: %d, Z: %d\n\r", (int)dest[3], (int)dest[4], (int)dest[5]);
}

void mpu9250_reset() {
	// Write a one to bit 7 reset bit; toggle reset device
	i2c_write_register(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x80);
}

void mpu9250_cal(uint16_t* accelOut) {
	int16_t accelTemp[3];
	int16_t accelData[3];

	int x = 0;
	while (x < 200) {

		mpu9250_readAccelData(accelTemp);

		accelData[0] += accelTemp[0];
		accelData[1] += accelTemp[1];
		accelData[2] += accelTemp[2];

		DelayUs(1000);
		x++;
	}

	accelData[0] = accelData[0] / x;
	accelData[1] = accelData[1] / x;
	accelData[2] = accelData[2] / x;
}

int main(void)
{
	// Initialize delay
   	DelayInit();

	// Print the CPU System clock speed
	printf("CPU SystemCoreClock is %u Hz\r", (unsigned int)SystemCoreClock);

	// Initialize the IIC bus
	i2c_init();

	// Reset device
	//mpu9250_reset();

	//mpu9250_self_test();

	// Calibrate the MPU9250
	//mpu9250_calibrate(accelBias, gyroBias);

	// Print out Gyroscope bias
	/*printf("Gyroscope Biases (o/s) X: %d, Y: %d, Z: %d\r", (int)gyroBias[0],
			(int)gyroBias[1], (int)gyroBias[2]);*/

	// Initialize MPU9250
	mpu9250_init();

	mpu9250_cal(accelBias);

	// Print out Accelerometer bias
	printf("Accelerometer Biases (mg) X: %d, Y: %d, Z: %d\r", accelBias[0],
			accelBias[1], accelBias[2]);

	// Get the temperature and print it
	uint16_t temp = readTemperature();
	printf("Temperature: %d F\n\r", temp);

	int16_t gyroData[3], accelData[3];
    while(1) {
    	mpu9250_readGyroData(gyroData);
    	mpu9250_readAccelData(accelData);
    }
}
