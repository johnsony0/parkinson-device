#include <mbed.h>
#include <arm_math.h>

I2C i2c(PB_11, PB_10);  // I2C2: SDA = PB11, SCL = PB10

#define LSM6DSL_ADDR (0x6A << 1)  // Equals 0xD4
#define WHO_AM_I    0x0F  // ID register - should return 0x6A
#define CTRL1_XL    0x10  // Accelerometer control register to configure range
#define CTRL2_G     0x11  // Gyroscope control register to configure range
#define OUTX_L_XL   0x28  // XL X-axis (low byte)
#define OUTX_H_XL   0x29  // XL X-axis (high byte)
#define OUTY_L_XL   0x2A  // XL Y-axis (low byte)
#define OUTY_H_XL   0x2B  // XL Y-axis (high byte)
#define OUTZ_L_XL   0x2C  // XL Z-axis (low byte)
#define OUTZ_H_XL   0x2D  // XL Z-axis (high byte)
#define OUTX_L_G    0x22  // Gyro X-axis (low byte)
#define OUTX_H_G    0x23  // Gyro X-axis (high byte)
#define OUTY_L_G    0x24  // Gyro Y-axis (low byte)
#define OUTY_H_G    0x25  // Gyro Y-axis (high byte)
#define OUTZ_L_G    0x26  // Gyro Z-axis (low byte)
#define OUTZ_H_G    0x27  // Gyro Z-axis (high byte)

#define FFT_SIZE 312
//26 samples per second, we store in intervals of 3 seconds
#define SAMPLE_RATE 104.0f
//Our max sampling rate required is 7Hz
//need double that for Sampling Theorem
//for a minimum rate of 14Hz, however
//the LSM6DSL data rate closest is 26Hz
#define SAMPLE_DURATION 3000ms

arm_rfft_fast_instance_f32 FFT_Instance;

// Write a value to a register
void write_register(uint8_t reg, uint8_t value) {
  char data[2] = {(char)reg, (char)value};
  i2c.write(LSM6DSL_ADDR, data, 2);
}

// Read a value from a register
uint8_t read_register(uint8_t reg) {
  char data = reg;
  i2c.write(LSM6DSL_ADDR, &data, 1, true); // No stop
  i2c.read(LSM6DSL_ADDR, &data, 1);
  return (uint8_t)data;
}

// Read a 16-bit value (combines low and high byte registers)
int16_t read_16bit_value(uint8_t low_reg, uint8_t high_reg) {
  char low_byte = read_register(low_reg);
  char high_byte = read_register(high_reg);
  return (high_byte << 8) | low_byte;
}

void show_results(float *magnitude) {
  float32_t resolution = SAMPLE_RATE / FFT_SIZE;
  float32_t maxValue;
  uint32_t maxIndex;

  arm_max_f32(magnitude, FFT_SIZE/2, &maxValue, &maxIndex);
  
  printf("Max magnitude: %.1f at bin %lu (%.2f Hz)\r\n", 
  maxValue, maxIndex, maxIndex * resolution);

  printf("Bin\tFreq (Hz)\tMagnitude\n");
 for (int i = 0; i < FFT_SIZE; i++) { // Show bins for debugging
  printf(" %d\t%.2f\t\t%.4f\n",i, i * resolution, magnitude[i]);
 }
}

int main() {
  // Setup I2C at 400kHz
  i2c.frequency(400000);
  
  // Check if sensor is connected
  uint8_t id = read_register(WHO_AM_I);
  printf("WHO_AM_I = 0x%02X (Expected: 0x6A)\r\n", id);

  if (id != 0x6A) {
    printf("Error: LSM6DSL sensor not found!\r\n");
    while (1) { /* Stop here */ }
  }

  // Configure the accelerometer (104 Hz, ±2g range)
  write_register(CTRL1_XL, 0x40);
  printf("Accelerometer configured: 26 Hz, ±2g range\r\n");
  
  // Configure the gyroscope (104 Hz, ±250 dps range)
  write_register(CTRL2_G, 0x40);
  printf("Gyroscope configured: 26 Hz, ±250 dps range\r\n");
  
  // Conversion factors for ±2g and ±250 dps
  const float ACC_SENSITIVITY = 0.061f;  // mg/LSB for ±2g range
  const float GYRO_SENSITIVITY = 8.75f;  // mdps/LSB for ±250 dps range
  
  //tentative idea, collect data on each axis, and do an fft on each
  //and if any are within the ranges, we signal it. Or we can aggregate/avg
  //all 3 axis. Unsure which is better. imo aggregating is better. Another
  //idea is sensor fusion, to combine data of all 6.

  // Initialize the FFT instance
  arm_status status = arm_rfft_fast_init_f32(&FFT_Instance, FFT_SIZE);
  

  // Main loop
  while (1) {
    float32_t fft_input[FFT_SIZE];
    float32_t fft_out[FFT_SIZE];
    float32_t magnitude[FFT_SIZE/2];
    //grab FFT_SIZE data points, capture 3 second intervals of data with 104Hz sampling rate
    for(int i=0;i<FFT_SIZE;i++){
      // Read raw accelerometer values
      int16_t acc_x_raw = read_16bit_value(OUTX_L_XL, OUTX_H_XL);
      int16_t acc_y_raw = read_16bit_value(OUTY_L_XL, OUTY_H_XL);
      int16_t acc_z_raw = read_16bit_value(OUTZ_L_XL, OUTZ_H_XL);
      
      // Read raw gyroscope values
      int16_t gyro_x_raw = read_16bit_value(OUTX_L_G, OUTX_H_G);
      int16_t gyro_y_raw = read_16bit_value(OUTY_L_G, OUTY_H_G);
      int16_t gyro_z_raw = read_16bit_value(OUTZ_L_G, OUTZ_H_G);
      
      // Convert accelerometer values from raw to g
      float32_t acc_x_g = acc_x_raw * ACC_SENSITIVITY / 1000.0f;
      float32_t acc_y_g = acc_y_raw * ACC_SENSITIVITY / 1000.0f;
      float32_t acc_z_g = acc_z_raw * ACC_SENSITIVITY / 1000.0f;
      
      // Convert gyroscope values from raw to dps
      float32_t gyro_x_dps = gyro_x_raw * GYRO_SENSITIVITY / 1000.0f;
      float32_t gyro_y_dps = gyro_y_raw * GYRO_SENSITIVITY / 1000.0f;
      float32_t gyro_z_dps = gyro_z_raw * GYRO_SENSITIVITY / 1000.0f;

      //example code of adding for fft input
      fft_input[i] = gyro_x_dps; 
    }
    //prints fft_input
    printf("Bin\t gyro dps\n");
    for(int i=0;i<FFT_SIZE;i++){
      printf(" %d\t\t%.4f\n",i, fft_input[i]);
    }
    //FFT code here
    /* Process the data through the RFFT module */
    arm_rfft_fast_f32(&FFT_Instance, fft_input, fft_out, 0);

    /* Process the data through the Complex Magnitude Module */
    arm_cmplx_mag_f32(fft_out, magnitude, FFT_SIZE/2);
    show_results(magnitude);

    ThisThread::sleep_for(1000ms);
  }
}