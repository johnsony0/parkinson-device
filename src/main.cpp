
#include <iostream>
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

#define FFT_SIZE 128
#define SAMPLE_RATE 52
#define SAMPLE_RATE_HEX 0x30
#define SAMPLE_DURATION 3000ms

struct FFTResult {
  float32_t tremorEnergy;
  float32_t dyskinesiaEnergy;
};
FFTResult freq;

// LED Pins for Feedback
Thread thread_tremor;
Thread thread_dyskinesia;
DigitalOut led_tremor(LED2);       // Tremor Indicator 
DigitalOut led_dyskinesia(LED3);   // Dyskinesia Indicator 

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

void show_results(float32_t *magnitude){
  float32_t resolution = (float)SAMPLE_RATE / (float)FFT_SIZE;
  float32_t maxValue = 0;
  uint32_t maxIndex = 0;

  for (int i = 1; i < FFT_SIZE / 2; i++) {
    if (magnitude[i] > maxValue) {
      maxValue = magnitude[i];
      maxIndex = i;
    }
  }

  printf("\n Max magnitude: %.1f at bin %lu (%.2f Hz)\r\n", maxValue, maxIndex, resolution*maxIndex);
}

void get_results(float32_t *magnitude) {
  float32_t tremor_energy = 0.0f;
  float32_t dyskinesia_energy = 0.0f;

  for (int i = 1; i < FFT_SIZE / 2; i++) {
    float32_t freq = ((float)SAMPLE_RATE * i) / (float)FFT_SIZE;
    if (freq >= 3.0f && freq <= 5.0f) {
      tremor_energy += magnitude[i];
    } else if (freq > 5.0f && freq <= 7.0f) {
      dyskinesia_energy += magnitude[i];
    }
  }
  
  /*printf("\n Max magnitude: %.1f at bin %lu (%.2f Hz)\r\n", 
  maxValue, maxIndex, resolution * maxIndex);*/
  
  freq = {tremor_energy, dyskinesia_energy};
}

void display_results() {
  while(1){
    int tremor_level = freq.tremorEnergy > 3000 ? 3 : freq.tremorEnergy > 1500 ? 2 : freq.tremorEnergy > 500 ? 1 : 0;
    int dysk_level   = freq.dyskinesiaEnergy > 3000 ? 3 : freq.dyskinesiaEnergy > 1500 ? 2 : freq.dyskinesiaEnergy > 500 ? 1 : 0;
    if (tremor_level && dysk_level){ //both levels are not 0
      if (tremor_level > dysk_level){  //tremor clearly greater
        led_tremor = 1;
      } else if (tremor_level < dysk_level){ //dysk clearly greater
        led_dyskinesia = 1;
      } else if (tremor_level == dysk_level){ //neither significantly different
        led_tremor = 1;
        led_dyskinesia = 1;
      }
    } else {
      led_tremor = 0;
    led_dyskinesia = 0;
    }
  }
}

void tremor_display(){
  while(1){
    int tremor_level = freq.tremorEnergy > 3000 ? 6 : freq.tremorEnergy > 2500 ? 5 : freq.tremorEnergy > 2000 ? 4 : freq.tremorEnergy > 1500 ? 3 : freq.tremorEnergy > 1000 ? 2 : freq.tremorEnergy > 500 ? 1 : 0;
    if(tremor_level>0){
      chrono::milliseconds tremor_delay = 500ms/(tremor_level); //blink speed based on intensity
      led_tremor = 1;
      ThisThread::sleep_for(tremor_delay);
      led_tremor = 0;
      ThisThread::sleep_for(tremor_delay);
    } else {
      led_tremor = 0;
      ThisThread::sleep_for(500ms);
    }
  }
}

void dyskinesia_display(){
  while(1){
    int dyskinesia_level = freq.dyskinesiaEnergy > 3000 ? 6 : freq.dyskinesiaEnergy > 2500 ? 5 : freq.dyskinesiaEnergy > 2000 ? 4 : freq.dyskinesiaEnergy > 1500 ? 3 : freq.dyskinesiaEnergy > 1000 ? 2 : freq.dyskinesiaEnergy > 500 ? 1 : 0;
    if(dyskinesia_level>0){
      chrono::milliseconds dyskinesia_delay = 500ms/(dyskinesia_level); //blink speed based on intensity
      led_dyskinesia = 1;
      ThisThread::sleep_for(dyskinesia_delay);
      led_dyskinesia = 0;
      ThisThread::sleep_for(dyskinesia_delay);
    } else {
      led_dyskinesia = 0;
      ThisThread::sleep_for(500ms);
    }
  }
}

int main() {
  // Configurations
  i2c.frequency(400000);
    uint8_t id = read_register(WHO_AM_I);
  printf("WHO_AM_I = 0x%02X (Expected: 0x6A)\r\n", id);

  if (id != 0x6A) {
    printf("Error: LSM6DSL sensor not found!\r\n");
    while (1) { /* Stop here */ }
  }

  // Configure the accelerometer (208 Hz, ±2g range)
  write_register(CTRL1_XL, SAMPLE_RATE_HEX);
  printf("Accelerometer configured: 26 Hz, ±2g range\r\n");
  
  // Configure the gyroscope (208 Hz, ±250 dps range)
  write_register(CTRL2_G, SAMPLE_RATE_HEX);
  printf("Gyroscope configured: 26 Hz, ±250 dps range\r\n");
  
  // Conversion factors for ±2g and ±250 dps
  const float GYRO_SENSITIVITY = 8.75f;  // mdps/LSB for ±250 dps range

  // Initialize the FFT instance
  arm_status status = arm_rfft_fast_init_f32(&FFT_Instance, FFT_SIZE);
  if (status != ARM_MATH_SUCCESS) {
    printf("Error: RFFT initialization failed!\r\n");
    while (1){
      printf("Error: RFFT initialization failed!\r\n");
    }; // Or some other error handling
  }

  //start threads that display either conditions and intensity
  thread_tremor.start(tremor_display);
  thread_dyskinesia.start(dyskinesia_display);

  // Main loop
  while (1) {
    //Initialize arrays
    float32_t gyro_x[FFT_SIZE];
    float32_t gyro_y[FFT_SIZE];
    float32_t gyro_z[FFT_SIZE];

    float32_t fft_input[FFT_SIZE];
    float32_t fft_out[FFT_SIZE];
    float32_t magnitude[FFT_SIZE/2];

    for(int i=0;i<FFT_SIZE;i++){
      // Read raw gyroscope values
      int16_t gyro_x_raw = read_16bit_value(OUTX_L_G, OUTX_H_G);
      int16_t gyro_y_raw = read_16bit_value(OUTY_L_G, OUTY_H_G);
      int16_t gyro_z_raw = read_16bit_value(OUTZ_L_G, OUTZ_H_G);

      // Convert gyroscope values from raw to dps
      gyro_x[i] = gyro_x_raw * GYRO_SENSITIVITY / 1000.0f;
      gyro_y[i] = gyro_y_raw * GYRO_SENSITIVITY / 1000.0f;
      gyro_z[i] = gyro_z_raw * GYRO_SENSITIVITY / 1000.0f;

      fft_input[i] = sqrtf(gyro_x[i] * gyro_x[i] + gyro_y[i] * gyro_y[i] + gyro_z[i] * gyro_z[i]);
    }

    arm_rfft_fast_f32(&FFT_Instance, fft_input,fft_out, 0);      
    arm_cmplx_mag_f32(fft_out, magnitude, FFT_SIZE/2);
    //show_results(magnitude);
    get_results(magnitude);
    printf("\n Tremor Energy: %.1f, Dyskinesia Energy: %.1f \n", freq.tremorEnergy, freq.dyskinesiaEnergy);
    //display_results(freq);

    ThisThread::sleep_for(500ms);
  }
  return 0;
}
