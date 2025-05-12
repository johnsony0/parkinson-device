
#include <mbed.h>
#include <arm_math.h>
#include <cmath>

// I2C setup for LSM6DSL
I2C i2c(PB_11, PB_10);  // SDA, SCL

#define LSM6DSL_ADDR (0x6A << 1)
#define OUTX_L_XL   0x28
#define OUTX_H_XL   0x29
#define OUTY_L_XL   0x2A
#define OUTY_H_XL   0x2B
#define OUTZ_L_XL   0x2C
#define OUTZ_H_XL   0x2D

#define FFT_SIZE 312
#define SAMPLE_RATE 104.0f
#define SAMPLE_DURATION 3000ms

// LED Pins for Feedback
DigitalOut led_tremor(LED1);       // Tremor Indicator
DigitalOut led_dyskinesia(LED2);   // Dyskinesia Indicator

// Buffers
float32_t magnitude_data[FFT_SIZE];
float32_t fft_input[2 * FFT_SIZE];
float32_t fft_output[FFT_SIZE];
arm_cfft_instance_f32 S;

struct FFTResult {
    float tremorEnergy;
    float dyskinesiaEnergy;
};

// I2C read functions
uint8_t read_register(uint8_t reg) {
    char data = reg;
    i2c.write(LSM6DSL_ADDR, &data, 1, true);
    i2c.read(LSM6DSL_ADDR, &data, 1);
    return (uint8_t)data;
}

int16_t read_16bit_value(uint8_t low_reg, uint8_t high_reg) {
    char low_byte = read_register(low_reg);
    char high_byte = read_register(high_reg);
    return (high_byte << 8) | low_byte;
}

// Collect 3 seconds of XYZ accelerometer data
void collect_data(int16_t *x, int16_t *y, int16_t *z) {
    for (int i = 0; i < FFT_SIZE; i++) {
        x[i] = read_16bit_value(OUTX_L_XL, OUTX_H_XL);
        y[i] = read_16bit_value(OUTY_L_XL, OUTY_H_XL);
        z[i] = read_16bit_value(OUTZ_L_XL, OUTZ_H_XL);
        thread_sleep_for(1000 * 3 / FFT_SIZE);  // ~9.6ms interval for 104Hz
    }
}

// Convert XYZ to magnitude
void compute_magnitude(int16_t *x, int16_t *y, int16_t *z) {
    for (int i = 0; i < FFT_SIZE; i++) {
        float fx = (float)x[i];
        float fy = (float)y[i];
        float fz = (float)z[i];
        magnitude_data[i] = sqrtf(fx * fx + fy * fy + fz * fz);
    }
}

void fill_fft_input() {
    for (int i = 0; i < FFT_SIZE; i++) {
        fft_input[2 * i] = magnitude_data[i];
        fft_input[2 * i + 1] = 0.0f;
    }
}

// Perform FFT and return result
FFTResult perform_fft_and_classify() {
    fill_fft_input();

    arm_cfft_init_f32(&S, FFT_SIZE);
    arm_cfft_f32(&S, fft_input, 0, 1);
    arm_cmplx_mag_f32(fft_input, fft_output, FFT_SIZE);

    float tremor_energy = 0.0f;
    float dyskinesia_energy = 0.0f;

    for (int i = 1; i < FFT_SIZE / 2; i++) {
        float freq = (SAMPLE_RATE * i) / FFT_SIZE;
        if (freq >= 3.0f && freq <= 5.0f) {
            tremor_energy += fft_output[i];
        } else if (freq > 5.0f && freq <= 7.0f) {
            dyskinesia_energy += fft_output[i];
        }
    }

    FFTResult result = { tremor_energy, dyskinesia_energy };
    return result;
}

// Map energy to LED blink
void indicate_with_leds(FFTResult result) {
    int tremor_level = result.tremorEnergy > 3000 ? 3 : result.tremorEnergy > 1500 ? 2 : result.tremorEnergy > 500 ? 1 : 0;
    int dysk_level   = result.dyskinesiaEnergy > 3000 ? 3 : result.dyskinesiaEnergy > 1500 ? 2 : result.dyskinesiaEnergy > 500 ? 1 : 0;

    for (int i = 0; i < 3; i++) {
        led_tremor = (i < tremor_level);
        led_dyskinesia = (i < dysk_level);
        thread_sleep_for(200);
        led_tremor = 0;
        led_dyskinesia = 0;
        thread_sleep_for(100);
    }
}

int main() {
    int16_t x[FFT_SIZE], y[FFT_SIZE], z[FFT_SIZE];

    while (true) {
        collect_data(x, y, z);
        compute_magnitude(x, y, z);
        FFTResult result = perform_fft_and_classify();
        indicate_with_leds(result);
    }
}
