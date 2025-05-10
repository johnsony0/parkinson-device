
#include <mbed.h>
#include <arm_math.h>
#include <cmath>

I2C i2c(PB_11, PB_10);  // I2C2: SDA = PB11, SCL = PB10

#define LSM6DSL_ADDR (0x6A << 1)  // Equals 0xD4
#define WHO_AM_I    0x0F
#define CTRL1_XL    0x10
#define CTRL2_G     0x11
#define OUTX_L_XL   0x28
#define OUTX_H_XL   0x29
#define OUTY_L_XL   0x2A
#define OUTY_H_XL   0x2B
#define OUTZ_L_XL   0x2C
#define OUTZ_H_XL   0x2D

#define FFT_SIZE 312
#define SAMPLE_RATE 104.0f
#define SAMPLE_DURATION 3000ms

float32_t magnitude_data[FFT_SIZE];
float32_t fft_input[2 * FFT_SIZE];
float32_t fft_output[FFT_SIZE];
arm_cfft_instance_f32 S;

void write_register(uint8_t reg, uint8_t value) {
    char data[2] = {(char)reg, (char)value};
    i2c.write(LSM6DSL_ADDR, data, 2);
}

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

// Symptom classification and intensity quantification
void classify_symptom(float tremor_energy, float dyskinesia_energy) {
    const float detection_threshold = 10.0f;

    const char* symptom = "None";
    const char* intensity = "None";
    float dominant_energy = 0.0f;

    if (tremor_energy > detection_threshold && tremor_energy > dyskinesia_energy) {
        symptom = "Tremor";
        dominant_energy = tremor_energy;
    } else if (dyskinesia_energy > detection_threshold) {
        symptom = "Dyskinesia";
        dominant_energy = dyskinesia_energy;
    }

    if (strcmp(symptom, "None") != 0) {
        if (dominant_energy < 20.0f) {
            intensity = "Low";
        } else if (dominant_energy < 50.0f) {
            intensity = "Medium";
        } else {
            intensity = "High";
        }
    }

    printf("Symptom: %s\n", symptom);
    printf("Intensity: %s\n", intensity);
}


void perform_fft_and_classify() {
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

    printf("Tremor Energy (3–5Hz): %.3f\n", tremor_energy);
    printf("Dyskinesia Energy (5–7Hz): %.3f\n", dyskinesia_energy);

    // Replace this with actual function to pass values to Member C
    // send_to_classifier(tremor_energy, dyskinesia_energy);
    classify_symptom(tremor_energy, dyskinesia_energy);
}

int main() {
    printf("Starting FFT signal analysis demo...\n");

    // Mock data arrays (should be filled by actual sensor readings)
    int16_t accel_x[FFT_SIZE] = {0};
    int16_t accel_y[FFT_SIZE] = {0};
    int16_t accel_z[FFT_SIZE] = {0};

    // TODO: Replace with actual data sampling logic
    for (int i = 0; i < FFT_SIZE; i++) {
        accel_x[i] = 100 * sinf(2 * PI * 4.0f * i / SAMPLE_RATE);  // Simulate 4Hz
        accel_y[i] = 50 * sinf(2 * PI * 6.0f * i / SAMPLE_RATE);   // Simulate 6Hz
        accel_z[i] = 10;
    }

    compute_magnitude(accel_x, accel_y, accel_z);
    perform_fft_and_classify();

    while (true) {
        // idle loop
    }
}