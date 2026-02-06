#include "logger.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <RTClib.h>
#include <SD.h>
#include <MS5611.h>
#include <TinyGPS++.h>
#include <MPU6050.h>
#include <MPU9250.h>
#include <SPI.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <STM32TimerInterrupt.h>

// BOARD IS STM32F411CEU6

/////// GLOBALS & CONSTANTS ///////
#define IGNI_A_ALT 0
#define IGNI_B_ALT 0
#define IGNI_C_ALT 0
#define IGNI_D_ALT 0
#define IGNI_ARMED_BUFFER 3
#define BUZZ_EN 1
#define MAX_BUFFER_SIZE 300;

/////// PIN DEFINITIONS ///////
#define GPS_TX    PA9
#define GPS_RX    PA10
#define LORA_TX   PA2
#define LORA_RX   PA3
#define SDA       PB9
#define SCL       PB8
#define CS        PA4
#define SCK       PA5
#define MISO      PA6 //d0
#define MOSI      PA7 //d1

#define SDA2      PB3
#define SCL2      PB10
#define IGN_A     PA0
#define IGN_B     PA1
#define IGN_C     PB0
#define IGN_D     PB1
#define CONT_A    PB12
#define CONT_B    PB14
#define CONT_C    PB15
#define CONT_D    PC13
#define BUZZ      PA8
#define ARMED_IN  PB2
#define SERVO_A   PB5
#define SERVO_B   PB4
#define SERVO_C   PB0
#define SERVO_D   PB1

HardwareSerial Serial2(LORA_RX, LORA_TX);

STATE current_state = STATE_NONE;
KALMAN_DATA current_data, last_data, prediction_data;
KALMAN_DATA kdata[300];

MS5611 altimeter;
TinyGPSPlus gps;
MPU6050 mpu;
TRIGGER TRIG[4];

//bool isMinAltitude = false;
//bool isGPSValid = false;
double altitude = 0;
//double speed = 0;
//double start_altitude = 0;
//int count_alt = 0;
int loop_count = 0;
int32_t last_micros = 0;
//int32_t deltat = 0;
//int ready_flags = 0;
double last_altitude = 0;
float totalAccel = 0;
int launch_confirmations = 0;
bool igni_armed[4];
int igni_confirm[4];

float dt = 0;
float launch_timer = 0;
float altitude_at_start = 0;
bool tracking_launch = false;

/////// FUNCTION DECLARATIONS ///////
void zeroKalman(KALMAN_DATA *kd);
void getKalman(KALMAN_DATA *kd);
float mpuToMetPerSec(int16_t);
float mpuToDegPerSec(int16_t);
void saveToSD(void);
void radioSerial(int);
void handleIgniters(void);
void i2cAddress(void);

void setup() {
    Serial1.begin(9600);
    Serial2.begin(9600);

    // I2C Init
    Wire.setSDA(SDA);
    Wire.setSCL(SCL);
    Wire.begin();
    Wire.setClock(100000);

    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

    Wire.beginTransmission(0x77);
    Wire.write(0x1E);
    Wire.endTransmission(true);
    delay(20);

    // SPI & SD Init
    if (!SD.begin(CS)) {
        Serial.println("SD Initialization failed! Continuing without logging.");
    } 
    else {
        Serial.println("SD Initialization successful.");
    }

    zeroKalman(&current_data);
    zeroKalman(&last_data);
    zeroKalman(&prediction_data);

    delay(300);
    altimeter.begin();
    mpu.initialize();
    mpu.CalibrateAccel(20);
    mpu.CalibrateGyro(20);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

    pinMode(IGN_A, OUTPUT);
    pinMode(IGN_B, OUTPUT);
    pinMode(IGN_C, OUTPUT);
    pinMode(IGN_D, OUTPUT);
    digitalWrite(IGN_A, LOW);
    digitalWrite(IGN_B, LOW);
    digitalWrite(IGN_C, LOW);
    digitalWrite(IGN_D, LOW);

    for (int i = 0; i < 300; i++) {
        zeroKalman(&kdata[i]);
    }

    current_state = STATE_READY;
}

void loop() {
    // Timing Calculation
    uint32_t current_micros = micros();
    if (last_micros == 0) {
        last_micros = current_micros;
        return;
    }
    dt = (current_micros - last_micros) / 1000000.0f;
    last_micros = current_micros;

    // GPS Processing
    if (Serial1.available()) {
        char c = Serial1.read();
        gps.encode(c);
    }

    // Sensor Reading
    altimeter.read();
    last_altitude = altitude;
    altitude = altimeter.getAltitude();

    int16_t x_acl, y_acl, z_acl;
    int16_t x_rot, y_rot, z_rot;
    mpu.getAcceleration(&x_acl, &y_acl, &z_acl);
    mpu.getRotation(&x_rot, &y_rot, &z_rot);

    float xa = mpuToMetPerSec(x_acl);
    float ya = mpuToMetPerSec(y_acl);
    float za = mpuToMetPerSec(z_acl);
    totalAccel = sqrtf(xa * xa + ya * ya + za * za);

    // State Machine
    switch (current_state) {
        case STATE_NONE:
            break;

        case STATE_READY:
            // Acceleration Trigger
            if (totalAccel > 30 && !tracking_launch) {
                tracking_launch = true;
                launch_confirmations++;
                altitude_at_start = altitude;
                launch_timer = 0;
            }

            // Altitude Verification (10m climb)
            if (tracking_launch) {
                launch_timer += dt;
                if ((altitude - altitude_at_start) >= 10.0f) {
                    current_state = STATE_LAUNCHED;
                    tracking_launch = false;
                }
                // Optional: Timeout if 10m not reached in 2 seconds
                if (launch_timer > 2.0f) tracking_launch = false;
            } else {
                launch_confirmations = 0;
            }

            if (launch_confirmations > 30) {
                current_state = STATE_LAUNCHED;
            }
            break;

        case STATE_LAUNCHED:
            handleIgniters();
            break;

        case STATE_LANDED:
            break;
    }

    getKalman(&kdata[loop_count]);

    if (loop_count >= 300) {
        saveToSD();
        loop_count = 0;
        Serial.print("NOW");
    } else {
        loop_count++;
        Serial.print(loop_count);
    }

    radioSerial(1);
}

void getKalman(KALMAN_DATA *kd) {
    kd->alt_altitude = altimeter.getAltitude();
    kd->alt_pressure = altimeter.getPressure();
    kd->alt_temp = altimeter.getTemperature();

    kd->gps_day = gps.date.day();
    kd->gps_month = gps.date.month();
    kd->gps_year = gps.date.year();

    kd->gps_hou = gps.time.hour();
    kd->gps_min = gps.time.minute();
    kd->gps_sec = gps.time.second();

    kd->gps_speed = gps.speed.mps();

    kd->gps_altitude = gps.altitude.meters();
    kd->gps_lat = gps.location.lat();
    kd->gps_lng = gps.location.lng();

    kd->gyr_accl_x = mpu.getAccelerationX();
    kd->gyr_accl_y = mpu.getAccelerationY();
    kd->gyr_accl_z = mpu.getAccelerationZ();

    kd->gyr_accl_x = mpu.getRotationX();
    kd->gyr_accl_y = mpu.getRotationY();
    kd->gyr_accl_z = mpu.getRotationZ();

    kd->stm_milis = millis();

    kd->calc_accl = totalAccel;
}

/////// HELPER FUNCTIONS ///////

float mpuToMetPerSec(int16_t value) {
    return (value / 2048.0) * 9.80665;
}

float mpuToDegPerSec(int16_t value) {
    return value / 16.4;
}

void handleIgniters(void) {
    // Check Altitude Thresholds for Arming/Firing
    for (int i = 0; i < 4; i++) {
        float target_alt = (i == 0) ? IGNI_A_ALT : (i == 1) ? IGNI_B_ALT : (i == 2) ? IGNI_C_ALT : IGNI_D_ALT;
        int ign_pin = (i == 0) ? IGN_A : (i == 1) ? IGN_B : (i == 2) ? IGN_C : IGN_D;

        if ((altitude > target_alt) && (!igni_armed[i])) {
            igni_confirm[i]++;
        } else if ((altitude < target_alt) && (igni_armed[i])) {
            igni_confirm[i]++;
        } else {
            igni_confirm[i] = 0;
        }

        // Arming Logic
        if ((igni_confirm[i] > 10) && (altitude > (target_alt + IGNI_ARMED_BUFFER)) && (!igni_armed[i])) {
            igni_armed[i] = true;
        }
        // Firing Logic
        else if ((igni_confirm[i] > 10) && (altitude < target_alt) && (igni_armed[i])) {
            digitalWrite(ign_pin, HIGH);
        }
    }
}

void saveToSD() {
    File save_file = SD.open("TEST.TXT", FILE_WRITE);

    if (save_file) {
        for (int i = 0; i < 300; i++) {
            save_file.print(kdata[i].stm_milis);
            save_file.print(", ");
            save_file.print(kdata[i].gps_satilites);
            save_file.print(", ");
            save_file.print(String(kdata[i].gps_hou) + ":" + String(kdata[i].gps_min) + ":" + String(kdata[i].gps_sec));
            save_file.print(", ");
            save_file.print(String(kdata[i].gps_day) + "/" + String(kdata[i].gps_month) + "/" + String(kdata[i].gps_year));
            save_file.print(", ");
            save_file.print(kdata[i].calc_velocity);
            save_file.print(", ");
            save_file.print(String(kdata[i].gps_speed));
            save_file.print(", ");
            save_file.print(String(kdata[i].alt_altitude) + ", " + String(kdata[i].alt_pressure) + ", " + String(kdata[i].alt_temp));
        }

        save_file.close(); // Closing ensures data is saved if power is lost
    }
    else{ Serial.println("SD CARD ERROR");}
}

void radioSerial(int mode) {
    Serial.print("#ODN");
    Serial.print(" " + String(current_state));
    Serial.print(" " + String(altimeter.getAltitude()) + " " + String(altimeter.getPressure()) + " " + String(altimeter.getTemperature()));
    Serial.print(" " + String(totalAccel));
    Serial.print(" #MSE\n");
}

void i2cAddress() {
    byte error, address;
    int nDevices = 0;
    Serial.println("Scanning...");
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            Serial.print("Device found at address 0x");
            if (address < 16) Serial.print("0");
            Serial.println(address, HEX);
            nDevices++;
        }
    }
    if (nDevices == 0) Serial.println("No I2C devices found\n");
    delay(5000);
}