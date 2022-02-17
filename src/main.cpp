#include <Arduino.h>
#include "MPU9250.h"
#include <Adafruit_BMP280.h>
#include "MQ135.h"


#define LOCAL_ALTITUDE  1013.25// the pressure(hPa) at sea level in day

MPU9250 mpu;
Adafruit_BMP280 bme; // I2C

MQ135 gasSensor = MQ135(A0); // Attach sensor to pin A0

void setup() {
    Serial.begin(115200);
    Wire.begin();
    bme.begin(0x76);
    delay(2000);

    MPU9250Setting setting;
    setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    setting.gyro_fchoice = 0x03;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

    if (!mpu.setup(0x68, setting)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
}


void print_roll_pitch_yaw() {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(", ");
    Serial.print(mpu.getRoll(), 2);

    Serial.print("\t | Ax, Ay, Az: ");
    Serial.print(mpu.getAccX(), 2);
    Serial.print(", ");
    Serial.print(mpu.getAccY(), 2);
    Serial.print(", ");
    Serial.print(mpu.getAccZ(), 2);

    Serial.print("  | Temperature(*C): ");
    Serial.print(bme.readTemperature());
    Serial.print("  Pressure(KPa): ");
    Serial.print(bme.readPressure()/1000);
    Serial.print("  ApproxAltitude(m): ");
    Serial.print(bme.readAltitude(LOCAL_ALTITUDE)); // this should be adjusted to your local forcase
    Serial.print ("  CO2 (ppm): ");
    Serial.println (gasSensor.getPPM() + 400);
    //Serial.println (gasSensor.getCorrectedPPM(temp, humidity));
}

void loop() {
    if (mpu.update()) {
        print_roll_pitch_yaw();
    }
}