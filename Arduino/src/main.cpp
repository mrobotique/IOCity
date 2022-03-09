#include <Arduino.h>
#include "MPU9250.h"
#include <Adafruit_BMP280.h>
#include "MQ135.h"
#include <dht.h>
#include "blinker.h"

#define REF_PRESSURE  1013.25// the pressure(hPa) at sea level in day
#define REF_CO2 400 // 

#define DHTPIN 2 

dht DHT;
MPU9250 mpu;
Adafruit_BMP280 bme; // I2C
MQ135 gasSensor = MQ135(A0); // Attach sensor to pin A0
Flasher led(13);

unsigned long DHT11_samplingTime = 1000; //in [ms]
unsigned long MPU_samplingTime = 25; //in [ms]
unsigned long MQ_samplingTime = 100; //in [ms]

unsigned long previous_DHT11_samplingTime = millis();
unsigned long previous_MPU_samplingTime = millis();
unsigned long previous_MQ_samplingTime = millis();

float Ax, Ay, Az;
float roll, pitch, yaw;
float co2, co2_c;
float temp, hum, press, alt;

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
        Serial.println("MPU connection failed.");
        led.Update_mode(LED_ERROR);
        while (1) {
            led.Update();            
        }
    }
    led.Update_mode(LED_SHORT_DOUBLE);
}


void dataSender(){
        Serial.print("BOT");
        Serial.print(",");        
        Serial.print(millis());
        Serial.print(",");
        Serial.print(roll);
        Serial.print(",");
        Serial.print(pitch);
        Serial.print(",");
        Serial.print(yaw);
        Serial.print(",");
        Serial.print(Ax);
        Serial.print(",");
        Serial.print(Ay);
        Serial.print(",");
        Serial.print(Az);
        Serial.print(",");
        Serial.print(temp);
        Serial.print(",");
        Serial.print(hum);
        Serial.print(",");
        Serial.print(press);
        Serial.print(",");
        Serial.print(alt);
        Serial.print(",");
        Serial.print(co2);
        Serial.print(",");
        Serial.print(co2_c);
        Serial.print(",");
        Serial.println("EOT");
}

    void sequencer (){
    unsigned long top_time = millis();    
    //check MPU
    if (top_time - previous_MPU_samplingTime >= MPU_samplingTime){
        if (mpu.update()) {
            previous_MPU_samplingTime = top_time;
            Ax = mpu.getAccX();
            Ay = mpu.getAccY();
            Az = mpu.getAccZ();
        }     
    }

    //check MQ
    if (top_time- previous_MQ_samplingTime >= MQ_samplingTime){
        previous_MQ_samplingTime = top_time;
        co2 = gasSensor.getPPM() + REF_CO2; //CO2
        co2_c = gasSensor.getCorrectedPPM(temp, hum) + REF_CO2; //Corrected CO2
        temp = bme.readTemperature(); //C
        press = (bme.readPressure()/1000); //KPa
        alt = bme.readAltitude(REF_PRESSURE); //[m]

        if (mpu.update()) {
            roll = mpu.getRoll();
            pitch = mpu.getPitch();
            yaw = mpu.getYaw();
        }   
    
        dataSender();
    }

    //check DHT11
    if (top_time - previous_DHT11_samplingTime >= DHT11_samplingTime){
        previous_DHT11_samplingTime = top_time;       
        DHT.read11(DHTPIN);
        hum = DHT.humidity;
        //digitalWrite(LED_BUILTIN, !(digitalRead(LED_BUILTIN)));
        
    }
}


void loop() {
    sequencer();
    led.Update();
}