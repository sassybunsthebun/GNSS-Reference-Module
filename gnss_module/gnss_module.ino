///BME280 LIBRARIES///
#include <Wire.h>
#include <Adafruit_BME280.h>

///ENCODER LIBRARIES///
#include "Adafruit_seesaw.h"
#include <seesaw_neopixel.h>

///BNO055 LIBRARIES///
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

///ICM20948 LIBRARIES///
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>

///INITIALIZE BME280///
Adafruit_BME280 bme; // use I2C interface for BME280 sensor 
// see library examples -> test/unified for different communication examples
#define SEALEVELPRESSURE_HPA (1013.25)

///INITIALIZE ENCODER///
#define SS_SWITCH 24
#define SS_NEOPIX 6

#define SEESAW_ADDR 0x36

Adafruit_seesaw ss;


///INITIALIZE BNO055///

Adafruit_BNO055 bno = Adafruit_BNO055(55);

///INITIALIZE ICM20948///
Adafruit_ICM20948 icm;
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing
// For SPI mode, we need a CS pin

///INITIALIE ICP10125///

#define NORMAL 0x6825
#define LOW_POWER 0x609C
#define LOW_NOISE 0x70DF
#define ULTRA_LOW_NOISE 0x7866
#define NORMAL_T_FIRST 0x6825
#define NORMAL_P_FIRST 0x48A3
#define LOW_POWER_T_FIRST 0x609C
#define LOW_POWER_P_FIRST 0x401A
#define LOW_NOISE_T_FIRST 0x70DF
#define LOW_NOISE_P_FIRST 0x5059
#define ULN_T_FIRST 0x7866
#define ULN_P_FIRST 0x58E0

#define SOFT_RESET 0x805D
#define READ_ID 0xEFC8
#define MOVE_ADDRESS_PTR 0xC595
#define READ_OTP 0xC7F7

#define DEFAULT_I2C_ADDRESS 0x63
#define CHIP_ID 0x08

#define OK 0
#define CRC_FAIL 1

const int MEASUREMENT_DELAYS[4] = {
    7,   // NORMAL (5.6 to 6.3ms)
    2,   // LOW_POWER (1.6 to 1.8ms)
    24,  // LOW_NOISE (20.8 to 23.8ms)
    95   // ULTRA_LOW_NOISE (83.2 to 94.5ms)
};

int sensor_constants[4];

struct SensorConstants {
    float A;
    float B;
    float C;
};

struct measurementValues {
    float raw_pressure;
    float raw_temperature;
    float pressure;
    float temperature;
};
///GLOBAL BME280 VARIABLES///


///GLOBAL ENCODER VARIABLES///

int32_t encoder_position;
int32_t lastEncoderPosition;
bool isButtonPushed = LOW; 
int buttonState; 
int32_t lastButtonState = LOW;
unsigned long lastDebounceTime = 0; //last time button was toggled
unsigned long debounceDelay = 50; //debounce time (increase if debounce ocurs)
unsigned long previousMillis = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  initializeEncoder();

  if (! (bme.begin() )) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);

  if (!icm.begin_I2C()) {
  // if (!icm.begin_SPI(ICM_CS)) {
   // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {

  Serial.println("Failed to find ICM20948 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("ICM20948 Found!");

  Wire.begin();
  init_ICP10125();
  reset();
  delay(10);
}

void init_ICP10125() {
    int chip_id = get_chip_id();
    if (chip_id != CHIP_ID) {
        Serial.println("Invalid chip ID for ICP10125!");
    }
    read_otp();
}

void rdwr(int command, int length, int delayms, uint8_t* outData) {
    byte commandArray[2];
    commandArray[0] = (command >> 8) & 0xFF; // High byte
    commandArray[1] = command & 0xFF;        // Low byte

    Wire.beginTransmission(DEFAULT_I2C_ADDRESS);
    Wire.write(commandArray, 2);
    Wire.endTransmission();
    delay(delayms);

    if (length > 0) {
        Wire.requestFrom(DEFAULT_I2C_ADDRESS, length);
        int i = 0;
        while (Wire.available() && i < length) {
            outData[i++] = Wire.read();
        }
    }
}

int get_chip_id() {
    uint8_t result[3];
    rdwr(READ_ID, 3, 0, result);
    return result[0] & 0x3F;
}

void read_otp() {
    byte move_address_ptr[] = {
        (MOVE_ADDRESS_PTR >> 8) & 0xFF,
        MOVE_ADDRESS_PTR & 0xFF,
        0x00, 0x66, 0x9C // Address and CRC
    };

    Wire.beginTransmission(DEFAULT_I2C_ADDRESS);
    Wire.write(move_address_ptr, sizeof(move_address_ptr));
    Wire.endTransmission();

    for (int x = 0; x < 4; x++) {
        uint8_t result[3];
        rdwr(READ_OTP, 3, 0, result);
        if (crc8(result, 2) == result[2]) {
            sensor_constants[x] = (result[0] << 8) | result[1];
        } else {
            Serial.println("CRC failed reading OTP!");
            sensor_constants[x] = 0;
        }
    }
}

void reset() {
    uint8_t dummy[1];
    rdwr(SOFT_RESET, 0, 1, dummy);
}

struct measurementValues measure() {
    int delayms = MEASUREMENT_DELAYS[0];
    uint8_t result[9];
    rdwr(NORMAL, 9, delayms, result);

    measurementValues measurements = {0};

    if (crc8(&result[0], 2) != result[2] ||
        crc8(&result[3], 2) != result[5] ||
        crc8(&result[6], 2) != result[8]) {
        Serial.println("CRC failed on measurement data!");
        return measurements;
    }

    uint16_t T_raw = (result[0] << 8) | result[1];
    uint16_t P_raw = (result[3] << 8) | result[4];
    
    measurements.raw_pressure = P_raw;
    measurements.raw_temperature = T_raw;

    float temperature = -45.0 + (175.0 * T_raw / 65536.0);

    float LUT_lower = 3.5 * (1 << 20);
    float LUT_upper = 11.5;
    float quadr_factor = 1.0 / 16777216.0;
    float offst_factor = 2048.0;

    float t = T_raw - 32768;
    float s1 = LUT_lower + (sensor_constants[0] * t * t) * quadr_factor;
    float s2 = offst_factor * sensor_constants[3] + (sensor_constants[1] * t * t) * quadr_factor;
    float s3 = LUT_upper + (sensor_constants[2] * t * t) * quadr_factor;

    float sensorList[3] = {s1, s2, s3};
    SensorConstants constants = calculate_conversion_constants(sensorList);
    float pressure = constants.A + constants.B / (constants.C + P_raw);

    measurements.temperature = temperature;
    measurements.pressure = pressure;
    return measurements;
}

float calculate_altitude(float pressure) {
    return 44330.0 * (1.0 - pow((pressure / 1013.25), (1.0 / 5.255)));
}

struct SensorConstants calculate_conversion_constants(float *p_LUT) {
    float p_Pa[] = {45000.0, 80000.0, 105000.0};
    SensorConstants constants = {0};

    constants.C = (p_LUT[0] * p_LUT[1] * (p_Pa[0] - p_Pa[1]) +
                   p_LUT[1] * p_LUT[2] * (p_Pa[1] - p_Pa[2]) +
                   p_LUT[2] * p_LUT[0] * (p_Pa[2] - p_Pa[0])) /
                  (p_LUT[2] * (p_Pa[0] - p_Pa[1]) +
                   p_LUT[0] * (p_Pa[1] - p_Pa[2]) +
                   p_LUT[1] * (p_Pa[2] - p_Pa[0]));

    constants.A = (p_Pa[0] * p_LUT[0] - p_Pa[1] * p_LUT[1] -
                   (p_Pa[1] - p_Pa[0]) * constants.C) /
                  (p_LUT[0] - p_LUT[1]);

    constants.B = (p_Pa[0] - constants.A) * (p_LUT[0] + constants.C);

    return constants;
}

uint8_t crc8(uint8_t *data, int length) {
    uint8_t result = 0xFF;
    for (int i = 0; i < length; i++) {
        result ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (result & 0x80) {
                result = (result << 1) ^ 0x31;
            } else {
                result <<= 1;
            }
        }
    }
    return result;
}

void ICM20948data() {
  // icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  Serial.print("Accelerometer range set to: ");
  switch (icm.getAccelRange()) {
  case ICM20948_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case ICM20948_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case ICM20948_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case ICM20948_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  Serial.println("OK");

  // icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  Serial.print("Gyro range set to: ");
  switch (icm.getGyroRange()) {
  case ICM20948_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  }

  //  icm.setAccelRateDivisor(4095);
  uint16_t accel_divisor = icm.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);

  Serial.print("Accelerometer data rate divisor set to: ");
  Serial.println(accel_divisor);
  Serial.print("Accelerometer data rate (Hz) is approximately: ");
  Serial.println(accel_rate);

  //  icm.setGyroRateDivisor(255);
  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);

  Serial.print("Gyro data rate divisor set to: ");
  Serial.println(gyro_divisor);
  Serial.print("Gyro data rate (Hz) is approximately: ");
  Serial.println(gyro_rate);

  // icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);
  Serial.print("Magnetometer data rate set to: ");
  switch (icm.getMagDataRate()) {
  case AK09916_MAG_DATARATE_SHUTDOWN:
    Serial.println("Shutdown");
    break;
  case AK09916_MAG_DATARATE_SINGLE:
    Serial.println("Single/One shot");
    break;
  case AK09916_MAG_DATARATE_10_HZ:
    Serial.println("10 Hz");
    break;
  case AK09916_MAG_DATARATE_20_HZ:
    Serial.println("20 Hz");
    break;
  case AK09916_MAG_DATARATE_50_HZ:
    Serial.println("50 Hz");
    break;
  case AK09916_MAG_DATARATE_100_HZ:
    Serial.println("100 Hz");
    break;
  }
  Serial.println();

}

void initializeEncoder() {
  Serial.println("Looking for seesaw!");
  
  if (! ss.begin(SEESAW_ADDR)) {
    Serial.println("Couldn't find seesaw on default address");
    while(1) delay(10);
  }
  Serial.println("seesaw started");

  uint32_t version = ((ss.getVersion() >> 16) & 0xFFFF);
  if (version  != 4991){
    Serial.print("Wrong firmware loaded? ");
    Serial.println(version);
    while(1) delay(10);
  }
  Serial.println("Found Product 4991");
  
  // use a pin for the built in encoder switch
  ss.pinMode(SS_SWITCH, INPUT_PULLUP);

  // get starting position
  encoder_position = ss.getEncoderPosition();

  Serial.println("Turning on interrupts");
  delay(10);
  ss.setGPIOInterrupts((uint32_t)1 << SS_SWITCH, 1);
  ss.enableEncoderInterrupt();
}


void printBME280Values() {
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" °C");

    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
}

void printBNO055Values()
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");
}


void printICM20948Values() {
  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  Serial.print("\t\tTemperature ");
  Serial.print(temp.temperature);
  Serial.println(" deg C");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");

  Serial.print("\t\tMag X: ");
  Serial.print(mag.magnetic.x);
  Serial.print(" \tY: ");
  Serial.print(mag.magnetic.y);
  Serial.print(" \tZ: ");
  Serial.print(mag.magnetic.z);
  Serial.println(" uT");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println();

  //delay(100);

  //  Serial.print(temp.temperature);
  //
  //  Serial.print(",");
  //
  //  Serial.print(accel.acceleration.x);
  //  Serial.print(","); Serial.print(accel.acceleration.y);
  //  Serial.print(","); Serial.print(accel.acceleration.z);
  //
  //  Serial.print(",");
  //  Serial.print(gyro.gyro.x);
  //  Serial.print(","); Serial.print(gyro.gyro.y);
  //  Serial.print(","); Serial.print(gyro.gyro.z);
  //
  //  Serial.print(",");
  //  Serial.print(mag.magnetic.x);
  //  Serial.print(","); Serial.print(mag.magnetic.y);
  //  Serial.print(","); Serial.print(mag.magnetic.z);

  //  Serial.println();
  //
  //  delayMicroseconds(measurement_delay_us);
}

void printICP10125Values(){
measurementValues values = measure();
    Serial.print("Initial Pressure (raw): ");
    Serial.println(values.raw_pressure);
    
    Serial.print("Initial Temperature (raw): ");
    Serial.println(values.raw_temperature);

    Serial.print("Processed Pressure (hPa): ");
    Serial.println(values.pressure/100);
    Serial.print("Processed Temperature (°C): ");
    Serial.println(values.temperature);

    float altitude = calculate_altitude(values.pressure/100);
    Serial.print("Altitude (m): ");
    Serial.println(altitude);

    Serial.println("--------------------------");

}
/**
 * @brief Reads the encoder position and sets the neopixel color accordingly. 
 * @return Returns the position if changed from last position.
 */
int readEncoderMovement() {
  int32_t new_position = ss.getEncoderPosition();
  if (encoder_position != new_position) {
    Serial.println(new_position);  // display new position
    encoder_position = new_position;
  }
  return encoder_position;
}


/**
 * @brief Reads the button state and prints a statement if the button is pressed.
 * @return Returns the button reading if changed from last button reading.
 */
int readEncoderButtonPressed() {
  int32_t reading = ss.digitalRead(SS_SWITCH); // Read button state

  if (reading != lastButtonState) { // If the button state changed from the last reading
    lastDebounceTime = millis(); // reset debounce timer
  }

  if ((millis() - lastDebounceTime) > debounceDelay) { // If the button state has been stable for longer than debounceDelay
    if (reading != buttonState) { // If the button state has changed
      buttonState = reading;

      if (buttonState == LOW) { // If the new state is LOW (pressed)
        //isButtonPushed = !isButtonPushed;
        //Serial.println(isButtonPushed);
        Serial.println("Button pressed, state toggled!");
      }
    }
  }

  lastButtonState = reading; // Save reading for next loop
  return lastButtonState;
}
void loop() {

  encoder_position = readEncoderMovement();
  buttonState = readEncoderButtonPressed();

  // don't overwhelm serial port
  delay(10);

  switch (encoder_position) {
      case 1:
        printBME280Values();
        break;
      case 2:
        printBNO055Values();
        break;
      case 3:
        printICM20948Values();
        break;
      case 4: 
        printICP10125Values();
        break;
      default:
        Serial.println("Case default");
        // default is optional
        break;
    }
}