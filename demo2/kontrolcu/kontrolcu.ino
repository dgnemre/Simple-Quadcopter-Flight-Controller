#include<Servo.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

//-------------MPU-------------
#include <Wire.h>
#include <MPU9250_asukiaaa.h>

#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22
#endif

MPU9250 mySensor;
uint8_t sensorId;

//------------------------------

#define PI acos(-1) // 3.14.....

#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3

#define X           0     // X axis
#define Y           1     // Y axis
#define Z           2     // Z axis
#define SSF_GYRO    65.5  // Sensitivity Scale Factor of the gyro from datasheet
#define SSF_ACC     8.192 // Sensitivity Scale Factor of the accelerometer from datasheet

// ---------------- Controller variables ---------------------------------------
#define LEDC_CHANNEL_0     0
#define LEDC_CHANNEL_1     1
#define LEDC_CHANNEL_2     2
#define LEDC_CHANNEL_3     3
#define LEDC_TIMER_16_BIT  8
#define LEDC_BASE_FREQ     1000
#define ESC_BASE_FREQ      50
#define BAT_RED            13  //BATARYA
#define BAT_GREEN          32  //BATARYA
#define BLUE               26  //UCUS
#define GREEN              25  //UCUS

#define ESC_PIN_1              33
#define ESC_PIN_2              14
#define ESC_PIN_3              27
#define ESC_PIN_4              12

Servo ESC_1;
Servo ESC_2;
Servo ESC_3;
Servo ESC_4;

int brightness = 10;    // how bright the LED is
int fadeAmount = 1;    // how many points to fade the LED by
int pwm = 1000;
char Serialdata;
bool led_status = 0;
bool drone_status = 0;

void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
  // calculate duty, 8191 from 2 ^ 13 - 1
  uint32_t duty = (255 / valueMax) * min(value, valueMax);

  // write duty to LEDC
  ledcWrite(channel, duty);
}

uint32_t NUM = 5;

struct dataStruct{
  int16_t throttle;
  int16_t yaw;
  int8_t pitch;
  int8_t roll;
  int16_t pot;
  bool switch_l;
  bool switch_r;
  bool button_l;
  bool button_r;
  bool k_status;
}data;

RF24 radio(5,4);
const uint64_t pipe = 0xE8E8F0F0E1LL;
unsigned long timer = 0;

//------------------------------------------------------------------------

void setupMpu9250Registers();
void calibrateMpu9250();

// ----------------------- MPU variables -------------------------------------
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;

// ----------------------- Variables for servo signal generation -------------
unsigned long loop_timer;
unsigned long now, difference;

unsigned long pulse_length_esc1 = 1000,
        pulse_length_esc2 = 1000,
        pulse_length_esc3 = 1000,
        pulse_length_esc4 = 1000;

// ------------- Global variables used for PID automation --------------------
float errors[3];                     // Measured errors (compared to instructions) : [Yaw, Pitch, Roll]
float error_sum[3]      = {0, 0, 0}; // Error sums (used for integral component) : [Yaw, Pitch, Roll]
float previous_error[3] = {0, 0, 0}; // Last errors (used for derivative component) : [Yaw, Pitch, Roll]
float measures[3]       = {0, 0, 0}; // Angular measures : [Yaw, Pitch, Roll]
// ---------------------------------------------------------------------------

/**
 * Setup configuration
 */
void setup() {
  
  Serial.begin(115200); 
  setupMpu9250Registers();
  calibrateMpu9250();

  radio.begin();
  radio.setChannel(2);
  radio.setPayloadSize(13);
  radio.setDataRate(RF24_1MBPS);
  radio.openReadingPipe(1,pipe);
  radio.startListening();

  ESC_1.attach(ESC_PIN_1, 10, 1000, 2000);
  ESC_2.attach(ESC_PIN_2, 11, 1000, 2000);
  ESC_3.attach(ESC_PIN_3, 12, 1000, 2000);
  ESC_4.attach(ESC_PIN_4, 13, 1000, 2000);
  
  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_16_BIT);
  ledcAttachPin(BAT_RED, LEDC_CHANNEL_0);
  
  ledcSetup(LEDC_CHANNEL_1, LEDC_BASE_FREQ, LEDC_TIMER_16_BIT);
  ledcAttachPin(BAT_GREEN, LEDC_CHANNEL_1);
 
  ledcSetup(LEDC_CHANNEL_2, LEDC_BASE_FREQ, LEDC_TIMER_16_BIT);
  ledcAttachPin(BLUE, LEDC_CHANNEL_2);
  
  ledcSetup(LEDC_CHANNEL_3, LEDC_BASE_FREQ, LEDC_TIMER_16_BIT);
  ledcAttachPin(GREEN, LEDC_CHANNEL_3);
  
  // Initialize loop_timer.
//  loop_timer = micros();

//    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

  #ifdef _ESP32_HAL_I2C_H_ // For ESP32
    Wire.begin(SDA_PIN, SCL_PIN); // SDA, SCL
  #else
    Wire.begin();
  #endif
}

void loop() {
   
    if(led_status == 0){
    ledcAnalogWrite(LEDC_CHANNEL_0, 250-brightness);
    ledcAnalogWrite(LEDC_CHANNEL_1, brightness);
    ledcAnalogWrite(LEDC_CHANNEL_2, 250-map(pwm,1000,2000,250,5));
    ledcAnalogWrite(LEDC_CHANNEL_3, map(pwm,1000,2000,250,5));
  }else{
    ledcAnalogWrite(LEDC_CHANNEL_0, brightness);
    ledcAnalogWrite(LEDC_CHANNEL_1, 250-brightness);
    ledcAnalogWrite(LEDC_CHANNEL_2, 250-map(pwm,1000,2000,250,5));
    ledcAnalogWrite(LEDC_CHANNEL_3, map(pwm,1000,2000,250,5));
  }

  brightness = brightness + fadeAmount;
  
  // reverse the direction of the fading at the ends of the fade:100-65400
  if (brightness <= 0 || brightness >= 50) {
    fadeAmount = -fadeAmount;
  }
  
  if (radio.available()){ 
    radio.read(&data, sizeof(data)); 
    /*Serial.print(data.throttle);
    Serial.print(" ");
    Serial.print(data.yaw);
    Serial.print(" ");
    Serial.print(data.pitch);
    Serial.print(" ");
    Serial.print(data.roll);
    Serial.print(" "); 
    Serial.print(data.pot);
    Serial.print("\t");
    Serial.print(data.switch_l);
    Serial.print("\t");
    Serial.print(data.switch_r);
    Serial.print("\t");
    Serial.print(data.button_l);
    Serial.print("\t");
    Serial.print(data.button_r);
    Serial.print("\t");
    Serial.print(data.k_status);
    Serial.println(); */
    delay(2);  
    led_status = 1;
    if(data.k_status == 1)
      drone_status = 1;
    if(drone_status==data.k_status)
      pwm = data.throttle;
    timer = 0;
  }
  else{
    timer += 20-brightness/5;
    delay(20-brightness/5);
  }

  if(timer>500){
    led_status = 0; 
    ESC_1.writeMicroseconds(1000);
    ESC_2.writeMicroseconds(1000);
    ESC_3.writeMicroseconds(1000);
    ESC_4.writeMicroseconds(1000);
    data.throttle=1000;
    data.yaw=0;
    data.pitch=0;
    data.roll=0;
    data.pot=0;
  }
    
    
    // 1. First, read angular values from MPU-6050
    readSensor();
    convertRawValues();

    // 3. Calculate errors comparing received instruction with measures
    calculateErrors();

    // 4. Calculate motors speed with PID controller
    automation();

    // 5. Apply motors speed
    applyMotorSpeed();
}

/**
 * Generate servo-signal on digital pins #4 #5 #6 #7 with a frequency of 250Hz (4ms period).
 * Direct port manipulation is used for performances.
 * 
 * This function might not take more than 2ms to run, which lets 2ms remaining to do other stuff.
 * 
 * @see https://www.arduino.cc/en/Reference/PortManipulation
 * 
 * @return void
 */
void applyMotorSpeed() {

  Serial.print("\tSpeed:");
  Serial.print("\t1: ");
  Serial.print(pulse_length_esc1);
  Serial.print("\t2: ");
  Serial.print(pulse_length_esc2);
  Serial.print("\t3: ");
  Serial.print(pulse_length_esc3);
  Serial.print("\t4: ");
  Serial.println(pulse_length_esc4);

  ESC_1.writeMicroseconds(pulse_length_esc1);
  ESC_2.writeMicroseconds(pulse_length_esc2);
  ESC_3.writeMicroseconds(pulse_length_esc3);
  ESC_4.writeMicroseconds(pulse_length_esc4);
}

/**
 * Request raw values from MPU6050.
 * 
 * @return void
 */
void readSensor() {
  mySensor.accelUpdate();
  acc_x = mySensor.accelX();
  acc_y = mySensor.accelY();
  acc_z = mySensor.accelZ();
  Serial.print("\taccelX: " + String(acc_x));
  Serial.print("\taccelY: " + String(acc_y));
  Serial.print("\taccelZ: " + String(acc_z));

  mySensor.gyroUpdate();
  gyro_x = mySensor.gyroX();
  gyro_y = mySensor.gyroY();
  gyro_z = mySensor.gyroZ();
  Serial.print("\tgyroX: " + String(gyro_x));
  Serial.print("\tgyroY: " + String(gyro_y));
  Serial.print("\tgyroZ: " + String(gyro_z));
  
}

void convertRawValues() {
    gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
    gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
    gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value

    //Gyro angle calculations
    //0.0000611 = 1 / (250Hz / 65.5)
    angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
    angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable

    //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
    angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
    angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel

    //Accelerometer angle calculations
    acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
    //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
    angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
    angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle

    //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
    angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
    angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

    if (set_gyro_angles) {                                                 //If the IMU is already started
        angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
        angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
    } else {                                                                //At first start
        angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle
        angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle
        set_gyro_angles = true;                                            //Set the IMU started flag
    }

    //To dampen the pitch and roll angles a complementary filter is used
    measures[ROLL]  = measures[ROLL] * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
    measures[PITCH] = measures[PITCH] * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
    measures[YAW]   = gyro_z / SSF_GYRO;
}

void automation() {
    float Kp[3]       = {3, 5, 5}; // P coefficients in that order : Yaw, Pitch, Roll //ku = 0.21
    float Ki[3]       = {0.0, 0, 0};  // I coefficients in that order : Yaw, Pitch, Roll
    float Kd[3]       = {0, 0, 0};    // D coefficients in that order : Yaw, Pitch, Roll
    float deltaErr[3] = {0, 0, 0};    // Error deltas in that order :  Yaw, Pitch, Roll
    float yaw         = 0;
    float pitch       = 0;
    float roll        = 0;

    // Initialize motor commands with throttle
    pulse_length_esc1 = data.throttle;
    pulse_length_esc2 = data.throttle;
    pulse_length_esc3 = data.throttle;
    pulse_length_esc4 = data.throttle;

    // Do not calculate anything if throttle is 0
    if (data.throttle >= 1012) {
        // Calculate sum of errors : Integral coefficients
        error_sum[YAW] += errors[YAW];
        error_sum[PITCH] += errors[PITCH];
        error_sum[ROLL] += errors[ROLL];

        // Calculate error delta : Derivative coefficients
        deltaErr[YAW] = errors[YAW] - previous_error[YAW];
        deltaErr[PITCH] = errors[PITCH] - previous_error[PITCH];
        deltaErr[ROLL] = errors[ROLL] - previous_error[ROLL];

        // Save current error as previous_error for next time
        previous_error[YAW] = errors[YAW];
        previous_error[PITCH] = errors[PITCH];
        previous_error[ROLL] = errors[ROLL];

        yaw = (errors[YAW] * Kp[YAW]) + (error_sum[YAW] * Ki[YAW]) + (deltaErr[YAW] * Kd[YAW]);
        pitch = (errors[PITCH] * Kp[PITCH]) + (error_sum[PITCH] * Ki[PITCH]) + (deltaErr[PITCH] * Kd[PITCH]);
        roll = (errors[ROLL] * Kp[ROLL]) + (error_sum[ROLL] * Ki[ROLL]) + (deltaErr[ROLL] * Kd[ROLL]);

        // Yaw - Lacet (Z axis)
        pulse_length_esc1 -= yaw;
        pulse_length_esc4 -= yaw;
        pulse_length_esc3 += yaw;
        pulse_length_esc2 += yaw;

        // Pitch - Tangage (Y axis)
        pulse_length_esc1 += pitch;
        pulse_length_esc2 += pitch;
        pulse_length_esc3 -= pitch;
        pulse_length_esc4 -= pitch;

        // Roll - Roulis (X axis)
        pulse_length_esc1 -= roll;
        pulse_length_esc3 -= roll;
        pulse_length_esc2 += roll;
        pulse_length_esc4 += roll;
    }

    // Maximum value
    if (pulse_length_esc1 > 2000) pulse_length_esc1 = 2000;
    if (pulse_length_esc2 > 2000) pulse_length_esc2 = 2000;
    if (pulse_length_esc3 > 2000) pulse_length_esc3 = 2000;
    if (pulse_length_esc4 > 2000) pulse_length_esc4 = 2000;

    // Minimum value
    if (pulse_length_esc1 < 1000) pulse_length_esc1 = 1000;
    if (pulse_length_esc2 < 1000) pulse_length_esc2 = 1000;
    if (pulse_length_esc3 < 1000) pulse_length_esc3 = 1000;
    if (pulse_length_esc4 < 1000) pulse_length_esc4 = 1000;
}


/**
 * Calculate errors of Yaw, Pitch & Roll: this is simply the difference between the measure and the command.
 *
 * @return void
 */
void calculateErrors() {
    errors[YAW]   = measures[YAW]   - data.yaw;
    errors[PITCH] = measures[PITCH] - data.pitch;
    errors[ROLL]  = measures[ROLL]  - data.roll;
}

void setupMpu9250Registers() {
  
  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();
  sensorId = mySensor.readId();
  Serial.print("SendorId: ");
  Serial.println(sensorId);
}

void calibrateMpu9250()
{
    for (int cal_int = 0; cal_int < 2000; cal_int++) {                  //Run this code 2000 times
        readSensor();                                              //Read the raw acc and gyro data from the MPU-6050
        gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
        gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
        gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
        delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
    }
    gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
    gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
    gyro_z_cal /= 2000;

    Serial.println("Calibration Done");
}

