#include<Servo.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0     0
#define LEDC_CHANNEL_1     1
#define LEDC_CHANNEL_2     2
#define LEDC_CHANNEL_3     3

// use 8 bit precission for LEDC timer
#define LEDC_TIMER_8_BIT  8

// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ     1000
#define ESC_BASE_FREQ      50

// fade LED PIN (replace with LED_BUILTIN constant for built-in LED)
#define BAT_RED            13  //BATARYA
#define BAT_GREEN          32  //BATARYA
#define BLUE               26  //UCUS
#define GREEN              25  //UCUS

#define ESC_PIN_1              12
#define ESC_PIN_2              14
#define ESC_PIN_3              27
#define ESC_PIN_4              33

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

// Arduino like analogWrite
// value has to be between 0 and valueMax
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

void setup() {

  Serial.begin(115200);
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
  
  // Setup timer and attach timer to a led pin

  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_8_BIT);
  ledcAttachPin(BAT_RED, LEDC_CHANNEL_0);
  
  ledcSetup(LEDC_CHANNEL_1, LEDC_BASE_FREQ, LEDC_TIMER_8_BIT);
  ledcAttachPin(BAT_GREEN, LEDC_CHANNEL_1);
 
  ledcSetup(LEDC_CHANNEL_2, LEDC_BASE_FREQ, LEDC_TIMER_8_BIT);
  ledcAttachPin(BLUE, LEDC_CHANNEL_2);
  
  ledcSetup(LEDC_CHANNEL_3, LEDC_BASE_FREQ, LEDC_TIMER_8_BIT);
  ledcAttachPin(GREEN, LEDC_CHANNEL_3);
  
}

void loop() {
  
  ESC_1.writeMicroseconds(pwm);
  ESC_2.writeMicroseconds(pwm);
  ESC_3.writeMicroseconds(pwm);
  ESC_4.writeMicroseconds(pwm);

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
    Serial.print(data.throttle);
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
    Serial.println();     
    delay(10);
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
  //Serial.println("No radio available");
  }

  if(timer>500){
    led_status = 0;
   /* if(drone_status==1)
      pwm = 1500;
   */   
  }
}

