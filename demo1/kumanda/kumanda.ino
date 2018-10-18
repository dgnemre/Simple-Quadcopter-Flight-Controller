#include  <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <String.h>
#include <Wire.h>
#include <math.h>


const int MPU=0x68;
int16_t AcX,AcY,AcZ,Tmp;
int hizsiniri = 1950, hassasiyet = 2;
int8_t PITCH, ROLL;
int16_t THROTTLE, YAW; 
long THROTTLE_CAL=0, YAW_CAL=0, PITCH_CAL=0, ROLL_CAL=0;
int16_t HOWER;
void writeRawData();

void IMU(){
  Wire.beginTransmission(MPU); /* I2C haberleşmesi yapılacak kart seçildi */
  Wire.write(0x3B); /* 0x3B adresindeki register'a ulaşıldı */
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);/* 14 BYTE'lık veri istendi */ 
  AcX=Wire.read()<<8|Wire.read(); AcY=Wire.read()<<8|Wire.read(); AcZ=Wire.read()<<8|Wire.read(); // Tmp=Wire.read()<<8|Wire.read(); 
  AcX = map(AcX,0,16000,0,250); AcY = map(AcY,0,16000,0,250); AcZ = map(AcZ,0,16000,0,250); 
  /* 
  * Sırası ile okunan her iki byte birleştirilerek değişkenlere yazdırıldı
  * Böylece IMU'dan tüm değerler okunmuş oldu
  * 0X3B adresi imu değerlerinden ilk sensörün değerine denk gelmektedir.
  * IMU'dan tüm değerlerin okunabilmesi için bu adresten başlandı */
}  


char msg[6] = "hello";
RF24 radio(8, 7);
const uint64_t pipe = 0xE8E8F0F0E1LL;
int THRESHOLD = 1500;
bool condition = 0;
bool flag = 0;

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

void setup(void) {

  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0); /* MPU-6050 çalıştırıldı */
  Wire.endTransmission(true); /* I2C haberleşmesi başlatıldı ve MPU-6050'nin ilk ayarları yapıldı */
  
  delay(100);
  Serial.begin(115200);

  pinMode(A0, INPUT); /* sol joy buton */ pinMode(A1, INPUT); /* sol joy x eksen */ pinMode(A2, INPUT); /* sol joy y yükseklik */
  pinMode(A3, INPUT); /* sağ joy buton */ pinMode(A6, INPUT); /* sağ joy x sağ-sol */ pinMode(A7, INPUT); /* sağ joy y ileri-geri */
  pinMode(A4, INPUT); /* imu SDA pin4 */ pinMode(A5, INPUT); /* imu SCA pin5 */ pinMode(2 , INPUT); /* switch sağ */ 
  pinMode(3 , INPUT); /* switch sol */ pinMode(4 , INPUT);
  
  for(int i=0;i<200;i++){
    THROTTLE_CAL   += analogRead(A2); 
    YAW_CAL        += analogRead(A1); 
    PITCH_CAL      += analogRead(A7); 
    ROLL_CAL       += analogRead(A6); 
    }
  
  THROTTLE_CAL   = THROTTLE_CAL/200; 
  YAW_CAL        = YAW_CAL/200; 
  PITCH_CAL      = PITCH_CAL/200; 
  ROLL_CAL       = ROLL_CAL/200; 
  
  Serial.println("Kalibrasyon tamamlandı, kumanda Hazir.");

  while( digitalRead(3)!=0 || digitalRead(2)!=0 || digitalRead(A0)!=1 || digitalRead(4)!=1 ){
    Serial.println("Kumandayi acmak icin sag switch 0 konumunda iken joystick butonlarina basiniz!");
    }
  
  radio.begin();
  radio.setChannel(2);
  radio.setPayloadSize(13);
  radio.setDataRate(RF24_1MBPS);
  radio.openWritingPipe(pipe);
  
}

void loop(void) {
  
  if(analogRead(A2)-THROTTLE_CAL>2) THROTTLE   =  map(analogRead(A2),THROTTLE_CAL,670,1500,2000); // yukseklik
  else                              THROTTLE   =  map(analogRead(A2),100,THROTTLE_CAL,100,1500);  // yukseklik

  if(analogRead(A1)-YAW_CAL>2)      YAW        = -map(analogRead(A1),YAW_CAL,620,0,180);          // eksen
  else                              YAW        = -map(analogRead(A1),60,YAW_CAL,-180,0);          // eksen

  if(analogRead(A7)-PITCH_CAL>2)    PITCH      =  map(analogRead(A7),PITCH_CAL,680,0,30);         // ileri-geri
  else                              PITCH      =  map(analogRead(A7),150,PITCH_CAL,-30,0);        // ileri-geri

  if(analogRead(A6)-ROLL_CAL>2)     ROLL       = -map(analogRead(A6),ROLL_CAL,680,0,30);          // sag-sol
  else                              ROLL       = -map(analogRead(A6),150,ROLL_CAL,-30,0);         // sag-sol
  
  if( (condition == 1) && (flag == 1))
    THROTTLE = THROTTLE;
  else if( (condition ==1) && (flag == 0) && (THROTTLE > THRESHOLD) )
    THROTTLE = THRESHOLD;
  else
    THROTTLE = THROTTLE-495;
  
  if( (condition == 0) && (THROTTLE > THRESHOLD) ){
    condition = 1;
  }

  if( (condition == 1) && (THROTTLE < 1050) ){
    flag = 1;
    data.k_status = flag;
  }
  
  if(THROTTLE<1000) THROTTLE  = 1000;
  if(THROTTLE>2000) THROTTLE  = 2000;
  if(YAW<-180)      YAW       = -180;
  if(YAW>180)       YAW       = 180;
  if(PITCH<-30)     PITCH     = -30;
  if(PITCH>30)      PITCH     = 30;
  if(ROLL<-30)      ROLL      = -30;
  if(ROLL>30)       ROLL      = 30;
  
  if(abs(YAW)<2)    YAW       = 0;
  if(abs(PITCH)<2)  PITCH     = 0;
  if(abs(ROLL)<2)   ROLL      = 0;

  //writeRawData();
  
  if(digitalRead(2)==0){ 
    HOWER = THROTTLE;
    data.throttle = THROTTLE; 
  }else{
    data.throttle = HOWER;
    //IMU(); // Serial.print(AcX); Serial.print(" "); Serial.print(AcY); Serial.print(" "); Serial.println(AcZ);  
  }
 
  data.yaw = YAW;
  data.pitch = PITCH;
  data.roll = ROLL;
  data.pot = analogRead(A3);
  data.switch_l = digitalRead(3);
  data.switch_r = digitalRead(2);
  data.button_l = digitalRead(A0);
  data.button_r = digitalRead(4);
  Serial.println("send ...");
  radio.write(&data, sizeof(data));
  delay(10);

  Serial.print(data.throttle);
  Serial.print("\t");
  Serial.print(data.yaw);
  Serial.print("\t");
  Serial.print(data.pitch);
  Serial.print("\t");
  Serial.print(data.roll);
  Serial.print("\t");
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

  
}
