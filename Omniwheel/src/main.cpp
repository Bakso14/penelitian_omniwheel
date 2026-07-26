#include <Arduino.h>
#include <SPI.h>
#include <esp_now.h>
#include <WiFi.h>
#include "BNO055_support.h"		
#include <Wire.h>

//https://youtu.be/rUbmW4qAh8w?si=d8ImdF8glph8iRu_

//inverse kinematics
double matrix_kecepatan[9] = { -0.3333, 0.5774, 0.0317, -0.3333, -0.5774, 0.0317, 0.6667, 0, 0.0317 };
double V1, V2, V3, x_linier, y_linier, omega, Vmax, Speed_max;
int koordinat_x, koordinat_y, koordinat_theta = 0;


String inputString;

struct bno055_t myBNO;
struct bno055_euler myEulerData;

const float rpm_to_radians = 0.10471975512;
const float rad_to_deg = 57.29578;
const int pulsa_per_putaran = 1640;
const float jar_jari_roda = 2.9;
const float keliling_roda = 2 * PI * jar_jari_roda;
const float cm_per_pulsa = keliling_roda / pulsa_per_putaran;


int en1A = 4;
int en1B = 18;
int enc1A = 5;
int enc1B = 15;

int en2A = 13;
int en2B = 2;
int enc2A = 33;
int enc2B = 32;

int en3A = 27;
int en3B = 14;
int enc3A = 26;
int enc3B = 25;


volatile int encoder_value1 = 0; 
volatile int encoder_value2 = 0; 
volatile int encoder_value3 = 0; 

volatile int encoder_value1_jarak = 0; 
volatile int encoder_value2_jarak = 0; 
volatile int encoder_value3_jarak = 0; 

int waktu_sebelumnya = 0;
int waktu_display_sebelumnya = 0;
int waktu_pid_sebelumnya = 0;
int waktu_motor1_sebelumnya = 0;
int waktu_motor2_sebelumnya = 0;
int waktu_motor3_sebelumnya = 0;

// setting PWM properties
const int freq = 5000;
const int ledChannel = 0;
const int ledChannel1 = 1;
const int ledChannel2 = 2;
const int ledChannel3 = 3;
const int ledChannel4 = 4;
const int ledChannel5 = 5;
const int resolution = 8;
  
void encoder_isr1() {
  int A = digitalRead(enc1A);
  int B = digitalRead(enc1B);
  if ((A == HIGH) != (B == LOW)) {
    encoder_value1--;
    encoder_value1_jarak--;
  } else {
    encoder_value1++;
    encoder_value1_jarak++;
  }
}

void encoder_isr2() {
  int A = digitalRead(enc2A);
  int B = digitalRead(enc2B);
  if ((A == HIGH) != (B == LOW)) {
    encoder_value2--;
    encoder_value2_jarak--;
  } else {
    encoder_value2++;
    encoder_value2_jarak++; 
  }
}

void encoder_isr3() {
  int A = digitalRead(enc3A);
  int B = digitalRead(enc3B);
  if ((A == HIGH) != (B == LOW)) {
    encoder_value3--;
    encoder_value3_jarak--;
  } else {
    encoder_value3++;
    encoder_value3_jarak++;
  }
}

float kecepatan1, kecepatan2, kecepatan3 = 0;

//PID
const float Kp = 2.0;  // Proporsional
const float Ki = 0.1;  // Integral
const float Kd = 0.01; // Turunan

//Setpoint
float setpoint1 = 0.2;
float setpoint2 = 0.2;
float setpoint3 = 0.2;

// Variabel PID
float error1, lastError1, integral1, derivative1, output1;
float error2, lastError2, integral2, derivative2, output2;
float error3, lastError3, integral3, derivative3, output3;
float errorM1, lastErrorM1, integralM1, derivativeM1, outputM1;
float errorM2, lastErrorM2, integralM2, derivativeM2, outputM2;
float errorM3, lastErrorM3, integralM3, derivativeM3, outputM3;

float setpointM1,Kp1,Ki1,Kd1;
bool conditionM1 = 0;

float setpointM2,Kp2,Ki2,Kd2;
bool conditionM2 = 0;

float setpointM3,Kp3,Ki3,Kd3;
bool conditionM3 = 0;


void setPWM1() {
  error1 = (setpoint1 - kecepatan1)/100;
  integral1 += error1;
  derivative1 = error1 - lastError1;
  output1 = Kp * error1 + Ki * integral1 + Kd * derivative1;

  if((Ki*integral1) > 255){
    integral1 = 255/Ki;
  }

  if(Kd*derivative1 > 255){
    derivative1 = 255/Kd;
  }

  if (output1 > 255) {
    output1 = 255;
  } else if (output1 < 0) {
    output1 = 0;
  }

  // ledcWrite(ledChannel, output1);
  lastError1 = error1;
}

void setPWM2() {
  error2 = (setpoint2 - kecepatan2)/100;
  integral2 += error2;
  derivative2 = error2 - lastError2;
  output2 = Kp * error2 + Ki * integral2 + Kd * derivative2;

  if((Ki*integral2) > 255){
    integral2 = 255/Ki;
  }

  if(Kd*derivative2 > 255){
    derivative2 = 255/Kd;
  }

  if (output2 > 255) {
    output2 = 255;
  } else if (output2 < 0) {
    output2 = 0;
  }

  // ledcWrite(ledChannel1, output2);
  lastError2 = error2;
}

void setPWM3() {
  error3 = (setpoint3 - kecepatan3)/100;
  integral3 += error3;
  derivative3 = error3 - lastError3;
  output3 = Kp * error3 + Ki * integral3 + Kd * derivative3;

  if((Ki*integral3) > 255){
    integral1 = 255/Ki;
  }

  if(Kd*derivative3 > 255){
    derivative1 = 255/Kd;
  }

  if (output3 > 255) {
    output3 = 255;
  } else if (output3 < 0) {
    output3 = 0;
  }

  // ledcWrite(ledChannel2, output3);
  lastError3 = error3;
}


int condition1 = 0;
int condition2 = 0;
int condition3 = 0; 
float speed1 = 0; 
float speed2 = 0;
float speed3 = 0;
int timer_motor1 = 0;
int timer_motor2 = 0;
int timer_motor3 = 0;
long jarak_motor1 = 0;
long jarak_motor2 = 0;
long jarak_motor3 = 0;
bool flag_timer_motor1=0;
bool flag_timer_motor2=0;
bool flag_timer_motor3=0;
int flag_kecepatan = 1;


void setMotor1() {
  if(flag_timer_motor1 == 1){
    unsigned long waktu_init_motor1 = millis();
    if(waktu_init_motor1 - waktu_motor1_sebelumnya <= timer_motor1){
      setpoint1 = speed1;
      setPWM1();
    }else{
      speed1 = 0;
      flag_timer_motor1 = 0;
    }
  }else if(flag_timer_motor1 == 0){
    waktu_motor1_sebelumnya = millis();
    setpoint1 = speed1;
    setPWM1();
  }

  if(speed1 == 0){
    ledcWrite(ledChannel, 0);
    ledcWrite(ledChannel1, 0); 
  }else{
    if(condition1 == 1){
      ledcWrite(ledChannel, 0);
      ledcWrite(ledChannel1, output1);
  
    } else if(condition1 == 0) {
      ledcWrite(ledChannel, output1);
      ledcWrite(ledChannel1, 0);   
    }  
  }
    
}

void setMotor2() {
  if(flag_timer_motor2 == 1){
    unsigned long waktu_init_motor2 = millis();
    if(waktu_init_motor2 - waktu_motor2_sebelumnya <= timer_motor2){
      setpoint2 = speed2;
      setPWM2();
    }else{
      speed2 = 0;
      flag_timer_motor2 = 0;
    }
  }else if(flag_timer_motor2 == 0){
    waktu_motor2_sebelumnya = millis();
    setpoint2 = speed2;
    setPWM2();
  }

  if(speed2 == 0){
    ledcWrite(ledChannel2, 0);
    ledcWrite(ledChannel3, 0); 
  }else{
    if(condition2 == 1){
      ledcWrite(ledChannel2, 0);
      ledcWrite(ledChannel3, output2);
  
    } else if(condition2 == 0) {
      ledcWrite(ledChannel2, output2);
      ledcWrite(ledChannel3, 0);   
    }
  }

}

void setMotor3() {
  if(flag_timer_motor3 == 1){
    unsigned long waktu_init_motor3 = millis();
    if(waktu_init_motor3 - waktu_motor3_sebelumnya <= timer_motor3){
      setpoint3 = speed3;
      setPWM3();
    }else{
      speed3 = 0;
      flag_timer_motor3 = 0;
    }
  }else if(flag_timer_motor3 == 0){
    waktu_motor3_sebelumnya = millis();
    setpoint3 = speed3;
    setPWM3();
  }

  if(speed3 == 0){
    ledcWrite(ledChannel4, 0);
    ledcWrite(ledChannel5, 0); 
  }else{
    if(condition3 == 1){
      ledcWrite(ledChannel4, 0);
      ledcWrite(ledChannel5, output3);
  
    } else if(condition3 == 0) {
      ledcWrite(ledChannel4, output3);
      ledcWrite(ledChannel5, 0);   
    }
  }
 
}



void Split(char* e) {
  int jumlah_data = 7;
  char* v[jumlah_data];
  char *p;
  int i = 0;
  p = strtok(e, ",");
  while (p && i < jumlah_data) {
    v[i] = p;
    p = strtok(NULL, ",");
    i++;
  };

  if(atoi(v[0]) == 3){
    condition1 = atoi(v[1]);
    condition2 = atoi(v[2]);
    condition3 = atoi(v[3]);

    V1 = atof(v[4]);
    V2 = atof(v[5]);
    V3 = atof(v[6]);
    
  }

}


void setup() {

  pinMode(enc1A, INPUT_PULLUP);
  pinMode(enc1B, INPUT_PULLUP);
  pinMode(enc2A, INPUT_PULLUP);
  pinMode(enc2B, INPUT_PULLUP);
  pinMode(enc3A, INPUT_PULLUP);
  pinMode(enc3B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(enc1A), encoder_isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2A), encoder_isr2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc3A), encoder_isr3, CHANGE);
  Serial.begin(115200);

  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(en1A, ledChannel);

  ledcSetup(ledChannel1, freq, resolution);
  ledcAttachPin(en1B, ledChannel1);

  ledcSetup(ledChannel2, freq, resolution);
  ledcAttachPin(en2A, ledChannel2);

  ledcSetup(ledChannel3, freq, resolution);
  ledcAttachPin(en2B, ledChannel3);

  ledcSetup(ledChannel4, freq, resolution);
  ledcAttachPin(en3A, ledChannel4);

  ledcSetup(ledChannel5, freq, resolution);
  ledcAttachPin(en3B, ledChannel5);

  ledcWrite(ledChannel, 0);
  ledcWrite(ledChannel1, 0);

  ledcWrite(ledChannel2, 0);
  ledcWrite(ledChannel3, 0);

  ledcWrite(ledChannel4, 0);
  ledcWrite(ledChannel5, 0);


}


void loop() {
  
  unsigned long waktu_sekarang = millis();
  if(waktu_sekarang - waktu_sebelumnya >= 50){
    waktu_sebelumnya = waktu_sekarang;

    kecepatan1 = (float)(((abs(encoder_value1)*1200) / pulsa_per_putaran)*rpm_to_radians*jar_jari_roda);
    encoder_value1 = 0;

    kecepatan2 = (float)(((abs(encoder_value2)*1200) / pulsa_per_putaran)*rpm_to_radians*jar_jari_roda);
    encoder_value2 = 0;

    kecepatan3 = (float)(((abs(encoder_value3)*1200) / pulsa_per_putaran)*rpm_to_radians*jar_jari_roda);
    encoder_value3 = 0;

    x_linier = matrix_kecepatan[0] * kecepatan1 + matrix_kecepatan[1] * kecepatan2 + matrix_kecepatan[2] * kecepatan3;
    y_linier = matrix_kecepatan[3] * kecepatan1 + matrix_kecepatan[4] * kecepatan2 + matrix_kecepatan[5] * kecepatan3;
    omega = matrix_kecepatan[6] * kecepatan1 + matrix_kecepatan[7] * kecepatan2 + matrix_kecepatan[8] * kecepatan3;

  }
  
  koordinat_x     = matrix_kecepatan[0] * encoder_value1_jarak*cm_per_pulsa + matrix_kecepatan[1] * encoder_value2_jarak*cm_per_pulsa + matrix_kecepatan[2] * encoder_value3_jarak*cm_per_pulsa;
  koordinat_y     = matrix_kecepatan[3] * encoder_value1_jarak*cm_per_pulsa + matrix_kecepatan[4] * encoder_value2_jarak*cm_per_pulsa + matrix_kecepatan[5] * encoder_value3_jarak*cm_per_pulsa;
  koordinat_theta = matrix_kecepatan[6] * encoder_value1_jarak*cm_per_pulsa + matrix_kecepatan[7] * encoder_value2_jarak*cm_per_pulsa + matrix_kecepatan[8] * encoder_value3_jarak*cm_per_pulsa;


  unsigned long waktu_display = millis();
  if(waktu_display - waktu_display_sebelumnya >= 200){
    waktu_display_sebelumnya = waktu_display;

    Serial.print(koordinat_x);
    Serial.print(",");
    Serial.print(koordinat_y);
    Serial.print(",");
    Serial.print(koordinat_theta);
    Serial.print(",");
    Serial.print(x_linier);
    Serial.print(",");
    Serial.print(y_linier);
    Serial.print(",");
    Serial.println(omega);
    
    
  }

  if(Serial.available() >0){
    flag_kecepatan = 3;
    inputString = Serial.readStringUntil('\n'); 
    char inputCharArray[inputString.length() + 1]; 
    inputString.toCharArray(inputCharArray, inputString.length() + 1); 
    Split(inputCharArray);

    Vmax = 25;

    speed1 = abs(V1);
    speed2 = abs(V2);
    speed3 = abs(V3);

    Speed_max = max(speed1, max(speed2, speed3));
    if(Speed_max > 0){
      speed1 = (speed1 / Speed_max) * Vmax;
      speed2 = (speed2 / Speed_max) * Vmax;
      speed3 = (speed3 / Speed_max) * Vmax;

    }else{
      speed1 = 0;
      speed2 = 0;
      speed3 = 0;
    }
    
    flag_timer_motor1 = 0;
    flag_timer_motor2 = 0;
    flag_timer_motor3 = 0;

  }

  if (flag_kecepatan == 3){
    setMotor1();
    setMotor2();
    setMotor3();
    
  }
  
} 