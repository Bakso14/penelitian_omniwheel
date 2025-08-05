#include <Arduino.h>
#include <SPI.h>
#include <esp_now.h>
#include <WiFi.h>
#include "BNO055_support.h"		
#include <Wire.h>

//https://youtu.be/rUbmW4qAh8w?si=d8ImdF8glph8iRu_

//inverse kinematics
double matrix_kecepatan[9] = { -0.3333, 0.5774, 0.0317, -0.3333, -0.5774, 0.0317, 0.6667, 0, 0.0317 };
double V1, V2, V3;

String inputString;
double linear_x = 0;
double linear_y = 0;
double linear_z = 0;

double angular_x = 0;
double angular_y = 0;
double angular_z = 0;

struct bno055_t myBNO;
struct bno055_euler myEulerData;

unsigned long lastTime = 0;

const float rpm_to_radians = 0.10471975512;
const float rad_to_deg = 57.29578;
const int pulsa_per_putaran = 1640;
const float jar_jari_roda = 2.9;
const float keliling_roda = 2 * PI * jar_jari_roda;
const float cm_per_pulsa = keliling_roda / pulsa_per_putaran;

bool firstloop = true;
const int BUFFER_SIZE = 50;
char serialBuffer[BUFFER_SIZE];

// int en1A = 18;
// int en1B = 4;
// int enc1A = 34;
// int enc1B = 35;

// int en2A = 2;
// int en2B = 13;
// int enc2A = 32;
// int enc2B = 33;

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

void PIDM1() {
  errorM1 = (setpointM1 - kecepatan1);
  integralM1 += errorM1;
  derivativeM1 = errorM1 - lastErrorM1;
  outputM1 = Kp1 * errorM1 + Ki1 * integralM1 + Kd1 * derivativeM1;

  if((Ki1*integralM1) > 255){
    integralM1 = 255/Ki1;
  }

  if(Kd1*derivativeM1 > 255){
    derivativeM1 = 255/Kd1;
  }

  if (outputM1 > 255) {
    outputM1 = 255;
  } else if (outputM1 < 0) {
    outputM1 = 0;
  }

  if(conditionM1 == 1){
    ledcWrite(ledChannel, 0);
    ledcWrite(ledChannel1, outputM1);

  } else if(conditionM1 == 0) {
    ledcWrite(ledChannel, outputM1);
    ledcWrite(ledChannel1, 0);   
  }

  if(setpointM1 == 0){
    ledcWrite(ledChannel, 0);
    ledcWrite(ledChannel1, 0); 
  }

  lastErrorM1 = errorM1;
}

void PIDM2() {
  errorM2 = (setpointM2 - kecepatan2);
  integralM2 += errorM2;
  derivativeM2 = errorM2 - lastErrorM2;
  outputM2 = Kp2 * errorM2 + Ki2 * integralM2 + Kd2 * derivativeM2;

  if((Ki2*integralM2) > 255){
    integralM2 = 255/Ki2;
  }

  if(Kd2*derivativeM2 > 255){
    derivativeM2 = 255/Kd2;
  }

  if (outputM2 > 255) {
    outputM2 = 255;
  } else if (outputM2 < 0) {
    outputM2 = 0;
  }

  if(conditionM2 == 1){
    ledcWrite(ledChannel2, 0);
    ledcWrite(ledChannel3, outputM2);

  } else if(conditionM2 == 0) {
    ledcWrite(ledChannel2, outputM2);
    ledcWrite(ledChannel3, 0);   
  }

  if(setpointM2 == 0){
    ledcWrite(ledChannel2, 0);
    ledcWrite(ledChannel3, 0); 
  }
  lastErrorM2 = errorM2;
}

void PIDM3() {
  errorM3 = (setpointM3 - kecepatan3);
  integralM3 += errorM3;
  derivativeM3 = errorM3 - lastErrorM3;
  outputM3 = Kp3 * errorM3 + Ki3 * integralM3 + Kd3 * derivativeM3;

  if((Ki3*integralM3) > 255){
    integralM3 = 255/Ki3;
  }

  if(Kd3*derivativeM3 > 255){
    derivativeM3 = 255/Kd3;
  }

  if (outputM3 > 255) {
    outputM3 = 255;
  } else if (outputM3 < 0) {
    outputM3 = 0;
  }

  if(conditionM3 == 1){
    ledcWrite(ledChannel4, 0);
    ledcWrite(ledChannel5, outputM3);

  } else if(conditionM3 == 0) {
    ledcWrite(ledChannel4, outputM3);
    ledcWrite(ledChannel5, 0);   
  }

  if(setpointM3 == 0){
    ledcWrite(ledChannel4, 0);
    ledcWrite(ledChannel5, 0); 
  }
  
  lastErrorM3 = errorM3;
}

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

typedef struct struct_message {
    int function_code;
    int motor_code;
    float sp;
    int dir;
    float timer;
} struct_message;

struct_message myData;

typedef struct struct_message_pid {
    int function_code; 
    int motor_code;         
    float sp; 
    float dir; 
    float kp; 
    float ki;
    float kd;
} struct_message_pid;

struct_message_pid myDataPID;

typedef struct motor {
    int function_code;
    int dir1;
    int dir2;
    int dir3;
    float speed1;
    float speed2;
    float speed3;
} motor;

motor motor_keseluruhan;

motor command;

typedef struct motor_sinkron {
    int function_code;
    int dir1;
    int dir2;
    int dir3;
    float speed1;
    float speed2;
    float speed3;
    long timer1;
    long timer2;
    long timer3;
} motor_sinkron;

motor_sinkron sinkron_motor;


struct_message_pid dummy;

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

//Transmitter
uint8_t broadcastAddress[] = {0x08, 0xB6, 0x1F, 0x71, 0xBB, 0xEC};
//08:B6:1F:71:BB:EC
typedef struct data_kecepatan {
  float v1;
  float v2;
  float v3;
  float theta;
  int S1;
  int S2;
  int S3;
}data_kecepatan;

data_kecepatan current_velocity;
esp_now_peer_info_t peerInfo;
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
//end


unsigned long startTime;
bool timerRunning = false;
void startTimer() {
  startTime = millis(); 
  timerRunning = true;
}

void setMotor(int condition1, int condition2, int condition3, float speed1, float speed2, float speed3) {
  setpoint1 = speed1;
  setpoint2 = speed2;
  setpoint3 = speed3;
  setPWM1();
  setPWM2();
  setPWM3();
  
  if(condition1 == 1){
    ledcWrite(ledChannel, 0);
    ledcWrite(ledChannel1, output1);

  } else if(condition1 == 0) {
    ledcWrite(ledChannel, output1);
    ledcWrite(ledChannel1, 0);   
  }

  if(condition2 == 1){
    ledcWrite(ledChannel2, 0);
    ledcWrite(ledChannel3, output2);

  } else if(condition2 == 0) {
    ledcWrite(ledChannel2, output2);
    ledcWrite(ledChannel3, 0);   
  }

  if(condition3 == 1){
    ledcWrite(ledChannel4, 0);
    ledcWrite(ledChannel5, output3);

  } else if(condition3 == 0) {
    ledcWrite(ledChannel4, output3);
    ledcWrite(ledChannel5, 0);   
  }

  if(speed1 == 0){
    ledcWrite(ledChannel, 0);
    ledcWrite(ledChannel1, 0); 
  }

  if(speed2 == 0){
    ledcWrite(ledChannel2, 0);
    ledcWrite(ledChannel3, 0); 
  }

  if(speed3 == 0){
    ledcWrite(ledChannel4, 0);
    ledcWrite(ledChannel5, 0); 
  }

    
}

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

void setMotor1_jarak() {
  // Serial.print("MOTOR1");
  if(abs(encoder_value1_jarak) < jarak_motor1){
    setpoint1 = speed1;
    setPWM1();
    // Serial.print("\t");
    // Serial.print(output1);
  }else{
    speed1 = 0;
    setpoint1 = 0;
    setPWM1();
    // Serial.print("\t");
    // Serial.print("Stop Motor 1");
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

void setMotor2_jarak() {
  // Serial.print("\t");
  // Serial.print("MOTOR2");
  if(abs(encoder_value2_jarak) < jarak_motor2){
    setpoint2 = speed2;
    setPWM2();
    // Serial.print("\t");
    // Serial.print(output2);
  }else{
    speed2 = 0;
    setpoint2 = 0;
    setPWM2();
    // Serial.print("\t");
    // Serial.print("Stop Motor 2");
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
void setMotor3_jarak() {
  // Serial.print("\t");
  // Serial.print("MOTOR3");
  if(abs(encoder_value3_jarak) < jarak_motor3){
    setpoint3 = speed3;
    setPWM3();
    // Serial.print("\t");
    // Serial.println(output3);
  }else{
    speed3 = 0;
    setpoint3 = 0;
    setPWM3();
    // Serial.print("\t");
    // Serial.println("Stop Motor 3");
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

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&dummy, incomingData, sizeof(dummy));

  switch (dummy.function_code)
  {
  case 0:
    memcpy(&myDataPID, incomingData, sizeof(myDataPID));
    flag_kecepatan = 0;  
    if(dummy.motor_code == 94){
      Kp1 =0;
      Kp2 =0;
      Kp3 =0;
      Ki1 =0;
      Ki2 =0;
      Ki3 =0;
      Kd1 =0;
      Kd2 =0;
      Kd3 =0;
      setpointM1 = myDataPID.sp;
      conditionM1 = myDataPID.dir;
      Kp1 = myDataPID.kp;
      Ki1 = myDataPID.ki;
      Kd1 = myDataPID.kd;

    }else if(dummy.motor_code == 7){
      Kp1 =0;
      Kp2 =0;
      Kp3 =0;
      Ki1 =0;
      Ki2 =0;
      Ki3 =0;
      Kd1 =0;
      Kd2 =0;
      Kd3 =0;
      setpointM2 = myDataPID.sp;
      conditionM2 = myDataPID.dir;
      Kp2 = myDataPID.kp;
      Ki2 = myDataPID.ki;
      Kd2 = myDataPID.kd;

    }else if(dummy.motor_code == 10){
      Kp1 =0;
      Kp2 =0;
      Kp3 =0;
      Ki1 =0;
      Ki2 =0;
      Ki3 =0;
      Kd1 =0;
      Kd2 =0;
      Kd3 =0;
      setpointM3 = myDataPID.sp;
      conditionM3 = myDataPID.dir;
      Kp3 = myDataPID.kp;
      Ki3 = myDataPID.ki;
      Kd3 = myDataPID.kd;
    }
    break;
  
  case 1:
    memcpy(&myData, incomingData, sizeof(myData));
    flag_kecepatan = 1;
    if(dummy.motor_code == 94){
      condition1 = myData.dir;
      speed1 = myData.sp;
      timer_motor1 = myData.timer;
      if(timer_motor1 > 0){
        flag_timer_motor1 = 1;
      }else{
        flag_timer_motor1 = 0;
      }
    }else if(dummy.motor_code == 7){
      condition2 = myData.dir;
      speed2 = myData.sp; 
      timer_motor2 = myData.timer;
      if(timer_motor2 > 0){
        flag_timer_motor2 = 1;
      }else{
        flag_timer_motor2 = 0;
      }
    }else if(dummy.motor_code == 10){
      condition3 = myData.dir;
      speed3 = myData.sp;
      timer_motor3 = myData.timer;
      if(timer_motor3 > 0){
        flag_timer_motor3 = 1;
      }else{
        flag_timer_motor3 = 0;
      }
    }
    break;
  
  case 2:
    memcpy(&motor_keseluruhan, incomingData, sizeof(motor_keseluruhan));
    flag_kecepatan = 1;
    condition1 = motor_keseluruhan.dir1;
    condition2 = motor_keseluruhan.dir2;
    condition3 = motor_keseluruhan.dir3;
    speed1 = motor_keseluruhan.speed1;
    speed2 = motor_keseluruhan.speed2;
    speed3 = motor_keseluruhan.speed3;
    break;
  
  case 3:
    memcpy(&sinkron_motor, incomingData, sizeof(sinkron_motor));
    flag_kecepatan = 1;
    
    condition1 = sinkron_motor.dir1;
    speed1 = sinkron_motor.speed1;
    timer_motor1 = sinkron_motor.timer1;
    if(timer_motor1 > 0){
      flag_timer_motor1 = 1;
    }else{
      flag_timer_motor1 = 0;
    }

    condition2 = sinkron_motor.dir2;
    speed2 = sinkron_motor.speed2;
    timer_motor2 = sinkron_motor.timer2;
    if(timer_motor2 > 0){
      flag_timer_motor2 = 1;
    }else{
      flag_timer_motor2 = 0;
    }

    condition3 = sinkron_motor.dir3;
    speed3 = sinkron_motor.speed3;
    timer_motor3 = sinkron_motor.timer3;
    if(timer_motor3 > 0){
      flag_timer_motor3 = 1;
    }else{
      flag_timer_motor3 = 0;
    }
    break;
  
  case 4:
    memcpy(&sinkron_motor, incomingData, sizeof(sinkron_motor));
    flag_kecepatan = 2;
    
    condition1 = sinkron_motor.dir1;
    speed1 = sinkron_motor.speed1;
    jarak_motor1 = sinkron_motor.timer1 / cm_per_pulsa;
    
    condition2 = sinkron_motor.dir2;
    speed2 = sinkron_motor.speed2;
    jarak_motor2 = sinkron_motor.timer2 / cm_per_pulsa;
    
    condition3 = sinkron_motor.dir3;
    speed3 = sinkron_motor.speed3;
    jarak_motor3 = sinkron_motor.timer3 / cm_per_pulsa;

    break;

  case 5:
    memcpy(&command, incomingData, sizeof(command));
    if(command.dir1 == 78){
      encoder_value1_jarak = 0;
      encoder_value2_jarak = 0;
      encoder_value3_jarak = 0;
    }


    break;
    
  }
  
}

int encoder_value_dummy;

void menampilkan_data_serial(){
  Serial.print(kecepatan1);
  Serial.print("\t");
  Serial.print(kecepatan2);
  Serial.print("\t");
  Serial.print(kecepatan3);
  Serial.println("\t");
}

void Split_cmd_vel(char* e) {
  int jumlah_data = 6;
  char* v[jumlah_data];
  char *p;
  int i = 0;
  p = strtok(e, ",");
  while (p && i < jumlah_data) {
    v[i] = p;
    p = strtok(NULL, ",");
    i++;
  };

  linear_x = atof(v[0]);
  linear_y = atof(v[1]);
  linear_z = atof(v[2]);

  angular_x = atof(v[3]);
  angular_y = atof(v[4]);
  angular_z = atof(v[5]);

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

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }


}


void loop() {
  
  unsigned long waktu_sekarang = millis();
  if(waktu_sekarang - waktu_sebelumnya >= 50){
    waktu_sebelumnya = waktu_sekarang;

    kecepatan1 = (float)(((abs(encoder_value1)*1200) / pulsa_per_putaran)*rpm_to_radians*jar_jari_roda);
    current_velocity.v1 = kecepatan1;
    encoder_value1 = 0;

    kecepatan2 = (float)(((abs(encoder_value2)*1200) / pulsa_per_putaran)*rpm_to_radians*jar_jari_roda);
    current_velocity.v2 = kecepatan2;
    encoder_value2 = 0;

    kecepatan3 = (float)(((abs(encoder_value3)*1200) / pulsa_per_putaran)*rpm_to_radians*jar_jari_roda);
    current_velocity.v3 = kecepatan3;
    encoder_value3 = 0;

  }

  if ((millis() - lastTime) >= 100){
    lastTime = millis();
    bno055_read_euler_hrp(&myEulerData);
    current_velocity.theta = float(myEulerData.h) / 16.00;
  }

  unsigned long waktu_display = millis();
  if(waktu_display - waktu_display_sebelumnya >= 200 && flag_kecepatan != 3){
    waktu_display_sebelumnya = waktu_display;

    current_velocity.S1 = encoder_value1_jarak;
    current_velocity.S2 = encoder_value2_jarak;
    current_velocity.S3 = encoder_value3_jarak;
    
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &current_velocity, sizeof(current_velocity));
    if (result != ESP_OK) {
      Serial.println("Error sending the data");
    }

    // menampilkan_data_serial();
  }

  if(Serial.available() >0){
    flag_kecepatan = 3;
    inputString = Serial.readStringUntil('\n'); 
    char inputCharArray[inputString.length() + 1]; 
    inputString.toCharArray(inputCharArray, inputString.length() + 1); 
    Split_cmd_vel(inputCharArray);

    V3 = matrix_kecepatan[0] * linear_x + matrix_kecepatan[1] * linear_y + matrix_kecepatan[2] * angular_z;
    V2 = matrix_kecepatan[3] * linear_x + matrix_kecepatan[4] * linear_y + matrix_kecepatan[5] * angular_z;
    V1 = matrix_kecepatan[6] * linear_x + matrix_kecepatan[7] * linear_y + matrix_kecepatan[8] * angular_z;

  }

  if(flag_kecepatan == 1){
    setMotor1();
    setMotor2();
    setMotor3();

  }else if(flag_kecepatan == 0){
    PIDM1();
    PIDM2();
    PIDM3();

  }else if(flag_kecepatan == 2){
    setMotor1_jarak();
    setMotor2_jarak();
    setMotor3_jarak();
  
  }else if (flag_kecepatan == 3)
  {
    
    Serial.print("Linear X: ");
    Serial.print(linear_x);
    Serial.print(" | Linear Y: ");
    Serial.print(linear_y);
    Serial.print(" | Linear Z: ");
    Serial.print(linear_z);
    Serial.print(" | Angular X: ");
    Serial.print(angular_x);
    Serial.print(" | Angular Y: ");
    Serial.print(angular_y);
    Serial.print(" | Angular Z: ");
    Serial.println(angular_z);
  }
  
} 