#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

const float rpm_to_radians = 0.10471975512;
const float rad_to_deg = 57.29578;
const int pulsa_per_putaran = 1640;
const float jar_jari_roda = 2.9;

bool firstloop = true;
const int BUFFER_SIZE = 50;
char serialBuffer[BUFFER_SIZE];

int en1 = 15;
int in1p = 2;
int in1n = 4;
int enc1A = 13;
int enc1B = 33;

int en2 = 19;
int in2p = 5;
int in2n = 18;
int enc2A = 14;
int enc2B = 27;

int en3 = 23;
int in3p = 21;
int in3n = 22;
int enc3A = 26;
int enc3B = 25;
volatile int encoder_value1 = 0; 
volatile int encoder_value2 = 0; 
volatile int encoder_value3 = 0; 

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
const int resolution = 8;
  
void encoder_isr1() {
  int A = digitalRead(enc1A);
  int B = digitalRead(enc1B);
  if ((A == HIGH) != (B == LOW)) {
    encoder_value1--;
  } else {
    encoder_value1++;
  }
}

void encoder_isr2() {
  int A = digitalRead(enc2A);
  int B = digitalRead(enc2B);
  if ((A == HIGH) != (B == LOW)) {
    encoder_value2--;
  } else {
    encoder_value2++;
  }
}

void encoder_isr3() {
  int A = digitalRead(enc3A);
  int B = digitalRead(enc3B);
  if ((A == HIGH) != (B == LOW)) {
    encoder_value3--;
  } else {
    encoder_value3++;
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
  if(conditionM1 == 1){
    digitalWrite(in1p, LOW);
    digitalWrite(in1n, HIGH);
  } else if(conditionM1 == 0) {
    digitalWrite(in1n, LOW);
    digitalWrite(in1p, HIGH);    
  }

  if(setpointM1 == 0){
    digitalWrite(in1p, LOW);
    digitalWrite(in1n, LOW);
  }

  errorM1 = (setpointM1 - kecepatan1)/100;
  integralM1 += errorM1;
  derivativeM1 = errorM1 - lastErrorM1;
  outputM1 = Kp1 * errorM1 + Ki1 * integralM1 + Kd1 * derivativeM1;

  if (outputM1 > 255) {
    outputM1 = 255;
  } else if (outputM1 < 0) {
    outputM1 = 0;
  }

  ledcWrite(ledChannel, outputM1);
  lastErrorM1 = errorM1;
}

void setPWM1() {
  error1 = (setpoint1 - kecepatan1)/100;
  integral1 += error1;
  derivative1 = error1 - lastError1;
  output1 = Kp * error1 + Ki * integral1 + Kd * derivative1;

  if (output1 > 255) {
    output1 = 255;
  } else if (output1 < 0) {
    output1 = 0;
  }

  ledcWrite(ledChannel, output1);
  lastError1 = error1;
}

void setPWM2() {
  error2 = (setpoint2 - kecepatan2)/100;
  integral2 += error2;
  derivative2 = error2 - lastError2;
  output2 = Kp * error2 + Ki * integral2 + Kd * derivative2;

  if (output2 > 255) {
    output2 = 255;
  } else if (output2 < 0) {
    output2 = 0;
  }

  ledcWrite(ledChannel1, output2);
  lastError2 = error2;
}

void setPWM3() {
  error3 = (setpoint3 - kecepatan3)/100;
  integral3 += error3;
  derivative3 = error3 - lastError3;
  output3 = Kp * error3 + Ki * integral3 + Kd * derivative3;

  if (output3 > 255) {
    output3 = 255;
  } else if (output3 < 0) {
    output3 = 0;
  }

  ledcWrite(ledChannel2, output3);
  lastError3 = error3;
}

typedef struct struct_message {
    int motor_code;
    int function_code;
    float sp;
    int dir;
    float timer;
} struct_message;

struct_message myData;

typedef struct struct_message_pid {
    int motor_code;   
    int function_code;   
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

int condition1 = 0;
int condition2 = 0;
int condition3 = 0; 
float speed1 = 0; 
float speed2 = 0;
float speed3 = 0;
int timer_motor1 = 0;
int timer_motor2 = 0;
int timer_motor3 = 0;
bool flag_timer_motor1=0;
bool flag_timer_motor2=0;
bool flag_timer_motor3=0;
bool flag_kecepatan = 1;

//Transmitter
uint8_t broadcastAddress[] = {0x08, 0xB6, 0x1F, 0x71, 0xBB, 0xEC};
//08:B6:1F:71:BB:EC
typedef struct data_kecepatan {
  float v1;
  float v2;
  float v3;
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
  if(condition1 == 1){
    digitalWrite(in1p, LOW);
    digitalWrite(in1n, HIGH);
  } else if(condition1 == 0) {
    digitalWrite(in1n, LOW);
    digitalWrite(in1p, HIGH);    
  }

  if(condition2 == 1){
    digitalWrite(in2n, LOW);
    digitalWrite(in2p, HIGH);
  } else if(condition2 == 0) {
    digitalWrite(in2p, LOW);
    digitalWrite(in2n, HIGH);
  }

  if(condition3 == 1){
    digitalWrite(in3n, HIGH);
    digitalWrite(in3p, LOW);
  } else if(condition3 == 0) {
    digitalWrite(in3p, HIGH);
    digitalWrite(in3n, LOW);
  }

  if(speed1 == 0){
    digitalWrite(in1p, LOW);
    digitalWrite(in1n, LOW);
  }
  if(speed2 == 0){
    digitalWrite(in2p, LOW);
    digitalWrite(in2n, LOW);
  }
  if(speed3 == 0){
    digitalWrite(in3p, LOW);
    digitalWrite(in3n, LOW);
  }

  setpoint1 = speed1;
  setpoint2 = speed2;
  setpoint3 = speed3;
  setPWM1();
  setPWM2();
  setPWM3();
  
}

void setMotor1() {
  if(condition1 == 1){
    digitalWrite(in1p, LOW);
    digitalWrite(in1n, HIGH);
  } else if(condition1 == 0) {
    digitalWrite(in1n, LOW);
    digitalWrite(in1p, HIGH);    
  }

  if(speed1 == 0){
    digitalWrite(in1p, LOW);
    digitalWrite(in1n, LOW);
  }

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
}

void setMotor2() {
  if(condition2 == 1){
    digitalWrite(in2n, LOW);
    digitalWrite(in2p, HIGH);
  } else if(condition2 == 0) {
    digitalWrite(in2p, LOW);
    digitalWrite(in2n, HIGH);
  }

  if(speed2 == 0){
    digitalWrite(in2p, LOW);
    digitalWrite(in2n, LOW);
  }

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
  
}

void setMotor3() {
  if(condition3 == 1){
    digitalWrite(in3n, HIGH);
    digitalWrite(in3p, LOW);
  } else if(condition3 == 0) {
    digitalWrite(in3p, HIGH);
    digitalWrite(in3n, LOW);
  }

  if(speed3 == 0){
    digitalWrite(in3p, LOW);
    digitalWrite(in3n, LOW);
  }

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
  
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if(incomingData[0] == 0){
    memcpy(&myDataPID, incomingData, sizeof(myDataPID));
    flag_kecepatan = 0;  
    if(incomingData[1] == 94){
      setpointM1 = myDataPID.sp;
      conditionM1 = myDataPID.dir;
      Kp1 = myDataPID.kp;
      Ki1 = myDataPID.ki;
      Kd1 = myDataPID.kd;

    }else if(incomingData[1] == 7){
      setpointM2 = myDataPID.sp;
      conditionM2 = myDataPID.dir;
      Kp2 = myDataPID.kp;
      Ki2 = myDataPID.ki;
      Kd2 = myDataPID.kd;

    }else if(incomingData[1] == 10){
      setpointM2 = myDataPID.sp;
      conditionM2 = myDataPID.dir;
      Kp2 = myDataPID.kp;
      Ki2 = myDataPID.ki;
      Kd2 = myDataPID.kd;
    }
    //Serial.println("Masuk Data PID");

  }else if(incomingData[0] == 1){
    memcpy(&myData, incomingData, sizeof(myData));
    flag_kecepatan = 1;
    if(incomingData[1] == 94){
      condition1 = myData.dir;
      speed1 = myData.sp;
      timer_motor1 = myData.timer;
      if(timer_motor1 > 0){
        flag_timer_motor1 = 1;
      }else{
        flag_timer_motor1 = 0;
      }
      
      
    }else if(incomingData[1] == 7){
      condition2 = myData.dir;
      speed2 = myData.sp; 
      timer_motor2 = myData.timer;
      if(timer_motor2 > 0){
        flag_timer_motor2 = 1;
      }else{
        flag_timer_motor2 = 0;
      }

    }else if(incomingData[1] == 10){
      condition3 = myData.dir;
      speed3 = myData.sp;
      timer_motor2 = myData.timer;
      if(timer_motor3 > 0){
        flag_timer_motor3 = 1;
      }else{
        flag_timer_motor3 = 0;
      }

    }

  }else if(incomingData[0] == 2){
      memcpy(&motor_keseluruhan, incomingData, sizeof(motor_keseluruhan));
      flag_kecepatan = 1;
      condition1 = motor_keseluruhan.dir1;
      condition2 = motor_keseluruhan.dir2;
      condition3 = motor_keseluruhan.dir3;
      speed1 = motor_keseluruhan.speed1;
      speed2 = motor_keseluruhan.speed2;
      speed3 = motor_keseluruhan.speed3;
    //Serial.println("Masuk Data Kecepatan");
  }
}

int encoder_value_dummy;

void setup() {
  pinMode(en1, OUTPUT);
  pinMode(in1p, OUTPUT);
  pinMode(in1n, OUTPUT);

  pinMode(en2, OUTPUT);
  pinMode(in2p, OUTPUT);
  pinMode(in2n, OUTPUT);

  pinMode(en3, OUTPUT);
  pinMode(in3p, OUTPUT);
  pinMode(in3n, OUTPUT);

  pinMode(enc1A, INPUT_PULLUP);
  pinMode(enc1B, INPUT_PULLUP);
  pinMode(enc2A, INPUT_PULLUP);
  pinMode(enc2B, INPUT_PULLUP);
  pinMode(enc3A, INPUT_PULLUP);
  pinMode(enc3B, INPUT_PULLUP);

  digitalWrite(in1p, LOW);
  digitalWrite(in1n, LOW);
  digitalWrite(in2n, LOW);
  digitalWrite(in2p, LOW);
  digitalWrite(in3n, LOW);
  digitalWrite(in3p, LOW);

  attachInterrupt(digitalPinToInterrupt(enc1A), encoder_isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2A), encoder_isr2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc3A), encoder_isr3, CHANGE);
  Serial.begin(115200);

  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(en1, ledChannel);

  ledcSetup(ledChannel1, freq, resolution);
  ledcAttachPin(en2, ledChannel1);

  ledcSetup(ledChannel2, freq, resolution);
  ledcAttachPin(en3, ledChannel2);

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


  unsigned long waktu_display = millis();
  if(waktu_display - waktu_display_sebelumnya >= 200){
    waktu_display_sebelumnya = waktu_display;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &current_velocity, sizeof(current_velocity));
    if (result != ESP_OK) {
      Serial.println("Error sending the data");
    }
  }

 // unsigned long waktu_pid = millis();
  //if(waktu_pid - waktu_pid_sebelumnya >= 5){
   // waktu_pid_sebelumnya = waktu_pid;
  if(flag_kecepatan==1){
    //setMotor(condition1, condition2, condition3, speed1, speed2, speed3);
    setMotor1();
    setMotor2();
    setMotor3();
  }else if(flag_kecepatan==0){
    PIDM1();
  }
  //}


} 


//Maju 0,0,1,0,0.1,0.1
//Mundur 0,1,0,0,0.1,0.1
//Kanan 1,0,0,0.2,0.1,0.1
//Kiri 0,1,1,0.2,0.1,0.1
//Mati 0,0,0,0,0,0