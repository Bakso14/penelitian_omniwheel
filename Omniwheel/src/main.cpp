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

int nilai_sebelumnya1, nilai_sebelumnya2, nilai_sebelumnya3 = 0;

float kecepatan1, kecepatan2, kecepatan3 = 0;

//PID
const float Kp = 2.0;  // Proporsional
const float Ki = 0.1;  // Integral
const float Kd = 0.01; // Turunan

//Setpoint
float setpoint1 = 0.2;
float setpoint2 = 0.2;
float setpoint3 = 0.2;

bool motorConn1;
bool motorConn2;
bool motorConn3;

//function

// Variabel PID
float error1, lastError1, integral1, derivative1, output1;
float error2, lastError2, integral2, derivative2, output2;
float error3, lastError3, integral3, derivative3, output3;
float errorM1, lastErrorM1, integralM1, derivativeM1, outputM1;
float errorM2, lastErrorM2, integralM2, derivativeM2, outputM2;
float errorM3, lastErrorM3, integralM3, derivativeM3, outputM3;

void PIDM1(float setpointM1, float Kp1, float Ki1, float Kd1) {
  errorM1 = (setpointM1 - kecepatan1)/100;
  integralM1 += errorM1;
  derivativeM1 = errorM1 - lastErrorM1;
  outputM1 = Kp * error1 + Ki * integral1 + Kd * derivative1;

  if (output1 > 255) {
    output1 = 255;
  } else if (output1 < 0) {
    output1 = 0;
  }

  ledcWrite(ledChannel, output1);
  lastError1 = error1;
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
    int a;
    int b;
    int c;
    float d;
    float e;
    float f;
} struct_message;

struct_message myData;

typedef struct struct_message_pid {
    int a;
    float b;
    int c;
    float d;
    float e;
    float f;
} struct_message_pid;

struct_message_pid myDataPID;

int condition1;
int condition2;
int condition3; 
float speed1; 
float speed2;
float speed3;

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
  digitalWrite(in1p, LOW);
  digitalWrite(in1n, LOW);
  digitalWrite(in2n, LOW);
  digitalWrite(in2p, LOW);
  digitalWrite(in3n, LOW);
  digitalWrite(in3p, LOW);
  
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

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if(incomingData[0] == 0 || incomingData[0] == 1){
    memcpy(&myData, incomingData, sizeof(myData));
    condition1 = myData.a;
    condition2 = myData.b;
    condition3 = myData.c; 
    speed1 = myData.d; 
    speed2 = myData.e;
    speed3 = myData.f;
    setMotor(condition1, condition2, condition3, speed1, speed2, speed3);
  }else if(incomingData[0] == 94 || incomingData[0] == 7 || incomingData[0] == 10){
    memcpy(&myDataPID, incomingData, sizeof(myDataPID));

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
    Serial.print("V1= ");
    Serial.print(kecepatan1);
    Serial.print("\t");

    Serial.print("V2= ");
    Serial.print(kecepatan2);
    Serial.print("\t");

    Serial.print("V3= ");
    Serial.print(kecepatan3);

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &current_velocity, sizeof(current_velocity));
    if (result == ESP_OK) {
      Serial.println("  Sent with success");
    }
    else {
      Serial.println("  Error sending the data");
    }

  }


  if (firstloop = true) {
    setMotor(condition1, condition2, condition3, speed1, speed2, speed3);
    firstloop = false;
  }
  
} 


//Maju 0,0,1,0,0.1,0.1
//Mundur 0,1,0,0,0.1,0.1
//Kanan 1,0,0,0.2,0.1,0.1
//Kiri 0,1,1,0.2,0.1,0.1
//Mati 0,0,0,0,0,0