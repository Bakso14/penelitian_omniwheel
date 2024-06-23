#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

String inputString;
String dataIn;
String dt[10];
int i;
boolean parsing = false;
uint8_t broadcastAddress[] = {0xB0, 0xA7, 0x32, 0xDB, 0xBC, 0x88};

//Reveiver
typedef struct data_kecepatan {
  float v1;
  float v2;
  float v3;
}data_kecepatan;

data_kecepatan current_velocity;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&current_velocity, incomingData, sizeof(current_velocity));
  Serial.print(current_velocity.v1);
  Serial.print(";");

  Serial.print(current_velocity.v2);
  Serial.print(";");

  Serial.print(current_velocity.v3);
  Serial.println(";");
}

//END

// Structure example to send data
// Must match the receiver structure
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

esp_now_peer_info_t peerInfo;

unsigned long lastTime = 0;  
unsigned long timerDelay = 2000;  // send readings timer

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t sendStatus) {

}

void Split(char* e) {
  char* v[7];
  char *p;
  int i = 0;
  p = strtok(e, ",");
  while (p && i < 7) {
    v[i] = p;
    p = strtok(NULL, ",");
    i++;
  };
  
  if(atoi(v[0]) == 0){
    myDataPID.function_code = atoi(v[0]);
    myDataPID.motor_code = atoi(v[1]);
    myDataPID.sp = atof(v[2]);
    myDataPID.dir = atof(v[3]);
    myDataPID.kp = atof(v[4]);
    myDataPID.ki = atof(v[5]);
    myDataPID.kd = atof(v[6]);
    esp_now_send(broadcastAddress, (uint8_t *) &myDataPID, sizeof(myDataPID));
    
  }else if(atoi(v[0]) == 1){
    myData.function_code = atoi(v[0]);
    myData.motor_code = atoi(v[1]);
    myData.sp = atof(v[2]);
    myData.dir = atoi(v[3]);
    myData.timer = atof(v[4]);
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

  }else if(atoi(v[0]) == 2){
    motor_keseluruhan.function_code = atoi(v[0]);
    motor_keseluruhan.dir1 = atoi(v[1]);
    motor_keseluruhan.dir2 = atoi(v[2]);
    motor_keseluruhan.dir3 = atoi(v[3]);
    motor_keseluruhan.speed1 = atof(v[4]);
    motor_keseluruhan.speed2 = atof(v[5]);
    motor_keseluruhan.speed3 = atof(v[6]);
    esp_now_send(broadcastAddress, (uint8_t *) &motor_keseluruhan, sizeof(motor_keseluruhan));

  }

};

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    return;
  }
}

void loop() {
  // Set values to send
  if (Serial.available() > 0) {
    inputString = Serial.readStringUntil('\n'); // Read the input as a string
    char inputCharArray[inputString.length() + 1]; // Create a char array
    inputString.toCharArray(inputCharArray, inputString.length() + 1); // Convert string to char array
    Split(inputCharArray); // Pass the char array to Split function

    delay(1000);
    //Serial.println(inputString);
  }
  
}
