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

esp_now_peer_info_t peerInfo;

unsigned long lastTime = 0;  
unsigned long timerDelay = 2000;  // send readings timer

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t sendStatus) {

}

void Split(char* e) {
  char* v[6];
  char *p;
  int i = 0;
  p = strtok(e, ",");
  while (p && i < 6) {
    v[i] = p;
    p = strtok(NULL, ",");
    i++;
  };
  
  if(atoi(v[0]) == 0 || atoi(v[0]) == 1 ){
    myData.a = atoi(v[0]);
    myData.b = atoi(v[1]);
    myData.c = atoi(v[2]);
    myData.d = atof(v[3]);
    myData.e = atof(v[4]);
    myData.f = atof(v[5]);
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  }else if(atoi(v[0]) == 94 || atoi(v[0]) == 7 || atoi(v[0]) == 10){
    myDataPID.a = atoi(v[0]);
    myDataPID.b = atoi(v[1]);
    myDataPID.c = atoi(v[2]);
    myDataPID.d = atof(v[3]);
    myDataPID.e = atof(v[4]);
    myDataPID.f = atof(v[5]);
    esp_now_send(broadcastAddress, (uint8_t *) &myDataPID, sizeof(myDataPID));
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
