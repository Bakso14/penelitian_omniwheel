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
  Serial.print("V1= ");
  Serial.print(current_velocity.v1);
  Serial.print("\t");

  Serial.print("V2= ");
  Serial.print(current_velocity.v2);
  Serial.print("\t");

  Serial.print("V3= ");
  Serial.println(current_velocity.v3);
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

esp_now_peer_info_t peerInfo;

unsigned long lastTime = 0;  
unsigned long timerDelay = 2000;  // send readings timer

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == ESP_NOW_SEND_SUCCESS){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
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
  Serial.println(v[0]);
  Serial.println(v[1]);
  Serial.println(v[2]);
  Serial.println(v[3]);
  Serial.println(v[4]);
  Serial.println(v[4]);
  myData.a = atoi(v[0]);
  myData.b = atoi(v[1]);
  myData.c = atoi(v[2]);
  myData.d = atof(v[3]);
  myData.e = atof(v[4]);
  myData.f = atof(v[5]);
  esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
};

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
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
    Serial.println(inputString);
  }
  
}
