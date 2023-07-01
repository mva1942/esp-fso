/* 
Author: MV Akhil
Date: ~In development~  
motorESP32 Code: WiFi server (or) TCP-WebSocket server
Goes to any 2D coordinates from any initial coord. w/ SUDDEN STOP feature
Capable of remote reset
*/

#include "esp_wpa2.h"
#include <WiFi.h>
#include <WebSocketsServer.h>                         // needed for instant communication between client and server through Websockets
#include <ArduinoJson.h>                              // needed for JSON encapsulation (send multiple variables with one string)

#define home_steppers_button 5 //GPIO34 == D34
#define onboard_led 2

String theta_phi;

int delay_espnow = 30; //Delay between sending two consecutive data via ESP_now

//#define motor_scanning_timePt 15 //Time period in ms
#define motor_scanning_timePt 20 //Time period in ms
//#define motor_scanning_timePt 100 //Time period in ms
unsigned long currentMillis = 0UL;
unsigned long previousMillis = 0UL;

// All variables w/ "1" is for Yaw motion

// Pin configuration for 2 stepper motors
const int dirPin1 = 25;
const int stepPin1 = 26;
const int dirPin2 = 12;
const int stepPin2 = 13;

float theta = 0.0;
float phi = 0.0;

long theta_counter = 0;
long phi_counter = 0;

float status_1;
bool motor1_reverse;
bool proceed = true;
int temp = 1;
bool peak_overshooted = false;

bool rx_corner_pts = false; //changed in rx-callBack fn

//--------WiFi-----------//
// MAC Address of ADC-ESP32
// SSID and password of ESP32 Wifi server:
/*For personal wifi*/
const char* ssid = "anil-fso";
const char* password = "80189060";

/*For enterprise wifi*/
//const char* ssid = "iitmwifi"; // your ssid
//#define EAP_ID "ee20s132@iitm.ac.in" // roll no in SMALL CAPS
//#define EAP_USERNAME "ee20s132" // roll no in SMALL CAPS
//#define EAP_PASSWORD "cKfaJ58U}!t" // CASE SENSITIVE password

//const char * host = "10.42.12.159"; //adc-ESP IP addr.
const uint16_t port = 81;
byte client_id = 7; // Max 5 clients(0-4) are only allowed, so 7 is an invalid client id
byte prev_client_id = 7;

bool connection_success = false;

// The JSON library uses static memory, so this will need to be allocated:
StaticJsonDocument<200> doc_tx;                       // provision memory for about 200 characters
StaticJsonDocument<200> doc_rx;

// Initialization of websocket
WebSocketsServer webSocket = WebSocketsServer(port);    // the websocket uses port 81 (standard port for websockets

// ESPNow data-structure
typedef struct corner_pts {
  int theta; //desired theta,phi to be sent by adcESP
  int phi;
  int theta_resol;
  int phi_resol;
  bool corner_reached; // This is modified by only motor-ESP
  bool stop_scan;
  bool server_reset;
} corner_pts;

corner_pts sent_info; // initialized in setup()
corner_pts rx_info;


// Callback when data is received
void webSocketEvent(byte num, WStype_t type, uint8_t * payload, size_t length) {      // the parameters of this callback function are always the same -> num: id of the client who send the event, type: type of message, payload: actual data sent and length: length of payload
  switch (type) {                                     // switch on the type of information sent
    case WStype_DISCONNECTED:                         // if a client is disconnected, then type == WStype_DISCONNECTED
      Serial.println("Client " + String(num) + " disconnected");
      break;
    case WStype_CONNECTED:                            // if a client is connected, then type == WStype_CONNECTED
      Serial.println("Client " + String(num) + " connected");
      break;
    case WStype_PING:
      Serial.println("In ping case");
      break;
    case WStype_PONG:
      // When client is actually connected, it sends a ping request to us(in-build)..
      //...to which we respond pong(in-build)
      //...this is a better way to collect connected client's ID
      Serial.println("In pong case: recorded client_id");
      client_id = num;
      connection_success = true; //used only for first time client connection

      //If client reconnects
      if(prev_client_id != client_id) digitalWrite(onboard_led,LOW);
      break;    
    case WStype_TEXT:                                 // if a client has sent data, then type == WStype_TEXT
      // try to decipher the JSON string received
      DeserializationError error = deserializeJson(doc_rx, payload);
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
      }
      else {
        // JSON string was received correctly, so information can be retrieved:
        rx_info.server_reset = doc_rx["server_reset"];
        if(rx_info.server_reset){
          Serial.println("######### remote reset activated");
          ESP.restart();
        }
        rx_info.theta = doc_rx["theta"];
        rx_info.phi = doc_rx["phi"];
        rx_info.theta_resol = doc_rx["theta_resol"];
        rx_info.phi_resol = doc_rx["phi_resol"];
        rx_info.stop_scan = doc_rx["stop_scan"];

        if(rx_info.stop_scan == false){
          Serial.println("rx corner pts.");
          rx_corner_pts = true;
        }
        else Serial.println("$$$$$$$ Rejected rx corner as scan is stopped abruptly");
      }  
      break;
  }
}

//-----WiFi ends---------//

int coarse_peak;

volatile bool home_steppers_flag = false;
int peak_flag = 0;
int pulse_rx = 0;

portMUX_TYPE synch = portMUX_INITIALIZER_UNLOCKED;  // helps in syncing isr and main code

void IRAM_ATTR isr_home_steppers() {
  /*ISR for reset button*/
  portENTER_CRITICAL(&synch);
  
  home_steppers_flag = true;
//  proceed = false;  // To make sure no more scanning takes place

  portEXIT_CRITICAL(&synch);
}



//=================================================================================//
void setup() {  
//  sent_info.corner_reached = false;// signals after reaching desired location
//  sent_info.msg_delivered = false;// signals after getting the desired loc. info. from adcESP

  //motor-pins
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  motor1_reverse = false;

  digitalWrite(stepPin1,LOW);
  digitalWrite(stepPin2,LOW);

  pinMode(onboard_led,OUTPUT);
  pinMode(home_steppers_button,INPUT); //Pulled up externally w/ 10k ~5V
  attachInterrupt(home_steppers_button,isr_home_steppers,FALLING);
  digitalWrite(onboard_led,LOW);
  
  Serial.begin(115200);

  //-----------WiFi---------------//
  // start WiFi interface

  /*For personal WiFi*/
  WiFi.begin(ssid,password);
  
  // WPA2 enterprise magic starts here
//  WiFi.disconnect(true);  
//  WiFi.mode(WIFI_STA); //init wifi mode    
//  esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_ID, strlen(EAP_ID));
//  esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_USERNAME, strlen(EAP_USERNAME));
//  esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD));
//  esp_wifi_sta_wpa2_ent_enable();
//  WiFi.begin(ssid);
  // WPA2 enterprise magic ends here  
 
  while (WiFi.status() != WL_CONNECTED) {             // wait until WiFi is connected
    delay(1000);
    Serial.print(".");
  }
  digitalWrite(onboard_led,HIGH); // Indicating connected to WiFi
  Serial.print("Connected to network with IP address: ");
  Serial.println(WiFi.localIP());                     // show IP address that the ESP32 has received from router
  
  webSocket.begin();                                  // start websocket
  webSocket.onEvent(webSocketEvent);                  // define a callback function -> what does the ESP32 need to do when an event from the websocket is received? -> run function "webSocketEvent()"
  
  //waiting for the client to connect for the v first time
  while(connection_success == false){
    webSocket.loop();
  }
  prev_client_id = client_id; 
  
  Serial.println("Is client-" + String(client_id) + " Connected(0/1)= " + String(webSocket.clientIsConnected(client_id)));

//  digitalWrite(onboard_led,HIGH); // Indicating connection is setup
  Serial.println("################## Exiting setup() fn.");
}

void loop() {
  //No data tx-rx: physical home pos. reset is an indipendent fn.
  //Won't goto home once rx a corner pt., to make sure both adcESP and motorESP are in sync.
  if(home_steppers_flag) home_steppers(); 
  
  webSocket.loop();

  digitalWrite(onboard_led,HIGH); 
  //waiting for corner points:
  while(true){
    webSocket.loop();

    //If client re-connects
    if(prev_client_id != client_id){
      webSocket.disconnect(prev_client_id);
      Serial.println("Client-" + String(prev_client_id) + " disconnected");
      prev_client_id = client_id;
      Serial.println("no. of clients connected: " + String(webSocket.connectedClients()));
      digitalWrite(onboard_led,HIGH); //It became off when client re-connected in callback-fn.
    }
    
    if(home_steppers_flag) home_steppers();
    if(rx_corner_pts) break;
  }
  rx_corner_pts = false;
  
  //sends DELIVERY ACK., its made sure at adcESP that only delivery boolean is read.
//  sent_info.msg_delivered = true;
//  sent_info.corner_reached = false;
//  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sent_info, sizeof(sent_info));  

  //storing rx corner points
  int theta_destination = rx_info.theta;
  int phi_destination = rx_info.phi;

  //storing the current location in msg(yet to be sent).
  sent_info.theta = theta_counter;
  sent_info.phi = phi_counter;

  if(theta_counter == theta_destination and phi_counter == phi_destination){
    //sends current location and REACHED LOCATION, its made sure at adcESP that only REACHED LOCATION boolean is read.
    sent_info.corner_reached = true;

    Serial.println("sent termination from main loop");
//    Serial.println("actual current coord: (" + String(theta_counter) + "," 
//                    + String(phi_counter) + ")"); 
    send_data_wifi(sent_info);
  }

  else{
    //sends only the current location before going to the destination
    sent_info.corner_reached = false;
//    Serial.println("actual current coord: (" + String(theta_counter) + "," 
//                    + String(phi_counter) + ")"); 
    send_data_wifi(sent_info);
  }

  while(theta_counter != theta_destination or phi_counter !=phi_destination){
    webSocket.loop(); // polling for incoming wifi data
    
    //Checking if current motors need to abruptly stop
    if(rx_info.stop_scan){
      Serial.println("#########Abrupt stop sgn rx");
      Serial.println("Got this along with previous corner: " 
                      + String(rx_info.theta) + "," + String(rx_info.phi));
      rx_corner_pts = false;

      //Sending corner-reached, even though false, to stop ADC-ESP from waiting till reached
      sent_info.corner_reached = true;
      sent_info.theta = theta_counter;
      sent_info.phi = phi_counter;
      send_data_wifi(sent_info);
      Serial.println("######### Sent the stop ack- to make ADC-ESP not wait");
      digitalWrite(onboard_led,LOW);
      break;// Coming out and waiting for next corner-pt.
    }

    //moving motor step-by-step
    currentMillis = millis();
    if(currentMillis - previousMillis >= motor_scanning_timePt){
      goto_destination(theta_destination,phi_destination);
      previousMillis = currentMillis;
    }
  }
}
//=================================================================================//

//==========Sending data via WiFi=================//
void send_data_wifi(corner_pts sent_info){
/*Sends data to the client(adcESP) via WiFi*/  
  String jsonString;                                    // Temporary storage for JSON String
  JsonObject object = doc_tx.to<JsonObject>();          // Create JSON object
  object["theta"] = sent_info.theta;
  object["phi"] = sent_info.phi;
  object["corner_reached"] = sent_info.corner_reached;
  serializeJson(doc_tx, jsonString);                    // convert JSON object to string
  webSocket.sendTXT(client_id,jsonString); // Send data to the server

  Serial.println("Sent coord(teta,phi,reached): (" + String(sent_info.theta) + "," 
                    + String(sent_info.phi)
                    + "," + String(sent_info.corner_reached) + ")");
}

// https://stackoverflow.com/questions/9072320/split-string-into-string-array
String split_string(String data, char separator, int index)
{/*To split the given string, given the delimitor, and return i^th string ele.*/
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void goto_destination(int theta_d, int phi_d){
/*Goes to desired coordinates theta_d and phi_d
  phi first, then theta first*/  
  digitalWrite(onboard_led,HIGH);
  bool phi_done = false;
  
  //First reach the phi_d, then only reach the theta_d
  if(phi_counter < phi_d){
    one_step(String("pitch"),HIGH);
  }
  else if(phi_counter > phi_d){
    one_step(String("pitch"),LOW);
  }
  else phi_done = true;  
  
  if(theta_counter < theta_d and phi_done){
    one_step(String("yaw"),HIGH);
  }
  else if(theta_counter > theta_d and phi_done){
    one_step(String("yaw"),LOW);
  }  
  
  if(theta_counter == theta_d and phi_counter == phi_d){    
    //sends final point and REACHED LOCATION, its made sure at adcESP that only REACHED LOCATION boolean is read.
    delay(delay_espnow); /*v. imp delay(w/o this, corner reach sgn. won't reach adcESP): delay b/w sending coord from onestep() and this fn.*/
    sent_info.theta = theta_counter;
    sent_info.phi = phi_counter;
    sent_info.corner_reached = true;
    Serial.println("sent termination from inside fn.");

    send_data_wifi(sent_info);
    digitalWrite(onboard_led,LOW);
  }  
}

void one_step(String yaw_pitch, bool dir){
/*Moves the stepper motor by exactly one step*/
  if(String("yaw") == yaw_pitch){
     digitalWrite(dirPin1,dir);
     digitalWrite(stepPin1,HIGH);
     delayMicroseconds(100); // Can be 1us from A4988 driver datasheet
     digitalWrite(stepPin1,LOW);

     if(dir){
        theta_counter ++;
     }
     else{
        theta_counter --;
     }
  }
  else if(String("pitch") == yaw_pitch){
     digitalWrite(dirPin2,dir);
     digitalWrite(stepPin2,HIGH);
     delayMicroseconds(100); // Can be 1us from A4988 driver datasheet
     digitalWrite(stepPin2,LOW);

     if(dir){
        phi_counter ++;
     }
     else{
        phi_counter --;
     }
  }
  else{
    Serial.println("Wrong input to one_step fn.");
  }

  //-----Sending theta-phi according to the selected resolution.
  if(String("yaw") == yaw_pitch and theta_counter % rx_info.theta_resol == 0){
    sent_info.theta = theta_counter;
    sent_info.phi = phi_counter;
    sent_info.corner_reached = false;
    send_data_wifi(sent_info);
  }

  else if(String("pitch") == yaw_pitch and phi_counter % rx_info.phi_resol == 0){
    sent_info.theta = theta_counter;
    sent_info.phi = phi_counter;
    sent_info.corner_reached = false;
    send_data_wifi(sent_info);
  }
}


void home_steppers(){
/*Goes to initial pos. theta,phi = 0, DOESN'T wait for scanning pulses*/  
  home_steppers_flag = false;

  Serial.println("Before home reset");
  Serial.print(theta_counter);Serial.print(" ");Serial.println(phi_counter);
  detachInterrupt(home_steppers_button);
  digitalWrite(dirPin2, LOW);
  digitalWrite(dirPin1, LOW);
  digitalWrite(onboard_led,HIGH);
  
  while(phi_counter != 0){
    Serial.print("in phi return ");
    Serial.println(phi_counter);
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(10000);
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(2000);
    if(phi_counter >= 0) phi_counter --;
    else if(phi_counter <= 0) phi_counter ++;
  }

  while(theta_counter != 0){
    Serial.print("in theta return ");
    Serial.println(theta_counter);  
    digitalWrite(stepPin1, HIGH);
    delayMicroseconds(10000);
    digitalWrite(stepPin1, LOW);
    delayMicroseconds(2000);
    if(theta_counter >= 0) theta_counter --;
    else if(theta_counter <= 0) theta_counter ++;
  }
  Serial.println("After home reset");
  Serial.print(theta_counter);Serial.print(" ");Serial.println(phi_counter); 
  digitalWrite(onboard_led,LOW);
}
