/*
Author: MV Akhil
Date: 01/07/2023
adcESP32 Code: WiFi client (or) TCP-WebSocket client
Find and Go-back to coarse-peak, then start SGD, find peak using convergance criteria and observe-link for x min.
Can remotly reset server upon button press.
*/

// Include Libraries
#include "esp_wpa2.h"
#include <Statistical.h>
#include <WiFi.h>                                     // needed to connect to WiFi
#include <WebSocketsClient.h>                         // needed for instant communication between client and server through Websockets
#include <ArduinoJson.h>                              // needed for JSON encapsulation (send multiple variables with one string)

#define onboard_led 2
#define BS_calib_button 34 //GPIO34 == D34
#define remote_reset_button 5 //GPIO5 == D05, not 26
#define ADC_ip_pin 35 //GPIO35 == D35

int delay_espnow = 30; //Delay between sending two consecutive data via ESP_now
float link_obs_time = 1; // in minutes
int obs_delay = 250; //sampling time during link observations

/*Based on LIA o/p LPF's f_cuttoff*/
#define ADC_sampling_time 1250 // ADC sampling time in micro-sec(us)----IDEAL MIN----f=160Hz
//#define ADC_sampling_time 1660 // ADC sampling time in micro-sec(us)----REAL MIN based on THD_pcbLIA----f=120Hz

bool BS_calib_flag = false;
bool start_flag = false;
bool terminate_goHome = false; // updated in BS_calib_isr AND used only to break the link_observe.

//------variables req. for corner_pt. calculations----------//
/*---Fine-scan specs.*/
int theta_max = 200;
int phi_max = 200;

//Both CANNOT be 0 at-a-time, else motors cover sq. boundary in infinity
int theta_gap = 10;
int phi_gap = 10;

int theta_resol = 10;
int phi_resol = 10;

// Offset from manually aligned peak to start scan
int self_offset_theta = -10;
int self_offset_phi = -10;

// fine scan theta_max/2 and phi_max/2
//int theta_overshoot = 25;
//int phi_overshoot = 25;
int theta_overshoot = 0;
int phi_overshoot = 0;

int counter = 0.0;
int local_counter = 0;
int touple_counter = 0;
bool scan_done = false;

int theta_stop = ceil(float(theta_max)/2);
int phi_stop = ceil(float(phi_max)/2);

int theta = 0;
int phi = 0;

int theta_prev = 0;
int phi_prev = 0;

int theta1,theta2,theta3,theta4;
int phi1,phi2,phi3,phi4;

typedef struct angles{
  int theta;
  int phi;
} angles;

angles corner_pts;

//----------ADC variables-----------//
float LIA_delta = 0.0; //LIA op w/ LOS present - BS
float LIA_bs = 0.0; //LIA op w/o LOS, BaseLine
float LIA_op = 0.0;
float scan_dc_offset = 0.0; //To offset DC shift in LIA o/p, offset calc. after coarse scan 

//-------------GoHome: switch debounce----------------//
bool buttonState;            // the current button state
bool lastButtonState = HIGH;  // Active low button
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

//-------------Steepest Gradient Descend----------------//
bool store_in_mat = false;

//_______SGD parameters__________
byte diff_step = 4;
byte max_iterations = 15;
float alpha_theta = 2; //Learning rate
float alpha_phi = 2;
float convergance_perc = 20;
float grad_sign;

//_______2D matrix defined__________
const int theta_fine_scan_bound = 50;
const int phi_fine_scan_bound = 50;
float LIA_2d_arr[theta_fine_scan_bound][phi_fine_scan_bound];
int theta_fineOrigin, phi_fineOrigin;

typedef struct grad{
  float s_theta;
  float s_phi;
} grad;


//-------------WiFi----------------//
// SSID and password of ESP32 Wifi server:
const char* ssid = " "; // your ssid
#define EAP_ID " " // roll no in SMALL CAPS
#define EAP_USERNAME " " // roll no in SMALL CAPS
#define EAP_PASSWORD " " // CASE SENSITIVE password

const char * host = " "; //motor-ESP IP addr.
const uint16_t port = 81;

bool connection_success = false;  // To know if connected to motor-ESP(server)
const int msg_size = 200; //Message size in bytes

WebSocketsClient webSocket; // websocket client class instance
StaticJsonDocument<msg_size> doc_tx; // Allocate a static JSON document
StaticJsonDocument<msg_size> doc_rx; // Allocate a static JSON document

// ESPNow data-structure
typedef struct corner_points_struct {
  int theta; //desired theta,phi to be sent by adcESP
  int phi;
  int theta_resol;
  int phi_resol;
  bool corner_reached; // This is only modified by motor-ESP
  bool stop_scan; // To abruptly stop the scan at motor-esp side
  bool server_reset; // To remotly send reset cmd to motor-esp
} corner_points_struct;

corner_points_struct sent_info;
//corner_points_struct rx_info;

//_______Arrays defined__________
const int arr_size = 2000;
int theta_rx_arr[arr_size];
int phi_rx_arr[arr_size];
float LIA_delta_arr[arr_size];
int arr_index = 0;

bool corner_reached = false;

/* Fn. to send data via WiFi..
..second argument, by default=T, means by default will wait for motors to reach corner*/
void send_data_wifi(corner_points_struct sent_info, bool wait_for_motors = true); // Fn. defined after loop()

// Callback when data is received
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch (type) { // switch on the type of information sent
    case WStype_CONNECTED:
      connection_success = true;
      Serial.println("Server connected");
      break;
    case WStype_DISCONNECTED:
      connection_success = false;
      Serial.println("Server disconnected");
      break;
    case WStype_PING:
      Serial.println("In ping case");
      break;
    case WStype_PONG:
      Serial.println("In pong case");
      break;
      
    case WStype_TEXT: // Case when actual message is recieved
      DeserializationError error = deserializeJson(doc_rx, payload); // deserialize incoming Json String
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
      }

      //Loading variables
      const int rx_theta = doc_rx["theta"];
      const int rx_phi = doc_rx["phi"];
      const bool rx_corner_reached = doc_rx["corner_reached"];
      
      //flag checking if motor has reached the desired corner-pt.
      corner_reached = rx_corner_reached;

      //Reading LIA voltage at each rx motor coordinate
      LIA_op = roundf(analogReadAdjusted(ADC_ip_pin,5)*100)/100;  //Rounding to 2nd decimal    
      LIA_delta = LIA_op - LIA_bs;
      LIA_delta -= scan_dc_offset; // To offset DC shift in LIA o/p, offset calc. after coarse scan 

      //Storing the data only when the reading is within +/- 14V, to avoid random adc noise
      if (abs(LIA_delta) <= 14){
        //storing the current coordinates of motor-ESP
        theta_rx_arr[arr_index] = rx_theta;
        phi_rx_arr[arr_index] = rx_phi;
        LIA_delta_arr[arr_index] = LIA_delta;
    
        //printing the stored current coords. for plotting
        Serial.print(String(LIA_delta_arr[arr_index]));
        Serial.println(" " + String(theta_rx_arr[arr_index])
                        + " " + String(phi_rx_arr[arr_index])
                        + " " + String(corner_reached)); 
    
        //storing in 2d matrix, ONLY when SGD begins: coord always within fine-scan box
        if(store_in_mat == true){
    //      Serial.print("Should record theta,phi: " + String(rx_info.theta) + "," + String(rx_info.phi));
          LIA_2d_arr[int(rx_theta-theta_fineOrigin)][int(rx_phi-phi_fineOrigin)]= LIA_delta;                    
        }
            
        arr_index++; //Updating index only when data is stored
      }
      else{
      Serial.print("Rejected the corner > +/-14V: "); 
      Serial.print(String(LIA_delta));
      Serial.println(" " + String(rx_theta)
                        + " " + String(rx_phi)
                        + " " + String(corner_reached)); 
     }
     break; 
  }
}

//-----Caliberation button setup-----//
portMUX_TYPE synch = portMUX_INITIALIZER_UNLOCKED;  // helps in syncing isr and main code

void IRAM_ATTR isr_BS_calib() {
  /*ISR*/
  portENTER_CRITICAL(&synch);  
  detachInterrupt(BS_calib_button);
  BS_calib_flag = true; 
  start_flag = true;
  terminate_goHome = true;
  Serial.println("in isr");
  portEXIT_CRITICAL(&synch);
}

//-----Remote reset button setup-----//
portMUX_TYPE synch2 = portMUX_INITIALIZER_UNLOCKED;  // helps in syncing isr and main code

void IRAM_ATTR isr_remote_reset() {
  /*ISR*/
  portENTER_CRITICAL(&synch2);  
  detachInterrupt(remote_reset_button);
  Serial.println("remote reset button pressed");
  sent_info.server_reset = true;
  portEXIT_CRITICAL(&synch2);
}

void setup() {  
  if(theta_stop == theta_max){
    theta_stop = 0;}
  if(phi_stop == phi_max){
    phi_stop = 0;} 

  sent_info.stop_scan = false; // Abrupt scan stop NA in this code.
  sent_info.server_reset = false;

  //-------ADC-------------
  adcAttachPin(ADC_ip_pin);
//  LIA_bs = BS_calib(); //Caliberating ADC's baseline
  
  // Set up Serial Monitor
  Serial.begin(500000);
//  Serial.begin(115200);

  pinMode(BS_calib_button,INPUT); //Pulled up externally w/ 10k ~5V
  pinMode(onboard_led,OUTPUT);
  attachInterrupt(BS_calib_button,isr_BS_calib,FALLING);
  digitalWrite(onboard_led,LOW);

  pinMode(remote_reset_button,INPUT); //Pulled up externally w/ 10k ~5V

//==============WiFi===================// 
  // start WiFi interface
  // WPA2 enterprise magic starts here
  WiFi.disconnect(true);  
  WiFi.mode(WIFI_STA); //init wifi mode    
  esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_ID, strlen(EAP_ID));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_USERNAME, strlen(EAP_USERNAME));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD));
  esp_wifi_sta_wpa2_ent_enable();
  WiFi.begin(ssid);
  // WPA2 enterprise magic ends here  
 
  while (WiFi.status() != WL_CONNECTED) {             // wait until WiFi is connected
    delay(1000);
    Serial.print(".");
  }
  
  Serial.print("Connected to network with IP address: ");
  Serial.println(WiFi.localIP());                     // show IP address that the ESP32 has received from router
  
  webSocket.begin(host, port, "/");           // Connect to ESP32 server websocket (port 81)
  webSocket.onEvent(webSocketEvent);                  // Define a callback function -> what does the ESP32 need to do when an event from the websocket is received? -> run function "webSocketEvent()"
  webSocket.setReconnectInterval(2000);               // if connection failed retry every 5s

  while(connection_success == false){// flag is controlled inside WebSocket callback fn
    webSocket.loop();
  }
  attachInterrupt(remote_reset_button, isr_remote_reset, FALLING);
  digitalWrite(onboard_led,HIGH); // Indicating connection is setup
  Serial.println("################## Exiting setup() fn.");
}

void loop() {
  //waiting for button-press to start
  while(start_flag == false){
      webSocket.loop(); // Keep the socket alive, polls for data rx
      if(sent_info.server_reset){
        //Sending (0,0) first before resetting
        delay(delay_espnow);
        sent_info.theta = 0;
        sent_info.phi = 0;
        sent_info.theta_resol = 10;
        sent_info.phi_resol = 10;
        sent_info.server_reset = false;
        Serial.println("Sending (0,0) corner");
        corner_reached = false; // To compensate for old data recieved, if any.
        send_data_wifi(sent_info);

        //----COMMENT FOLLOWING 2-lines if want to perform remote-reset--------//
        delay(100);
        ESP.restart();
        
        //Sending the actual remote reset
        delay(delay_espnow);
        sent_info.server_reset = true;
        send_data_wifi(sent_info,false); // Send remote reset w/o waiting for motors to reach
      }
  }
  detachInterrupt(remote_reset_button); // To avoid resetting motors when they arent at (0,0);
  Serial.println("out of waiting");
  digitalWrite(onboard_led,LOW);
  start_flag = false;

  // Offsetting from manually aligned peak
  delay(delay_espnow);
  sent_info.theta = self_offset_theta;
  sent_info.phi = self_offset_phi;
  sent_info.theta_resol = 10;
  sent_info.phi_resol = 10;
  sent_info.server_reset = false;
  Serial.println("Sending offset from manual peak: " + String(self_offset_theta) + "," + String(self_offset_phi));
  send_data_wifi(sent_info);

  LIA_bs = BS_calib(); //Caliberating ADC's baseline
  Serial.println("pp-calib " + String(LIA_bs));
  arr_index = 0; //To rewrite the array with coarse-scan data
  
//  //Taking dummy reading to avoid noise in the coarse_scan 
//  LIA_op = roundf(analogReadAdjusted(ADC_ip_pin,5)*100)/100;  //Rounding to 2nd decimal    
//  LIA_delta = LIA_op - LIA_bs;

  /*---Coarse-scan specs. already loaded in start of program and in setup()*/

  //-------------------Coarse scanning-------------------
  Serial.println("\n~~~~~~~~~~~~~~~~~~~~~~~~~bool_Coarse scanning");
  unsigned long coarseScan_start_millis = millis();
//  do_scan(0,0);
  do_scan(self_offset_theta, self_offset_phi);
  Serial.println("@@@@@@@@@@@@@@@@@ Time taken for coarse scan = " + String(millis() - coarseScan_start_millis)
                  + " ms");  
  Serial.println("array index= " + String(arr_index));

  // Removing the DC offset
  Array_Stats<float> Data_Array(LIA_delta_arr, sizeof(LIA_delta_arr) / sizeof(LIA_delta_arr[0]));
  float temp_avg = Data_Array.Average(Data_Array.Arithmetic_Avg);
  /*From next line onwards, all the ADC values will be offsetted to counter for DC shift in callback fn.*/
  if(arr_index != 0) scan_dc_offset = (temp_avg * (float)arr_size)/(float)arr_index; 
  else Serial.println("################################--- arr_index = 0, could NOT find the DC shift offset");
  Serial.println("pp-DCOffset " + String(scan_dc_offset));

  //Finding global max of LIA_readings #Coarse_peak 
  int max_index = 0;

  // (not)IGNORING the first sample(i=1 and not 0) while finding coarse peak: found that usually its noisy...sometimes more than the coarse peak!
  for(int i=0; i<arr_index; i++){
    /*This stores the last occurance of the global peak*/
    LIA_delta_arr[i] -= scan_dc_offset; // Offsetting the already recorded coarse scan data to account for DC shift
    if(abs(LIA_delta_arr[i]) >= abs(LIA_delta_arr[max_index])) max_index = i;
  }

  Serial.println("pp-cp " + String(LIA_delta_arr[max_index]) 
                  + " " + String(theta_rx_arr[max_index]) + " " + String(phi_rx_arr[max_index]));

  int theta_peakOvershoot = theta_rx_arr[max_index] - theta_overshoot;
  int phi_peakOvershoot = phi_rx_arr[max_index] - phi_overshoot;

  Serial.println("pp-ov " + String(theta_peakOvershoot) + " " + String(phi_peakOvershoot));

  //Checking the sign of coarse-peak to decide the dir. in which SGD must move
  if(LIA_delta_arr[max_index] < 0) grad_sign = -1.000;    // Moving against inc. grad: finding minima
  else if(LIA_delta_arr[max_index] >0) grad_sign = 1.000; // Moving along inc. grad: finding maxima
  else{
    Serial.println("########### zero coarse peak detected- stopping and waiting for reset");
    goHome_waitTillReset();
  }
  
  //To overwrite the arrays w/ overshoot steps
  //.....Not overwriting so that coarse-scan and go-back data can both be stored in matrix
//  arr_index = 0;

  //-------------------Sending the overshooted coarse peak-------------------

  //Finer resol for going back
  theta_resol = 2;
  phi_resol = 2;
  
  delay(delay_espnow); //delay b/w previous ESPNow sending
  sent_info.theta_resol = theta_resol;
  sent_info.phi_resol = phi_resol;
  sent_info.theta = theta_peakOvershoot;
  sent_info.phi = phi_peakOvershoot;
  Serial.println("\n~~~~~~~~~~~~~~~~bool_overshooted coarse peak: Going-back");
  
  send_data_wifi(sent_info); //Sending pt. and waiting till motors reach it
  Serial.println("reached coarse peak-reached by motors"); 

  //-------------------Converting 1D to 2D array-------------------
  //offset from coarse peak to reach the origin of fine scan area
  int theta_offset = int(round(float(theta_fine_scan_bound)/2));
  int phi_offset = int(round(float(phi_fine_scan_bound)/2));

  // Both global variables, to be accessed by functions
  theta_fineOrigin = theta_peakOvershoot - theta_offset;
  phi_fineOrigin = phi_peakOvershoot - phi_offset;

  // Filling all the array elements with NaN
  //Src_logic: https://www.techiedelight.com/initialize-matrix-cpp/
  //Src_nanSyntax: https://stackoverflow.com/a/1923933
  std::fill(*LIA_2d_arr, *LIA_2d_arr + theta_fine_scan_bound*phi_fine_scan_bound, NAN);

  // Filling coarse-scan and goto-peak data,in fine-scan box range, into 2d mat
  for(int i=0; i<arr_index; i++){
    if ((theta_rx_arr[i]>=theta_fineOrigin) && (theta_rx_arr[i]<=theta_fineOrigin + theta_fine_scan_bound - 1)){
      if ((phi_rx_arr[i]>=phi_fineOrigin) && (phi_rx_arr[i]<=phi_fineOrigin + phi_fine_scan_bound - 1)){
        // If coord. is repeated, then only the latest occuring voltage is stored at that coord.(theta,phi)
        LIA_2d_arr[int(theta_rx_arr[i]-theta_fineOrigin)][int(phi_rx_arr[i]-phi_fineOrigin)]= LIA_delta_arr[i];
      }
    }
  }

  //-------------------SGD begins-------------------
  //To overwrite the arrays w/ all the sgd + diff. coordinates  
  arr_index = 0;
  
  //Start storing all pts. in matrix
  store_in_mat = true;
  
  //Finer resol for SGD
  theta_resol = 2;
  phi_resol = 2;
  
  //To record data properly in python
  Serial.println("\n~~~~~~~~~~~~~~~~bool_SGD with differentiation begins");

  //~~~~~~~~~~~SGD starting point: coarse peak~~~~~~~~~~~
  float LIA_n = mat_offset_call(LIA_2d_arr,theta_peakOvershoot,phi_peakOvershoot);
  if(isnan(LIA_n)){
    Serial.println("########### Coarse peak NOT found in mat!!!- stopping and waiting for reset");
    goHome_waitTillReset();
  }
  int theta_n = theta_peakOvershoot;
  int phi_n = phi_peakOvershoot; 
   
  int theta_next,phi_next;
  float LIA_next, perc_inc;

  grad s_slope;
  for(int i=0; i<max_iterations; i++){
    //~~~~~~~~~~~Backward differentiation~~~~~~~~~~~
    s_slope = diff_2d(LIA_n,theta_n,phi_n);

    // For python plots: sending only SGD coord separatly
    Serial.println("pp-sgd " + String(LIA_n) 
                    + " " + String(theta_n) + " " + String(phi_n)
                    + " " + String(s_slope.s_theta) + " " + String(s_slope.s_phi));

    // For debugging
    Serial.println("LIA_n,th,phi,s_th,s_phi" + String(LIA_n) 
                    + " " + String(theta_n) + " " + String(phi_n)
                    + " " + String(s_slope.s_theta) + " " + String(s_slope.s_phi));                    

    //~~~~~~~~~~~Moving along gradient~~~~~~~~~~~
    theta_next = round_coord(float(theta_n) + alpha_theta*s_slope.s_theta,
                              theta_n, s_slope.s_theta);
    phi_next = round_coord(float(phi_n) + alpha_phi*s_slope.s_phi,
                              phi_n, s_slope.s_phi); 

    Serial.println("theta_next,phi_next = " + String(theta_next) + "," + String(phi_next));
    /*Sending the coord. to motor-ESP: going to next SGD*/
    delay(delay_espnow); //delay b/w previous ESPNow sending
    sent_info.theta_resol = theta_resol;
    sent_info.phi_resol = phi_resol;
    sent_info.theta = theta_next;
    sent_info.phi = phi_next;
    
    send_data_wifi(sent_info); //Sending pt. and waiting till motors reach it

    // All the motor data should be updated in mat. realtime(used callback fn.)
    LIA_next = mat_offset_call(LIA_2d_arr,theta_next,phi_next);
    if(isnan(LIA_next)){
      Serial.println("########### LIA_next NOT found in mat, (LIA_next from 1D_arr,theta_next,phi_next)=> "
                      + String(LIA_delta_arr[arr_index-1]) + ","
                      + String(theta_next) + "," + String(phi_next));
      Serial.println("- stopping and waiting for reset");                      
      goHome_waitTillReset();
    }
//    LIA_next = LIA_delta_arr[arr_index-1];

    //~~~~~~~~~~~Checking for convergance~~~~~~~~~~~
//    if(LIA_next > LIA_n){
    if(abs(LIA_n) > abs(LIA_next)){
//      perc_inc = 100*(LIA_n - LIA_next)/LIA_n;
//      Serial.println("Perc inc. in current SGD step wrt prev = " + String(perc_inc));

      //new formula to account for LIA_n = 0 case:
      perc_inc = 100*(LIA_n - LIA_next)/(LIA_n + LIA_next);
      Serial.println("Perc inc. in current SGD step wrt prev (new formula perc, og. perc) = "
                      + String(perc_inc) + "," + String(200*(perc_inc)/(100-perc_inc)));

//      if(perc_inc > convergance_perc){
      if(perc_inc > 100*convergance_perc/(200+convergance_perc)){// Changing limit acc. to new perc_inc formula
        Serial.println("Converged");

        /*Sending the coord. to motor-ESP: going to minima*/
        delay(delay_espnow); //delay b/w previous ESPNow sending
        sent_info.theta_resol = theta_resol;
        sent_info.phi_resol = phi_resol;
        sent_info.theta = theta_n;
        sent_info.phi = phi_n;
        send_data_wifi(sent_info); //Sending pt. and waiting till motors reach it

        break;
      }
    }

    //~~~~~~~~~~~Updation~~~~~~~~~~~
    theta_n = theta_next;
    phi_n = phi_next;
    LIA_n = LIA_next;
  }

  // Observes either the SGD converged pt. or the last SGD pt. after max_iterations
  Serial.println("\n~~~~~~~~~~~~~~~~bool_observe link");
  terminate_goHome = false; //Important as this will be "true" otherwise, due to program flow
//  attachInterrupt(BS_calib_button,isr_BS_calib,FALLING); 
  unsigned long obs_millis = millis();
  while(true){
    if(millis()-obs_millis >= link_obs_time*60*1000) break;

    // Stop observing abruptly and go back home 
    if(terminate_goHome){
      sent_info.theta_resol = theta_resol;
      sent_info.phi_resol = phi_resol;
      sent_info.theta = 0;
      sent_info.phi = 0;
      send_data_wifi(sent_info); //Sending pt. and waiting till motors reach it
      Serial.println("Went back to home (0,0)");

      Serial.println("sgd_coord-ended");  //To stop python code section
      digitalWrite(onboard_led,HIGH);

      break;
    }
    
    //Taking vg readings and printing for plotting
    LIA_op = roundf(analogReadAdjusted(ADC_ip_pin,5)*100)/100;  //Rounding to 2nd decimal    
    LIA_delta = LIA_op - LIA_bs;
    LIA_delta -= scan_dc_offset; //Offsetting the taken ADC readings to account for DC shift

    Serial.print(String(LIA_delta));
    Serial.println(" " + String(theta_rx_arr[arr_index-1])
                    + " " + String(phi_rx_arr[arr_index-1])
                    + " " + String(corner_reached));
    delay(obs_delay);
  }
  Serial.println("sgd_coord-ended");  //To stop python code section
  digitalWrite(onboard_led,HIGH);
  
  
  //-------------------Go Home-------------------
  //waiting here till go-home button pressed
  while(true){
    //Debounce switch source==> https://docs.arduino.cc/built-in-examples/digital/Debounce
    bool reading = digitalRead(BS_calib_button);
    
    if (reading != lastButtonState) {
      // reset the debouncing timer
      lastDebounceTime = millis();
    }
  
    if ((millis() - lastDebounceTime) > debounceDelay) {
      // whatever the reading is at, it's been there for longer than the debounce
      // delay, so take it as the actual current state:
  
      // if the button state has changed:
      if (reading != buttonState) {
        buttonState = reading;

        // Sending (0,0): home coordinates....only once
        if (buttonState == LOW) {
          sent_info.theta_resol = theta_resol;
          sent_info.phi_resol = phi_resol;
          sent_info.theta = 0;
          sent_info.phi = 0;
          send_data_wifi(sent_info); //Sending pt. and waiting till motors reach it
          Serial.println("Went back to home (0,0)");
          break;
        }
      }
  }
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;
  }

  //-------------------Waiting till hard reset-------------------
  Serial.println("indefinite waiting till hard reset");
  while(true){
      digitalWrite(onboard_led,HIGH);
      delay(1000);
      digitalWrite(onboard_led,LOW);
      delay(1000);
  }
}  

//==========Sending data via WiFi=================//
void send_data_wifi(corner_points_struct sent_info,bool wait_for_motors){
/*Sends data via WiFi, waits till motors reach the corner while polling data from it*/ 
/*......Default value of second argument = "True"....see fn. declaration before setup()-- Line 162 */ 
  String jsonString;                                    // Temporary storage for JSON String
  JsonObject object = doc_tx.to<JsonObject>();          // Create JSON object
  object["theta"] = sent_info.theta;
  object["phi"] = sent_info.phi;
  object["theta_resol"] = sent_info.theta_resol;
  object["phi_resol"] = sent_info.phi_resol;
  object["stop_scan"] = sent_info.stop_scan;
  object["server_reset"] = sent_info.server_reset;
  serializeJson(doc_tx, jsonString);                    // convert JSON object to string
  webSocket.sendTXT(jsonString); // Send data to the server
  
  if(sent_info.server_reset){
    Serial.println("########### Self reset after sending reset");
    delay(100);
    ESP.restart();
  }
  
  if(wait_for_motors){
    //waiting for the motors to reach the location
    while(true){
      webSocket.loop(); // Polling for data from motor-esp
      if(corner_reached) break;
    }
    corner_reached = false;
  }
}

//==========SGD Functions=================//
int round_coord(float coord_next,int coord_now,float s){
/*Forces the next SGD coordinate to be diff. from current step by rounding to next integer*/
  int rounded_coord = round(coord_next);

  if (rounded_coord == round(coord_now)){
    // Moving one step based on gradient dir.
    if (s<0){ 
      rounded_coord = rounded_coord - 1;
    }
    else if (s>0){
      rounded_coord = rounded_coord + 1;
    }
  }
  return rounded_coord;
}

float mat_offset_call(float mat[theta_fine_scan_bound][phi_fine_scan_bound],int x, int y){
/*Makes an axis transformation: linear translation of xy(in fine scan area) in matrix coord(with 0,0 origin)*/
  float mat_element = mat[x-theta_fineOrigin][y-phi_fineOrigin];

//  Serial.println("Recieved for mat_call: theta,phi " + String(x) + ","
//                  + String(y));
  
  if(x<theta_fineOrigin){
    Serial.print("########### Invalid mat coord, x<theta_fineOrigin, mat_ele==>");
    Serial.println(String(x) + " " + String(theta_fineOrigin) + " " + String(mat_element));
  }
  if(y<phi_fineOrigin){
    Serial.print("########### Invalid mat coord, y<phi_fineOrigin==>");
    Serial.println(String(y) + " " + String(phi_fineOrigin) + " " + String(mat_element));
  }

  return mat_element;
}


grad diff_2d(float Vnn, int theta_n, int phi_n){
/*Finds backward differentiation from given dataset(2d mat), moves motors otherwise*/

//  Serial.println("Recieved for diff: theta,phi " + String(theta_n) + ","
//                  + String(phi_n));
  grad s_slope;
  float temp1,temp2;
  int theta_p,phi_p;
  float Vpn,Vnp;

  int motor_mov_theta = 1; // +1 => motor moved, -1 => motor didn't move, i.e. already in dataset
  int motor_mov_phi = 1;
  
  //---For s_theta---
  
  bool got_flag = false;
  for(int j=1;j<diff_step+1; j++){
//    Serial.println("theta_diff_loop j = " + String(j));
    temp1 = mat_offset_call(LIA_2d_arr, theta_n-j, phi_n); //Searching backward
    temp2 = mat_offset_call(LIA_2d_arr, theta_n+j, phi_n); //Searching fwd

    if (!isnan(temp1) || !isnan(temp2)){
      //Store temp1 if both are not NAN
      got_flag = true;
      if (!isnan(temp1)){
        theta_p = theta_n - j;
        Vpn = temp1;
      }
      else{
        theta_p = theta_n + j;
        Vpn = temp2;
      }
      motor_mov_theta = -1;
      break;
    }
  }
  // If (theta_p,phi_n) not found in mat, move motors
  if (got_flag == false){
    theta_p = theta_n - diff_step;

    /*Sending the coord. to motor-ESP*/
    delay(delay_espnow); //delay b/w previous ESPNow sending
    sent_info.theta_resol = theta_resol;
    sent_info.phi_resol = phi_resol;
    sent_info.theta = theta_p;
    sent_info.phi = phi_n;
    
    send_data_wifi(sent_info); //Sending pt. and waiting till motors reach it

    // All the motor data should be updated in mat. realtime(used callback fn.)
    Vpn = LIA_delta_arr[arr_index-1];
  }

  if(theta_n == theta_p){
    Serial.println("########### Wrong theta differentiation: theta_n = theta_p = " + String(theta_p));
    if(got_flag == true){
      Serial.println("Motor was NOT moved for erronious theta_differentiation");
    }
    else{
      Serial.println("Motor was moved for erronious theta_differentiation");
    }
  }

  //---For s_phi---
  
  got_flag = false;
  for(int i=1; i<diff_step+1; i++){
    temp1 = mat_offset_call(LIA_2d_arr, theta_n, phi_n-i); //Searching backward
    temp2 = mat_offset_call(LIA_2d_arr, theta_n, phi_n+i); //Searching fwd

    if (!isnan(temp1) || !isnan(temp2)){
      //Store temp1 if both are not NAN
      got_flag = true;
      if (!isnan(temp1)){
        phi_p = phi_n - i;
        Vnp = temp1;
      }
      else{
        phi_p = phi_n + i;
        Vnp = temp2;
      }
      motor_mov_phi = -1;
      break;
    }
  }
  // If (theta_n,phi_p) not found in mat, move motors
  if (got_flag == false){
    phi_p = phi_n - diff_step;

    /*Sending the coord. to motor-ESP*/
    delay(delay_espnow); //delay b/w previous ESPNow sending
    sent_info.theta_resol = theta_resol;
    sent_info.phi_resol = phi_resol;
    sent_info.theta = theta_n;
    sent_info.phi = phi_p;
    
    send_data_wifi(sent_info); //Sending pt. and waiting till motors reach it

    // All the motor data should be updated in mat. realtime(used callback fn.)
    Vnp = LIA_delta_arr[arr_index-1];
  }

  if(phi_n == phi_p){
    Serial.println("########### Wrong phi differentiation: phi_n = phi_p = " + String(phi_p));
    if(got_flag == true){
      Serial.println("Motor was NOT moved for erronious phi_differentiation");
    }
    else{
      Serial.println("Motor was moved for erronious phi_differentiation");
    }
  }
  
  //---Backward differentiation---
  if(theta_n != theta_p){
//    s_slope.s_theta = -(Vnn-Vpn)/float(theta_n-theta_p);
    s_slope.s_theta = grad_sign*(Vnn-Vpn)/float(theta_n-theta_p);
    
    if(abs(s_slope.s_theta)==0){
      Serial.println("########### 0 theta slope: Vnn,Vpn,theta_n,theta_p,phi_n="
                      + String(Vnn) + "," +String(Vpn) + "," + String(theta_n) + ","
                      + String(theta_p) + "," + String(phi_n));
      Serial.println("Unrounded theta_next = " + String(float(theta_n)+alpha_phi*s_slope.s_theta));                      
    }
  }
  else{
    Serial.println("s_theta cant be found: since theta_n=theta_p");
  }

  if(phi_n != phi_p){
//    s_slope.s_phi = -(Vnn-Vnp)/float(phi_n-phi_p);
    s_slope.s_phi = grad_sign*(Vnn-Vnp)/float(phi_n-phi_p);
    
    if(abs(s_slope.s_phi)==0){
      Serial.println("########### 0 phi slope(Vnn=Vnp): Vnp,theta_n,phi_n,phi_p="
                      + String(Vnn) + "," + String(Vnp) + "," + String(theta_n) + ","
                      + String(phi_n) + "," + String(phi_p));
      Serial.println("Unrounded phi_next = " + String(float(phi_n)+alpha_phi*s_slope.s_phi));
    }
  }
  else{
    Serial.println("s_phi cant be found: since phi_n=phi_p");
  }

  Serial.println("pp-diff " + String(theta_n) + " " +String(phi_n) + " " 
                  + String(motor_mov_theta*theta_p) + " " + String(motor_mov_phi*phi_p) + " "
                  + String(Vnp) + " " + String(Vpn));
  return s_slope;
}

//==========Scan pattern Functions=================//
void do_scan(int theta_offset, int phi_offset){
/*Does scanning completly and then returns*/

  //loop runs till all the spiral-points are sent and motor reaches it
  while(true){
//    if(BS_calib_flag) LIA_bs = BS_calib(); //Caliberating ADC's baseline on button-press.
    
    corner_pts = spiralScan_thetaPhi(); //Spiral scanning

    //exiting if all corner points sent
    if(scan_done and (corner_pts.theta != theta_stop or corner_pts.phi != phi_stop)){
        Serial.println("first break");
        break;}

    //sending the corner pt.
    sent_info.theta_resol = theta_resol;
    sent_info.phi_resol = phi_resol;
    sent_info.theta = corner_pts.theta + theta_offset; //Shifting origin from (0,0) to (theta_offset,theta_offset)..
    sent_info.phi = corner_pts.phi + phi_offset;        //.. for fine scanning;
    
    send_data_wifi(sent_info); //Sending pt. and waiting till motors reach it
//    Serial.println("corner-reached by motors");

    //exiting, taking care of sp. case when at a valid point, spiral scan is completed
    if(scan_done){
      Serial.println("second break");
      break;}
  }
}


angles spiralScan_thetaPhi(){
/*Gives out corner pts. of spiral scanning, each fn. call returns a next corner pt.*/  
  local_counter++;
  counter++;
  touple_counter = ceil(float(counter)/4);

  angles corners;

  if (touple_counter == 1){
      if (local_counter == 1){ 
          theta = 0;
          phi = 0;

          corners.theta = theta;
          corners.phi = phi;
          
          theta_prev = theta;
          phi_prev = phi;

          return corners;} //compulsorily send first corner pt.
          
//Here on, just finding the current corner point, without sending        
      else if (local_counter == 2){
          theta = theta_max;
          phi = 0;}
          
      else if (local_counter == 3){
          theta = theta_max;
          phi = phi_max;}
          
      else if (local_counter == 4){
          theta = 0;
          phi = phi_max;
            
          local_counter = 0;}
  }        
  else if (touple_counter == 2){
      if (local_counter == 1){ 
          theta = 0;
          phi = phi_gap;
          
          if(theta>theta_max or phi>phi_max){
              scan_done = true; //if points invalid, come-out of loop---all corner pts sent!!
              Serial.println("scan-done changed to true: T2L1");
              return corners;} //invalid corner pts WON'T be sent to motor-ESP
          else{
              theta1 = theta;
              phi1 = phi;}}
          
      else if (local_counter == 2){
          theta = theta_max - theta_gap;
          phi = phi_gap;
          
          if(theta<0 or theta<theta_prev){
              scan_done = true; //if points invalid, come-out of loop---all corner pts sent!!
              Serial.println("scan-done changed to true: T2L2");
              return corners;} //invalid corner pts WON'T be sent to motor-ESP
          else{
              theta2 = theta;
              phi2 = phi;}}
          
      else if (local_counter == 3){
          theta = theta_max - theta_gap;
          phi = phi_max - phi_gap;
          
          if(phi<0 or phi<phi_prev){
              scan_done = true; //if points invalid, come-out of loop---all corner pts sent!!
              Serial.println("scan-done changed to true: T2L3");
              return corners;} //invalid corner pts WON'T be sent to motor-ESP
          else{
              theta3 = theta;
              phi3 = phi;}}
          
      else if (local_counter == 4){
          theta = theta_gap;
          phi = phi_max - phi_gap;
          
          if(phi<0 or phi<phi_prev){
              scan_done = true; //if points invalid, come-out of loop---all corner pts sent!!
              Serial.println("scan-done changed to true: T2L4");
              return corners;} //invalid corner pts WON'T be sent to motor-ESP
          else{
              theta4 = theta;
              phi4 = phi;}
              
          local_counter = 0;}
  }        
  else if(touple_counter>2){
      if (local_counter == 1){
          theta = theta1 + theta_gap;
          phi = phi1 + phi_gap;

          if(theta>theta_max or phi>phi_max or phi>phi_prev){
              scan_done = true; //if points invalid, come-out of loop---all corner pts sent!!
              Serial.println("scan-done changed to true: TnL1");
              return corners;} //invalid corner pts WON'T be sent to motor-ESP
          else{
              theta1 = theta;
              phi1 = phi;}}

      else if (local_counter == 2){
          theta = theta2 - theta_gap;
          phi = phi2 + phi_gap;

          if(theta<0 or theta<theta_prev){
              scan_done = true; //if points invalid, come-out of loop---all corner pts sent!!
              Serial.println("scan-done changed to true: TnL2");
              return corners;} //invalid corner pts WON'T be sent to motor-ESP
          else{
              theta2 = theta;
              phi2 = phi;}}

      else if (local_counter == 3){
          theta = theta3 - theta_gap;
          phi = phi3 - phi_gap;

          if(phi<0 or phi<phi_prev){
              scan_done = true; //if points invalid, come-out of loop---all corner pts sent!!
              Serial.println("scan-done changed to true: TnL3");
              return corners;} //invalid corner pts WON'T be sent to motor-ESP
          else{
              theta3 = theta;
              phi3 = phi;}}

      else if (local_counter == 4){
          theta = theta4 + theta_gap;
          phi = phi4 - phi_gap;

          if(theta>theta_max or theta>theta_prev){
              scan_done = true; //if points invalid, come-out of loop---all corner pts sent!!
              Serial.println("scan-done changed to true: TnL4");
              return corners;} //invalid corner pts WON'T be sent to motor-ESP
          else{
              theta4 = theta;
              phi4 = phi;}

          local_counter = 0;}    
  }   
  
//Checking the validity of theta and phi, returning them accordingly
  if(theta_prev == theta and phi_prev == phi){
        scan_done = true; //if points invalid, come-out of loop---all corner pts sent!!
        Serial.println("scan-done changed to true: end if");
        return corners;} //invalid corner pts WON'T be sent to motor-ESP
        
  else if(theta == theta_stop and phi == phi_stop){
      corners.theta = theta;
      corners.phi = phi;

      scan_done = true; //valid end reached, send and come out of loop
      Serial.println("scan-done changed to true: end elif");
      return corners;} 
      
  else{
      corners.theta = theta;
      corners.phi = phi;


      theta_prev = theta;
      phi_prev = phi;
      
      return corners;}  
}

//==========ADC Functions=================//
double BS_calib(){
    /*  Takes the LIA op reading when no LOS between tx-rx present, LIA o/p Baseline  */
    detachInterrupt(BS_calib_button); // To make sure no interrupt arrives while in ISR
    digitalWrite(onboard_led,HIGH);
    LIA_bs = analogReadAdjusted(ADC_ip_pin,100);
    LIA_bs = roundf(LIA_bs*100)/100;  //rounding to 2nd decimal place
    BS_calib_flag = false;
    digitalWrite(onboard_led,LOW);
    attachInterrupt(BS_calib_button,isr_BS_calib,FALLING);  // Re-enabling interrupt
    return LIA_bs;
}


double analogReadAdjusted(byte pinNumber,int ADC_samples_to_avg){
  /*  Reads ADC value by fitting a 12th order poly appx  */
  //Source: https://bitbucket.org/Blackneron/esp32_adc/src/master/ESP32_ADC/ESP32_ADC.ino
  
  // Specify the adjustment factors.
  const double f1 = 1.7111361460487501e+001;
  const double f2 = 4.2319467860421662e+000;
  const double f3 = -1.9077375643188468e-002;
  const double f4 = 5.4338055402459246e-005;
  const double f5 = -8.7712931081088873e-008;
  const double f6 = 8.7526709101221588e-011;
  const double f7 = -5.6536248553232152e-014;
  const double f8 = 2.4073049082147032e-017;
  const double f9 = -6.7106284580950781e-021;
  const double f10 = 1.1781963823253708e-024;
  const double f11 = -1.1818752813719799e-028;
  const double f12 = 5.1642864552256602e-033;

  // Oversampling
  const int loops = ADC_samples_to_avg;

  // Sampling time: Delay between the loops while compensating for oversampling to keep same effective req delay
  const unsigned long loopDelay = round(ADC_sampling_time/ADC_samples_to_avg);

  // Initialize the used variables.
  int counter = 1;
  int inputValue = 0;
  double totalInputValue = 0;
  double averageInputValue = 0;

  // Loop to get the average of different analog values.
  for (counter = 1; counter <= loops; counter++) {

    // Read the analog value.
    inputValue = analogRead(pinNumber);

    // Add the analog value to the total.
    totalInputValue += inputValue;

    // Wait some time after each loop.
    delayMicroseconds(loopDelay);
  }

  // Calculate the average input value.
  averageInputValue = totalInputValue / loops;

  // Calculate and return the adjusted input value.
  // ADC read 3.32V when i/p to ADC daughter board was 27V and an offset of 13.5V was given
  return ((3.3/4096)*(27/3.32)*(f1 + f2 * pow(averageInputValue, 1) + f3 * pow(averageInputValue, 2) + f4 * pow(averageInputValue, 3) + f5 * pow(averageInputValue, 4) + f6 * pow(averageInputValue, 5) + f7 * pow(averageInputValue, 6) + f8 * pow(averageInputValue, 7) + f9 * pow(averageInputValue, 8) + f10 * pow(averageInputValue, 9) + f11 * pow(averageInputValue, 10) + f12 * pow(averageInputValue, 11)))-13.5;
}

void goHome_waitTillReset(){
/*Called when a coord cant be found in mat: makes motors goto starting pos. (0,0) and waits till hard reset*/  
  //Blinking 5 times
  for(int i=1;i<=5;i++){
    digitalWrite(onboard_led,HIGH);
    delay(1000);
    digitalWrite(onboard_led,LOW);
    delay(2000);
  }

  //Returning motors to initial-pos.
  delay(delay_espnow); //delay b/w previous ESPNow sending
  sent_info.theta_resol = theta_resol;
  sent_info.phi_resol = phi_resol;
  sent_info.theta = 0;
  sent_info.phi = 0;
  
  send_data_wifi(sent_info); //Sending pt. and waiting till motors reach it   

  Serial.println("sgd_coord-ended");  //To stop python code section 

  //-------------------Waiting till hard reset-------------------
  Serial.println("indefinite waiting till hard reset");
  while(true){
      digitalWrite(onboard_led,HIGH);
      delay(1000);
      digitalWrite(onboard_led,LOW);
      delay(1000);
  }
}
