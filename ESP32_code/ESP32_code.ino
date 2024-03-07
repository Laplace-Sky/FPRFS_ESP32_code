#include <WiFi.h>
#include <HardwareSerial.h>

#define dataPin 4
#define clockPin 2
#define lclockPin 0
#define PB1 5
#define PB2 18
#define PB3 19
#define OPAMP_EN_pos 32
#define OPAMP_EN_neg 33
#define LED1 23
#define LED2 22

HardwareSerial uart2(2);

//const char* ssid = "TPG08CB";
//const char* password = "meimimaa";

const char* ssid = // enter the ssid
const char* password =  // enter the password
int board_num = 1;
int region_num = 8;
int data_num = 32;
int i = 0;
int j = 0;
int m = 0;

int serial_bytecount = 0;
int shift_bytecount = 0;

bool wifi_connected = false;
bool continue_without_wifi = false;

bool boardnum_saved = false;

byte bitstream[32] = {};            // 8 bytes (64 bits) array for single board stub pattern

bool configured = false;    //whether pattern has been completely received from uart2
bool continuous_shift = false;     //set configured_flag to true to continuously shift out data;




WiFiServer server(80);

void IRAM_ATTR ISR_PB1() {
  digitalWrite(OPAMP_EN_neg, LOW);    //PB1 for Non-inverting-polarity (opamp is disabled)

}

void IRAM_ATTR ISR_PB2() {
  digitalWrite(OPAMP_EN_neg, HIGH);   //PB2 for Inverting-polarity (opamp is enabled, -3.3V is enabled)

}

void IRAM_ATTR ISR_PB3() {
  wifi_connected = false;
  continue_without_wifi = false;
  digitalWrite(LED2, LOW);
}

String translateEncryptionType(wifi_auth_mode_t encryptionType) {
 
  switch (encryptionType) {
    case (WIFI_AUTH_OPEN):
      return "Open";
    case (WIFI_AUTH_WEP):
      return "WEP";
    case (WIFI_AUTH_WPA_PSK):
      return "WPA_PSK";
    case (WIFI_AUTH_WPA2_PSK):
      return "WPA2_PSK";
    case (WIFI_AUTH_WPA_WPA2_PSK):
      return "WPA_WPA2_PSK";
    case (WIFI_AUTH_WPA2_ENTERPRISE):
      return "WPA2_ENTERPRISE";
  }
}

void scanNetworks() {
 
  int numberOfNetworks = WiFi.scanNetworks();
 
  Serial.print("Number of networks found: ");
  Serial.println(numberOfNetworks);
 
  for (int i = 0; i < numberOfNetworks; i++) {
 
    Serial.print("Network name: ");
    Serial.println(WiFi.SSID(i));
 
    Serial.print("Signal strength: ");
    Serial.println(WiFi.RSSI(i));
 
    Serial.print("MAC address: ");
    Serial.println(WiFi.BSSIDstr(i));
 
    Serial.print("Encryption type: ");
    String encryptionTypeDescription = translateEncryptionType(WiFi.encryptionType(i));
    Serial.println(encryptionTypeDescription);
    Serial.println("-----------------------");
 
  }
}
 
void connectToNetwork() {
  int n = 0;
  
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Establishing connection to WiFi..");
    
    n++;
    
    if(n >= 5){
      Serial.println("Try to reconnect...");
      break;
    }
  }

  if(WiFi.status() == WL_CONNECTED){
    wifi_connected = true;
    m = 0;
    
    Serial.println("Connected to network");

    Serial.println("Mac address: ");
    Serial.println(WiFi.macAddress());
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    digitalWrite(LED2, HIGH);
  
    Serial.println();
  
    server.begin();
    
    Serial.println("Server has started...");
    
    Serial.println();
  } 
}

void setup() {
 
  Serial.begin(115200);
  uart2.begin(115200, SERIAL_8N1, 16, 17);
  Serial.println("ESP32 Serial port 2 configured!");
  Serial.println();
  
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(lclockPin, OUTPUT);
  pinMode(PB1, INPUT);
  pinMode(PB2, INPUT);
  pinMode(PB3, INPUT);
  pinMode(OPAMP_EN_pos, OUTPUT);
  pinMode(OPAMP_EN_neg, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  for(i = 0; i < data_num; i++){          //initialize the bitstream array
    bitstream[i] = 0;
  }

  GPIO.out_w1ts |= (1ULL<<lclockPin);
  //digitalWrite(LED2, continuous_shift);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);

  attachInterrupt(PB1, ISR_PB1, FALLING);
  attachInterrupt(PB2, ISR_PB2, FALLING);
  attachInterrupt(PB3, ISR_PB3, FALLING);

  
  scanNetworks();

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  while((wifi_connected == false) & (continue_without_wifi == false)){
    connectToNetwork();

    m++;

    if(m >= 3){
      m = 0;

      Serial.println();
      Serial.println("WiFi is not available, press S3 button to retry, otherwise continue without WiFi...");
      Serial.println();
      continue_without_wifi = true;         //stop trying to connect to Wifi
      
      Serial.println("Continue without WiFi...");
      
      Serial.println();
      Serial.println();
      break;
    }
  }


  /*
  WiFi.disconnect(true);
  Serial.println(WiFi.localIP());
  */

}
 
void loop(){
  scanNetworks();
  check_for_reconnection();
  
  read_RF_pattern_wifi(bitstream);
  read_RF_pattern_serial(bitstream);

  //shift out the received pattern configuration
  if(configured){
    config_RF_pattern(bitstream);
    if(continuous_shift == false){
      configured = false;                
    }
  }
}

void check_for_reconnection(){
  while((wifi_connected == false) & (continue_without_wifi == false)){
    connectToNetwork();

    m++;

    if(m >= 3){
      m = 0;

      Serial.println();
      Serial.println("WiFi is not available, press S3 button to retry, otherwise continue without WiFi...");
      Serial.println();
      continue_without_wifi = true;         //stop trying to connect to Wifi
      
      Serial.println("Continue without WiFi...");
      
      Serial.println();
      Serial.println();
      break;
    }
  }
}

void read_RF_pattern_wifi(byte Data[]){
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("New Client.");          // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        board_num = client.read(); 
        data_num = board_num * region_num;
        Serial.println("Number of boards activated:");
        Serial.println(board_num);
        Serial.println();
        for(i = 0; i < data_num; i++){                       //i counts the received byte number
          Data[i] = client.read();                         
        }
        
        for(j = 0; j < data_num; j++){
          Serial.println(Data[j]); 
        }
        Serial.println("Data received through WiFi.");
        
        configured = true;                     //flag denoting all 8 bytes are received
        break;
      }
    }
    // close the connection:
    client.stop();
    Serial.println("Client Disconnected.");
    Serial.println();

    digitalWrite(LED1, HIGH);
  }
}

void read_RF_pattern_serial(byte Data[]){
  if(uart2.available() > 0){
    //Serial.println(serial_bytecount);
    if (!boardnum_saved){
      //Serial.println("Incoming new bitstream...");
      board_num = uart2.read();
      data_num = board_num * region_num;
      //Serial.println(data_num);
      //Serial.println("Number of boards activated:");
      //Serial.println(board_num);
      //Serial.println();
      digitalWrite(LED2, LOW);
      boardnum_saved = true;
    } else if(serial_bytecount != data_num){  
      Data[serial_bytecount] = uart2.read();                //save the recevied byte
      Serial.print(Data[serial_bytecount]); 
      Serial.print(" ");
      serial_bytecount++;                                   //increase byte number counter
    } 

    if (serial_bytecount == data_num){                       //i counts the received byte number
      serial_bytecount = 0;
      uart2.read();                         //dump the last byte ('/n': ASCII code 10)
      configured = true;                     //flag denoting all 8 bytes are received
      boardnum_saved = false;

      Serial.println();
      Serial.println("Data received through serial port.");

      //digitalWrite(LED1, LOW);
      digitalWrite(LED2, HIGH);
                                       //clear byte counter i for next receiving
    } 
  }
}


void config_RF_pattern(byte Data[]){
  GPIO.out_w1tc |= (1ULL<<lclockPin);        //set the latch clock pin to be LOW: starting the bit shifting operation 
  delayMicroseconds(100);
  for(shift_bytecount = data_num - 1; shift_bytecount >= 0; shift_bytecount--){
    shift_fast(Data[shift_bytecount]);                       //shift last region first
  }
  GPIO.out_w1ts |= (1ULL<<lclockPin);        //set the latch clock pin to be HIGH: ending the bit shifting operation

  Serial.println("Shift is done.");
  Serial.println();
  Serial.println();
}

void shift_fast(byte data){
  shiftOut(dataPin, clockPin, LSBFIRST, data);     //LSB first so the bit order in each data is 12345678 (from left to right)
}
 
