#include <SPI.h>

#define CS   5
#define SCK  18
#define MISO 19
#define LED  2
#define ADC  34
#define V_ref 2.54

unsigned int result = 0;
unsigned int double_byte = 0;
byte byte_H = 0;
byte byte_L = 0;

bool test_code = 0;

uint16_t ADC_data;
char serial_read;
int datatosend[16];
int i;


void setup() {
  Serial.begin(115200);
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  SPI.begin();

  pinMode(CS, OUTPUT);
  pinMode(SCK, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(ADC, INPUT);

  digitalWrite(CS, LOW);
  digitalWrite(CS, HIGH);
  digitalWrite(LED, HIGH);


}

void loop() {
/*
  if (Serial.available()){
    serial_read = Serial.read();

    if (serial_read == '1'){
      digitalWrite(LED, LOW);
      digitalWrite(CS, LOW);
      double_byte = SPI.transfer16(0x0000);
      digitalWrite(CS, HIGH);
      double_byte = ((double_byte << 3) >> 4) & 0b0000111111111111; 
      Serial.println(double_byte, BIN);
      //Serial.print('\r');

    } 
  }*/
  

  if (Serial.available()){
    serial_read = Serial.read();

    if (serial_read == '1'){
      Serial.println(analogRead(ADC));
    }
  }
  
  //Serial.println(analogRead(ADC));



}
