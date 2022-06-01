#include <SPI.h>

#define encoder0PinA  5
#define encoder0PinB  3
#define PI 3.1415926535897932384626433832795
#define CAN_2515
// #define CAN_2518FD

// Set SPI CS Pin according to your hardware
// For Arduino MCP2515 Hat:
// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;
const int CAN_INT_PIN = 2;

#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
#endif


volatile long encoder0Pos=0;
volatile long newposition;
volatile long oldposition = 0;
long newtime;
long oldtime = 0;
// max PPR of this motor
int maxPPR = 1000;
// radius of the wheel
float wheel_radius = 0.285;
// gear ratio from the motor
float gear_ratio = 1;
long frequency;
long RPM;
signed long veh_speed;


void setup() { //Setup runs once//
  
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinA, HIGH);
  digitalWrite(encoder0PinB, HIGH);
  attachInterrupt(encoder0PinA, doEncoderA, CHANGE); //Interrupt trigger mode: RISING
  attachInterrupt(encoder0PinB, doEncoderB, CHANGE); //Interrupt trigger mode: RISING
  // eliminate the motor part when using the atlascar2

  SERIAL_PORT_MONITOR.begin(115200);
  while(!Serial){};
//
  while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
      SERIAL_PORT_MONITOR.println("CAN init fail, retry...");
      delay(100);
  }
  SERIAL_PORT_MONITOR.println("CAN init ok!");
}


byte signed stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
void loop() { //Loop runs forever//
   // I will only know if this is right after checking with the wheel encoder
   newposition = encoder0Pos;
   newtime = millis();
   if (newtime - oldtime >= 10) {
     // pulses per second
     
     SERIAL_PORT_MONITOR.print ("position = ");
     SERIAL_PORT_MONITOR.println (newposition);
     oldposition = newposition;
     oldtime = newtime;

     // Send velocity to the CAN Bus 
     stmp[0] = (newposition >> 56);
     stmp[1] = (newposition >> 48);
     stmp[2] = (newposition >> 40);
     stmp[3] = (newposition >> 32);
     stmp[4] = (newposition >> 24);
     stmp[5] = (newposition >> 16);
     stmp[6] = (newposition >> 8);
     stmp[7] = newposition;
     long newLong = (stmp[4] << 24) | (stmp[5] << 16) | (stmp[6] << 8) | (stmp[7]);
     Serial.println(newLong);
     Serial.print(stmp[4],HEX);
     Serial.print(" ");
     Serial.print(stmp[5],HEX);
     Serial.print(" ");
     Serial.print(stmp[6],HEX);
     Serial.print(" ");
     Serial.println(stmp[7],HEX);
     CAN.sendMsgBuf(0x500, 0, 8,stmp);
     SERIAL_PORT_MONITOR.println("CAN BUS sendMsgBuf ok!");
    } 
}

void doEncoderA()
{
  if (digitalRead(encoder0PinA) != digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
}

void doEncoderB()
{
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
}
