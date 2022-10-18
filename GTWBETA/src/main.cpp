#include <Arduino.h>
#include <SPI.h>
#include <stdio.h>
#include <EEPROM.h>
#include <mcp2515.h>
#include <TimerOne.h>
#include <SD.h>

#define TMR_BASE 100000
#define TMR_TEMP1 300000
#define TMR_TEMP2 300000
#define TMR_BLINK 100000
#define TMR_SD 100000

#define LED_CPU 8            //Porta para o LED do módulo
#define CAN_SCK 13          //Pino SCK da CAN
#define CAN_SO 12           //Pino SO da CAN
#define CAN_SI 11           //Pino SI da CAN
#define CAN_CS 10           //Pino CS da CAN
#define TEMP1_PIN A3
#define TEMP2_PIN A4

#define Acc01_ID 0x01             //ID CAN Acelerometro 1
#define Gyro01_ID 0x02            //ID CAN Giroscopio 1
#define Acc02_ID 0x03             //ID CAN Acelerometro 2
#define Gyro02_ID 0x04            //ID CAN Giroscopio 2
#define Acc03_ID 0x05             //ID CAN Acelerometro 3
#define Gyro03_ID 0x06            //ID CAN Giroscopio 3
#define Susp_F_ID 0x07            //ID CAN Susp_Frontal
#define Susp_R_ID 0x08            //ID CAN Susp_Rear
#define Steering_ID 0x09          //ID CAN Steering_Wheel
#define ACK1_ID 0x0E              //ID CAN ACK1
#define ACK2_ID 0x0F              //ID CAN ACK2


MCP2515 mcp2515(CAN_CS);

bool Ack_Setup=false;
File Data;
String FileName="";

String Temp1_data="";
String Temp2_data="";

String SuspFR_data="";
String SuspFL_data="";
String SuspRR_data="";
String SuspRL_data="";
String Steering_data="";

bool tmrTemp1_Enable = true;
bool tmrTemp1_Overflow = true;
char tmrTemp1_Count = 0;

bool tmrTemp2_Enable = true;
bool tmrTemp2_Overflow = true;
char tmrTemp2_Count = 0;

bool tmrCAN_Enable = true;
bool tmrCAN_Overflow = true;
char tmrCAN_Count = 0;

bool tmrSD_Enable = true;
bool tmrSD_Overflow = true;
char tmrSD_Count = 0;

bool tmrBlink_Enable = false;
bool tmrBlink_Overflow = true;
char tmrBlink_Count = 0;

bool SuspF_OK=false;
bool SuspR_OK=false;
bool Steering_OK=false;

bool SF=false;        //ACK SuspFront
bool SR=false;        //ACK SuspRear
bool SW=false;        //ACK SteeringWheel

bool SD_Error;

uint16_t ID_aux=0;

struct can_frame canMsg;
struct can_frame ACK1;
struct can_frame ACK2;

int Susp_FR_Ref=0;
int Susp_FL_Ref=0;
int Susp_RR_Ref=0;
int Susp_RL_Ref=0;
int Steering_Ref=0;

void Timer1_setup();
void fn_Temp1();
void fn_Temp2();
void Setup_SD_CARD();
void SD_CARD();
void CAN_Setup();
void CAN_Read();
void taskScheduler();
void fn_Susp_F();
void fn_Susp_R();
void fn_Steering();
void Setup_ECU1();
void Setup_ECU2();

void setup() {
  Serial.begin(9600);
  CAN_Setup();
  Setup_SD_CARD();
  Setup_ECU1();
  Setup_ECU2();
  Timer1_setup();
  Ack_Setup=true;
}

void loop() {
  if(Ack_Setup){
    fn_Temp1();
    fn_Temp2();
    CAN_Read();   
    SD_CARD();
}
}

void Timer1_setup() {
  Timer1.initialize(TMR_BASE);
  Timer1.attachInterrupt(taskScheduler);
}

void Setup_ECU1(){  
  while (SF!=true)
  {
    mcp2515.readMessage(&canMsg);
    switch (canMsg.can_id){
      case Susp_F_ID:
        Susp_FR_Ref=(canMsg.data[1]<<8)+canMsg.data[0];
        Susp_FL_Ref=(canMsg.data[3]<<8)+canMsg.data[2];
        SF=true;
      break;
  }
  }
  ACK1.data[0]=1;
  mcp2515.sendMessage(&ACK1);
}
void Setup_ECU2(){
  while (SR!=true && SW!=true){
    mcp2515.readMessage(&canMsg);
    switch (canMsg.can_id){
    case Susp_R_ID:
        Susp_RR_Ref=(canMsg.data[1]<<8)+canMsg.data[0];
        Susp_RL_Ref=(canMsg.data[3]<<8)+canMsg.data[2];
        SR=true; 
      break;

      case Steering_ID:
        Steering_Ref=(canMsg.data[1]<<8)+canMsg.data[0];;
        SW=true; 
      break;
  }
}
  ACK2.data[0]=1;
  mcp2515.sendMessage(&ACK2);
}


///////////////////////////////////////////////             FUNÇÃO SENSORES             ///////////////////////////////////////////////

void fn_Temp1()
{
  if(tmrTemp1_Overflow){
float Temp1;
uint16_t pot= analogRead(TEMP1_PIN);
Temp1= (pow(float(pot),0.218898)/0.012149)-11.18-273; //Temperatura em ºC
Temp1_data= String(Temp1);
Temp1_data.concat(",");
Serial.print("Temperatura Intercooler : ");
Serial.println(Temp1);
}
tmrTemp1_Overflow=false;
}

void fn_Temp2()
{
  if(tmrTemp2_Overflow){
float Temp2;
uint16_t pot= analogRead(TEMP2_PIN);
Temp2= (pow(float(pot),1.054362)/11.11938)-14; //Temperatura em ºC
Temp2_data= String(Temp2);
Temp2_data.concat(",");
Serial.print("Temperatura TBI : ");
Serial.println(Temp2);
Serial.println(" ");

}
tmrTemp2_Overflow=false;
}


void fn_Susp_F(){
 float SuspFR = (float((canMsg.data[1]<<8)+canMsg.data[0])-float(Susp_FR_Ref))*90/1023;
 float SuspFL = (float((canMsg.data[3]<<8)+canMsg.data[2])-float(Susp_FL_Ref))*90/1023;
 SuspFR_data= String(SuspFR);
 SuspFL_data= String(SuspFL);
  if (SuspF_OK=false)
 {
  SuspFR_data="NaN";
  SuspFL_data="NaN";
 }
 SuspFR_data.concat(",");
 SuspFL_data.concat(",");

ID_aux=canMsg.can_id;
SuspF_OK=false;
}

void fn_Susp_R(){
 float SuspRR = (float((canMsg.data[1]<<8)+canMsg.data[0])-float(Susp_RR_Ref))*90/1023;
 float SuspRL = (float((canMsg.data[3]<<8)+canMsg.data[2])-float(Susp_RL_Ref))*90/1023;
 SuspRR_data= String(SuspRR);
 SuspRL_data= String(SuspRL);
  if (SuspR_OK=false)
 {
  SuspRR_data="NaN";
  SuspRL_data="NaN";
 }
 SuspRR_data.concat(",");
 SuspRL_data.concat(",");
 
ID_aux=canMsg.can_id;
SuspR_OK=false;
}

void fn_Steering(){
 float SWvalue = (float((canMsg.data[1]<<8)+canMsg.data[0])-float(Steering_Ref))*90/1023;
 
 Steering_data= String(SWvalue);
  if (Steering_OK=false)
 {
  Steering_data="NaN";
  Steering_data="NaN";
 }
 Steering_data.concat(",");
 Steering_data.concat(",");
 
ID_aux=canMsg.can_id;
Steering_OK=false;

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////             FUNÇÃO SD CARD             ///////////////////////////////////////////////

void Setup_SD_CARD(){
int testNumber = EEPROM.read(0x00);
  FileName = "test";
  FileName.concat(testNumber);
  FileName.concat(".csv");
  EEPROM.write(0x00, testNumber+1);
  while (!Serial){
  }
  
  if (!SD.begin(4)){
    Serial.println ("init_fail");
    SD_Error=true;
  }
  else{
    SD_Error=false;
  }
  Data=SD.open(FileName, FILE_WRITE);
  if(Data){
    Data.println("Temp Intercooler(ºC),Temp TBI(ºC),Susp_Front_Right(º),Susp_Front_Left(º),Susp_Rear_Right(º),Susp_Rear_Left(º),Steering_Wheel,Time(s)");
    Data.close();
  }
}

void SD_CARD()
{
  if (!SD_Error){
    if (tmrSD_Overflow)
    {
  Data=SD.open(FileName, FILE_WRITE);
  Serial.print("");
  Data.close();

    Data=SD.open(FileName, FILE_WRITE);
//_SENSORES
    Data.print(Temp1_data);
    Data.print(Temp2_data);
    Data.print(SuspFR_data);
    Data.print(SuspFL_data);
    Data.print(SuspRR_data);
    Data.print(SuspRL_data);
    Data.println(millis());

    Data.close();
}
}
    tmrSD_Overflow=false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CAN_Setup(){
  digitalWrite(LED_CPU, HIGH);
  digitalWrite(LED_CPU, LOW);

  ACK1.can_id = ACK1_ID;
  ACK1.can_dlc=1;

  ACK2.can_id = ACK2_ID;
  ACK2.can_dlc=1;

  SPI.begin();                                               //Inicia a comunicação SPI
  mcp2515.reset();                                           //Reset do Controlador CAN atraves da SPI do Arduino
  mcp2515.setBitrate(CAN_500KBPS,MCP_8MHZ);                  //Configura a velocidade de comunicação CAN para 500KBPS com um Clock de 8MHz. Clock do cristal do Controlador MCP2515
  mcp2515.setNormalMode();                                   //Configura o modo normal
}

void CAN_Read(){
if (mcp2515.readMessage(&canMsg)==MCP2515::ERROR_OK){
  if (uint16_t(canMsg.can_id)!=ID_aux){
  switch(canMsg.can_id){
    case Acc01_ID:

    break;
    case Susp_F_ID:
      SuspF_OK=true;
      fn_Susp_F();
    break;
    case Susp_R_ID:
      SuspR_OK=true;
      fn_Susp_R();
    break;
    case Steering_ID:
      Steering_OK=true;
      fn_Steering();
  }
}
}
}\

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void taskScheduler(){
if (tmrSD_Enable) { 
    tmrSD_Count++;
    if (tmrSD_Count>=(TMR_SD/TMR_BASE)){
      tmrSD_Count=0;
      tmrSD_Overflow=true;
    }
  }
  if (tmrTemp1_Enable){
    tmrTemp1_Count++;
    if (tmrTemp1_Count>=(TMR_TEMP1/TMR_BASE)){
      tmrTemp1_Count=0;
      tmrTemp1_Overflow=true;
    }
  }

    if (tmrTemp2_Enable){
    tmrTemp2_Count++;
    if (tmrTemp2_Count>=(TMR_TEMP2/TMR_BASE)){
      tmrTemp2_Count=0;
      tmrTemp2_Overflow=true;
    }
  }
  
  if (tmrBlink_Enable){
    tmrBlink_Count++;
    if (tmrBlink_Count>=(TMR_BLINK/TMR_BASE)){
      tmrBlink_Count=0;
      tmrBlink_Overflow=true;
    }
  }
  else{
    tmrBlink_Count++;
    if (tmrBlink_Count>=10*(TMR_BLINK/TMR_BASE)){
      tmrBlink_Count=0;
      tmrBlink_Overflow=true;
    }
  }
  }



