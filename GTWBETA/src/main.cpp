#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <stdio.h>
#include <EEPROM.h>
#include <mcp2515.h>
#include <TimerOne.h>

#define TMR_BASE 100000
#define TMR_TEMP1 100000
#define TMR_TEMP2 100000
#define TMR_BLINK 100000
#define TMR_SD 100000

#define LED_CPU 8            //Porta para o LED do módulo
#define CAN_SCK 13          //Pino SCK da CAN
#define CAN_SO 12           //Pino SO da CAN
#define CAN_SI 11           //Pino SI da CAN
#define CAN_CS 10           //Pino CS da CAN
#define TEMP1_PIN A3

#define Temp2_ID 0x00             //ID CAN Temperatura 2
#define Acc01_ID 0x01             //ID CAN Accelerometro 2
#define Gyro01_ID 0x02            //ID CAN Giroscopio 2
#define Acc01_ID 0x03             //ID CAN Accelerometro 3
#define Gyro01_ID 0x04            //ID CAN Giroscopio 3
#define Susp_F_ID 0x05            //ID CAN Susp_Front
#define Susp_R_ID 0x06            //ID CAN Susp_Rear

MCP2515 mcp2515(CAN_CS);

bool start=0;
File Data;
String FileName="";

String Temp1_data="";
String Temp2_data="";

String SuspFR_data="";
String SuspFL_data="";
String SuspRR_data="";
String SuspRL_data="";

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
bool Temp2_OK=false;

bool SD_Error;

uint16_t ID_aux=0;

struct can_frame canMsg;

void Timer1_setup();
void fn_Temp1();
void fn_Temp2();
void Setup_SD_CARD();
void SD_CARD();
void CAN_Setup();
void CAN_Read();
void taskScheduler();
void fn_Susp_F();

void setup() {
  Serial.begin(9600);
  CAN_Setup();
  Timer1_setup();
  Setup_SD_CARD();

  start=1;
}

void loop() {
  if(start){
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


///////////////////////////////////////////////             FUNÇÃO SENSORES             ///////////////////////////////////////////////

void fn_Temp1()
{
  if(tmrTemp1_Overflow){
float Temp1;
uint16_t pot= analogRead(TEMP1_PIN);
Temp1= (pow(float(pot),1.054362)/11.11938)-14; //Temperatura em ºC
Temp1_data= String(Temp1);
Temp1_data.concat(",");

}
tmrTemp1_Overflow=false;
}

void fn_Temp2()
{
 float Temp2 = (pow(float((canMsg.data[1]<<8)+canMsg.data[0]),1.054362)/11.11938)-14; //Temperatura em ºC
 Temp2_data= String(Temp2);
  if (Temp2_OK=false)
 {
  Temp2_data="NaN";
 }
 Temp2_data.concat(",");

ID_aux=canMsg.can_id;
Temp2_OK=false;
}

void fn_Susp_F(){
 float SuspFR = float((canMsg.data[1]<<8)+canMsg.data[0])*90/1023;
 float SuspFL = float((canMsg.data[3]<<8)+canMsg.data[2])*90/1023;
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
 float SuspRR = float((canMsg.data[1]<<8)+canMsg.data[0])*90/1023;
 float SuspRL = float((canMsg.data[3]<<8)+canMsg.data[2])*90/1023;
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
    Data.println("Temp1(ºC),Temp2(ºC),Susp_Front_Right(º),Susp_Front_Left(º),Susp_Rear_Right(º),Susp_Rear_Left(º),Time(s)");
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
    case Temp2_ID:
      Temp2_OK=true;
      fn_Temp2();
    break;
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



