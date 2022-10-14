//ECU 01 - Susp Frontal e Acelerometro

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <TimerOne.h>

#define LED_CPU 8            //Porta para o LED do módulo
#define CAN_SCK 13          //Pino SCK da CAN
#define CAN_SO 12           //Pino SO da CAN
#define CAN_SI 11           //Pino SI da CAN
#define CAN_CS 10           //Pino CS da CAN

#define SUSP_PIN_FR 4      //Pino SUSP_FRONT_RIGHT
#define SUSP_PIN_FL 1      //Pino SUSP_FRONT_LEFT
#define TEMP2_PIN 3        //Pino Temperatura 2
//#define ACCGYRO_SDA 4      //Pino ACC SDA
#define ACCGYRO_SCL 5      //Pino ACC SCL

#define TMR_BASE 100000                 //Clock base para os multiplicadores (100ms)
#define TMR_SUSP 100000               //Timer para gravar dados da suspensão (100ms)
#define TMR_ACC 100000                //Timer para gravar e enviar dados do acelerômetro 1  (100ms)
#define TMR_BLINK 100000                //Timer para piscar o led  (100ms)
#define TMR_TEMP2 100000

#define Temp2_ID 0x00             //ID CAN Temperatura 2
#define Acc02_ID 0x01             //ID CAN Accelerometro
#define Gyro02_ID 0x02            //ID CAN Giroscopio
#define Susp_F_ID 0x05            //ID CAN Susp_Frontal

MCP2515 mcp2515(CAN_CS);        //Declara uma estrutura de dados com base numa estrutura predefinida na biblioteca do Controlador CAN

struct can_frame Temp2;         //CAN MSG Temperatura 2
struct can_frame Acc02;         //CAN MSG Acelerometro
struct can_frame Gyro02;        //CAN MSG Giroscopio
struct can_frame Susp_F;       //CAN MSG Susp_Front (Right & Left)
struct can_frame Test_recieve;

bool estadoLed=true;

bool tmrAccGyro_Enable = false;
bool tmrAccGyro_Overflow = true;
char tmrAccGyro_Count = 0;

bool tmrTemp2_Enable = true;
bool tmrTemp2_Overflow = true;
char tmrTemp2_Count = 0;

bool tmrSusp_Enable = true;
bool tmrSusp_Overflow = true;
char tmrSusp_Count = 0;

bool tmrBlink_Enable = false;
bool tmrBlink_Overflow = true;
char tmrBlink_Count = 0;

//Falcatrua
int valueSusp_FR= 0;
int valueSusp_FL= 0;



void can_Setup();
void Timer1_setup();
void taskScheduler();
void taskAccGyro();
void taskSusp();
void taskTemp2();
void taskBlink();
//void taskTest();

void setup() {
  Serial.begin(9600);
  SPI.begin();
  can_Setup();
  Timer1_setup();
}

void loop() {
  taskAccGyro();
  taskSusp();
  taskTemp2();
  taskBlink();

}

void Timer1_setup(){
  Timer1.initialize(TMR_BASE);
  Timer1.attachInterrupt(taskScheduler);
}

void can_Setup(){
  digitalWrite(LED_CPU, HIGH);
  digitalWrite(LED_CPU, LOW);

  //Acelerometro & Giroscopio
  Acc02.can_id=Acc02_ID;
  Acc02.can_dlc=6;
  Gyro02.can_id=Gyro02_ID;
  Gyro02.can_dlc=6;

  //Susp
  Susp_F.can_id=Susp_F_ID;
  Susp_F.can_dlc=4;

  //Temp2
  Temp2.can_id=Temp2_ID;
  Temp2.can_dlc=2;

  SPI.begin();                                               //Inicia a comunicação SPI
  mcp2515.reset();                                           //Reset do Controlador CAN atraves da SPI do Arduino
  mcp2515.setBitrate(CAN_500KBPS,MCP_8MHZ);                  //Configura a velocidade de comunicação CAN para 500KBPS com um Clock de 8MHz. Clock do cristal do Controlador MCP2515
  mcp2515.setNormalMode();                                   //Configura o modo normal
}

void taskAccGyro(){
  tmrAccGyro_Overflow=0;
}

void taskSusp(){
  if(tmrSusp_Overflow){
  valueSusp_FR= 0;
    for (int i=0; i<5;i++){
   valueSusp_FR+= analogRead(SUSP_PIN_FR);
   delay(2);
   //Serial.println(valueSusp_FR);
    }
    valueSusp_FR=valueSusp_FR/5;
  Susp_F.data[0]=valueSusp_FR &0xFF ;
  Susp_F.data[1]=(valueSusp_FR>>8) &0x03;
    Serial.print("FR ");
  Serial.println(valueSusp_FR);

  valueSusp_FL=0;
  for (int i=0; i<5;i++){
   valueSusp_FL+= analogRead(SUSP_PIN_FL);
   delay(2);
      //Serial.println(valueSusp_FL);
    }
    valueSusp_FL=valueSusp_FL/5;
  //Serial.print("FL ");
  //Serial.println(valueSusp_FL);
  Susp_F.data[2]=valueSusp_FL &0xFF ;
  Susp_F.data[3]=(valueSusp_FL>>8) & 0x03;

  mcp2515.sendMessage(&Susp_F);
  tmrSusp_Overflow=false;
  //taskTest();
  }
}

void taskTemp2(){
  if(tmrTemp2_Overflow){
    int valueTemp2=0;
  for (int i=0; i<5;i++){
  valueTemp2+=analogRead(TEMP2_PIN);
  delay(2);
     //Serial.println(valueTemp2);
  }
  valueTemp2=(valueTemp2/5);
  Temp2.data[0]=valueTemp2 &0xFF ;
  Temp2.data[1]=(valueTemp2>>8) & 0x03;
  Serial.print("Temp ");
  Serial.println(valueTemp2);
  mcp2515.sendMessage(&Temp2);
  tmrTemp2_Overflow=false;
  valueTemp2=0;
  }

}

void taskBlink(void)
{
  if (tmrBlink_Overflow)
  {
    //digitalWrite(LED_CPU, estadoLed);
    //estadoLed!= estadoLed;
    tmrBlink_Overflow = false;
    tmrBlink_Enable=false;
  }
}

/*void taskTest(){
  if(mcp2515.readMessage(&Test_recieve)==MCP2515:: ERROR_OK){
    Serial.print("test ");
    int penis= Test_recieve.data[1]<<8;
    penis+=Test_recieve.data[0];
    Serial.println(penis);
  }

}*/



void taskScheduler(){

  if (tmrAccGyro_Enable) { 
    tmrAccGyro_Count++;
    if (tmrAccGyro_Count>=(TMR_ACC/TMR_BASE)){
      tmrAccGyro_Count=0;
      tmrAccGyro_Overflow=true;
    }
  }
  if (tmrSusp_Enable){
    tmrSusp_Count++;
    if (tmrSusp_Count>=(TMR_SUSP/TMR_BASE)){
      tmrSusp_Count=0;
      tmrSusp_Overflow=true;
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
