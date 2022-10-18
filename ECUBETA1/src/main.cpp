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

#define SUSP_PIN_FR A0      //Pino SUSP_FRONT_RIGHT
#define SUSP_PIN_FL A1      //Pino SUSP_FRONT_LEFT
#define ACCGYRO_SDA A4      //Pino ACC SDA
#define ACCGYRO_SCL A5      //Pino ACC SCL

#define TMR_BASE 100000                 //Clock base para os multiplicadores (100ms)
#define TMR_SUSP 100000               //Timer para gravar dados da suspensão (100ms)
#define TMR_ACC 100000                //Timer para gravar e enviar dados do acelerômetro 1  (100ms)
#define TMR_BLINK 100000                //Timer para piscar o led  (100ms)

#define Acc01_ID 0x01             //ID CAN Acelerometro 1
#define Gyro01_ID 0x02            //ID CAN Giroscopio 1
#define Acc02_ID 0x03             //ID CAN Acelerometro 2
#define Gyro02_ID 0x04            //ID CAN Giroscopio 2
#define Susp_F_ID 0x07            //ID CAN Susp_Frontal
#define ACK1_ID 0x0E              //ID CAN ACK1
#define ACK2_ID 0x0F              //ID CAN ACK2

MCP2515 mcp2515(CAN_CS);        //Declara uma estrutura de dados com base numa estrutura predefinida na biblioteca do Controlador CAN

struct can_frame Temp2;         //CAN MSG Temperatura 2
struct can_frame Acc01;         //CAN MSG Acelerometro 1
struct can_frame Gyro01;        //CAN MSG Giroscopio 1
struct can_frame Acc02;         //CAN MSG Acelerometro 2
struct can_frame Gyro02;        //CAN MSG Giroscopio 2
struct can_frame Susp_F;        //CAN MSG Susp_Front (Right & Left)
struct can_frame ACK;           //CAN MSG Acknowledge
//struct can_frame Test_recieve;

bool estadoLed=true;
bool Ack_Setup=false;

bool tmrAccGyro1_Enable = false;
bool tmrAccGyro1_Overflow = true;
char tmrAccGyro1_Count = 0;

bool tmrSusp_Enable = true;
bool tmrSusp_Overflow = true;
char tmrSusp_Count = 0;

bool tmrBlink_Enable = false;
bool tmrBlink_Overflow = true;
char tmrBlink_Count = 0;

int Susp_FR_Ref=0;
int Susp_FL_Ref=0;


void can_Setup();
void Timer1_setup();
void taskScheduler();
void taskAccGyro1();
void taskSusp();
void taskBlink();
void Setup_Ref1();
//void taskTest();

void setup() {
  Serial.begin(9600);
  SPI.begin();
  can_Setup();

  while (ACK.can_id!=ACK1_ID){
    Setup_Ref1();
    mcp2515.readMessage(&ACK) ;           // Remover quando a CAN estiver desconectada
  }
  while (ACK.can_id!=ACK2_ID){            //Espera a resposta de ACK2 para a ECU 2
    mcp2515.readMessage(&ACK) ;
  }
  Timer1_setup();
  Ack_Setup=true;
}

void loop() {
  if (Ack_Setup){
  taskAccGyro1();
  taskSusp();
  taskBlink();
  }
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

  SPI.begin();                                               //Inicia a comunicação SPI
  mcp2515.reset();                                           //Reset do Controlador CAN atraves da SPI do Arduino
  mcp2515.setBitrate(CAN_500KBPS,MCP_8MHZ);                  //Configura a velocidade de comunicação CAN para 500KBPS com um Clock de 8MHz. Clock do cristal do Controlador MCP2515
  mcp2515.setNormalMode();                                   //Configura o modo normal
}

void taskAccGyro1(){
  tmrAccGyro1_Overflow=0;
}

void taskSusp(){
  if(tmrSusp_Overflow){
  ///////////////  Front Right  ///////////////
  
  int valueSusp_FR= 0;
  for (int i=0; i<5;i++){
   valueSusp_FR+= analogRead(SUSP_PIN_FR);
   //Serial.println(valueSusp_FR);
    }
   valueSusp_FR/=5;

  Susp_F.data[0]=valueSusp_FR &0xFF ;
  Susp_F.data[1]=(valueSusp_FR>>8) &0x03;
    Serial.print("FR ");
  Serial.println(valueSusp_FR);

  ///////////////  Front Left  ///////////////

  int valueSusp_FL=0;
  for (int i=0; i<5;i++){
   valueSusp_FL+= analogRead(SUSP_PIN_FL);
      //Serial.println(valueSusp_FL);
    }
    valueSusp_FL/=5;

  Serial.print("FL ");
  Serial.println(valueSusp_FL);
  Serial.println(" ");
  Susp_F.data[2]=valueSusp_FL &0xFF ;
  Susp_F.data[3]=(valueSusp_FL>>8) & 0x03;

  mcp2515.sendMessage(&Susp_F);
  tmrSusp_Overflow=false;
  //delay(500); //teste
  //taskTest();
  }
}

void Setup_Ref1(){
  //Suspensão
  Susp_FR_Ref=0;
  Susp_FL_Ref=0;
  for (int i=0; i<5; i++){
 Susp_FR_Ref+=analogRead(SUSP_PIN_FR);
 }
   for (int i=0; i<5; i++){
 Susp_FL_Ref+=analogRead(SUSP_PIN_FL);
 }
 Susp_FR_Ref/=5;
 Susp_FL_Ref/=5;
 Susp_F.data[0]=Susp_FR_Ref &0xFF ;
 Susp_F.data[1]=(Susp_FR_Ref>>8) &0x03;
 Susp_F.data[2]=Susp_FL_Ref &0xFF ;
 Susp_F.data[3]=(Susp_FL_Ref>>8) &0x03;
 
 mcp2515.sendMessage(&Susp_F);

 //Acelerometro 1

 //Acelerometro 2
 
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
    int test= Test_recieve.data[1]<<8;
    test+=Test_recieve.data[0];
    Serial.println(test);
  }

}*/



void taskScheduler(){

  if (tmrAccGyro1_Enable) { 
    tmrAccGyro1_Count++;
    if (tmrAccGyro1_Count>=(TMR_ACC/TMR_BASE)){
      tmrAccGyro1_Count=0;
      tmrAccGyro1_Overflow=true;
    }
  }
  if (tmrSusp_Enable){
    tmrSusp_Count++;
    if (tmrSusp_Count>=(TMR_SUSP/TMR_BASE)){
      tmrSusp_Count=0;
      tmrSusp_Overflow=true;
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
