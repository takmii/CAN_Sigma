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

#define SUSP_PIN_RR A0      //Pino SUSP_REAR_RIGHT
#define SUSP_PIN_RL A1      //Pino SUSP_REAR_LEFT
#define STEERING_PIN A2     //Pino STEERING_WHEEL
#define ACCGYRO_SDA A4      //Pino ACC SDA
#define ACCGYRO_SCL A5      //Pino ACC SCL

#define TMR_BASE 100000                 //Clock base para os multiplicadores (100ms)
#define TMR_SUSP 100000               //Timer para gravar dados da suspensão (100ms)
#define TMR_STEERING 100000
#define TMR_ACC 100000                //Timer para gravar e enviar dados do acelerômetro 1  (100ms)
#define TMR_BLINK 100000                //Timer para piscar o led  (100ms)

#define Acc03_ID 0x05             //ID CAN Acelerometro 3
#define Gyro03_ID 0x06            //ID CAN Giroscopio 3
#define Susp_R_ID 0x08            //ID CAN Susp_Rear
#define Steering_ID 0x09          //ID CAN Steering_Wheel
#define ACK1_ID 0x0E              //ID CAN ACK1
#define ACK2_ID 0x0F              //ID CAN ACK2


MCP2515 mcp2515(CAN_CS);        //Declara uma estrutura de dados com base numa estrutura predefinida na biblioteca do Controlador CAN

struct can_frame Acc03;         //CAN MSG Acelerometro
struct can_frame Acc03;         //CAN MSG Acelerometro
struct can_frame Gyro03;        //CAN MSG Giroscopio
struct can_frame Susp_R;        //CAN MSG Susp_Front (Right & Left)
struct can_frame Steering;
struct can_frame ACK;           //CAN MSG Acknowledge

bool estadoLed=true;
bool Ack_Setup=false;

bool tmrAccGyro_Enable = false;
bool tmrAccGyro_Overflow = true;
char tmrAccGyro_Count = 0;

bool tmrSusp_Enable = true;
bool tmrSusp_Overflow = true;
char tmrSusp_Count = 0;

bool tmrSteering_Enable = true;
bool tmrSteering_Overflow = true;
char tmrSteering_Count = 0;

bool tmrBlink_Enable = false;
bool tmrBlink_Overflow = true;
char tmrBlink_Count = 0;

int Susp_RR_Ref=0;
int Susp_RL_Ref=0;
int Steering_Ref=0;



void can_Setup();
void Timer1_setup();
void taskScheduler();
void taskAccGyro();
void taskSusp();
void taskSteering();
void taskBlink();
void Setup_Ref2();
//void taskTest();

void setup() {
  Serial.begin(9600);
  SPI.begin();
  can_Setup();
  while (ACK.can_id!=ACK1_ID){           //Espera a resposta de ACK1 para a ECU 1
    mcp2515.readMessage(&ACK) ;
  }
  while (ACK.can_id!=ACK2_ID){
    Setup_Ref2();
    mcp2515.readMessage(&ACK) ;          // Remover quando a CAN estiver desconectada
  }
  Timer1_setup();
  Ack_Setup=true;
}

void loop() {
  if (Ack_Setup){
  taskAccGyro();
  taskSusp();
  taskSteering();
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
  Acc03.can_id=Acc03_ID;
  Acc03.can_dlc=6;
  Gyro03.can_id=Gyro03_ID;
  Gyro03.can_dlc=6;

  //Susp
  Susp_R.can_id=Susp_R_ID;
  Susp_R.can_dlc=4;

  //Steering Wheel
  Steering.can_id=Steering_ID;
  Steering.can_dlc=4;

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
  ///////////////  Rear Right  ///////////////

  int valueSusp_RR= 0;
  for (int i=0; i<5;i++){
   valueSusp_RR+= analogRead(SUSP_PIN_RR);
   //Serial.println(valueSusp_RR);
    }
    valueSusp_RR/=5;

  Susp_R.data[0]=valueSusp_RR &0xFF ;
  Susp_R.data[1]=(valueSusp_RR>>8) &0x03;

  ///////////////  Rear Left  ///////////////

  int valueSusp_RL= 0;
  for (int i=0; i<5;i++){
   valueSusp_RL+= analogRead(SUSP_PIN_RL);
   //Serial.println(valueSusp_RL);
    }
    valueSusp_RL/=5;

  Susp_R.data[2]=valueSusp_RL &0xFF ;
  Susp_R.data[3]=(valueSusp_RL>>8) & 0x03;

  mcp2515.sendMessage(&Susp_R);
  tmrSusp_Overflow=false;
  //taskTest();
  }
}

void taskSteering(){
  int valueSteering= 0;
  for (int i=0; i<3;i++){
   valueSteering+= analogRead(STEERING_PIN);
   //Serial.println(valueSteering);
    }
    valueSteering/=3;

  Steering.data[0]=valueSteering &0xFF ;
  Steering.data[1]=(valueSteering>>8) & 0x03;

  mcp2515.sendMessage(&Steering);
  tmrSteering_Overflow=false;
}

void Setup_Ref2(){
  //Suspensão
  Susp_RR_Ref=0;
  Susp_RL_Ref=0;
  for (int i=0; i<5; i++){
 Susp_RR_Ref+=analogRead(SUSP_PIN_RR);
 }
   for (int i=0; i<5; i++){
 Susp_RL_Ref+=analogRead(SUSP_PIN_RL);
 }
 Susp_RR_Ref/=5;
 Susp_RL_Ref/=5;
 Susp_R.data[0]=Susp_RR_Ref &0xFF ;
 Susp_R.data[1]=(Susp_RR_Ref>>8) &0x03;
 Susp_R.data[2]=Susp_RL_Ref &0xFF ;
 Susp_R.data[3]=(Susp_RL_Ref>>8) &0x03;
 
 mcp2515.sendMessage(&Susp_R);

 //Acelerometro 3

 //Steering Wheel
 Steering_Ref=0;
 for (int i=0; i<3;i++){
   Steering_Ref+= analogRead(STEERING_PIN);
    }
  Steering_Ref/=3;
  Steering.data[0]=Steering_Ref &0xFF ;
  Steering.data[1]=(Steering_Ref>>8) & 0x03;

  mcp2515.sendMessage(&Steering);

}

void taskBlink(void)
{
  if (tmrBlink_Overflow)
  {
    digitalWrite(LED_CPU, estadoLed);
    estadoLed!= estadoLed;
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
   if (tmrSteering_Enable){
    tmrSteering_Count++;
    if (tmrSteering_Count>=(TMR_STEERING/TMR_BASE)){
      tmrSteering_Count=0;
      tmrSteering_Overflow=true;
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
