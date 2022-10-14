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
#define ACCGYRO_SDA A4      //Pino ACC SDA
#define ACCGYRO_SCL A5      //Pino ACC SCL

#define TMR_BASE 100000                 //Clock base para os multiplicadores (100ms)
#define TMR_SUSP 100000               //Timer para gravar dados da suspensão (100ms)
#define TMR_ACC 100000                //Timer para gravar e enviar dados do acelerômetro 1  (100ms)
#define TMR_BLINK 100000                //Timer para piscar o led  (100ms)

#define Acc03_ID 0x03             //ID CAN Accelerometro 3
#define Gyro03_ID 0x04            //ID CAN Giroscopio 3
#define Susp_F_ID 0x06            //ID CAN Susp_Rear

MCP2515 mcp2515(CAN_CS);        //Declara uma estrutura de dados com base numa estrutura predefinida na biblioteca do Controlador CAN

struct can_frame Acc03;         //CAN MSG Acelerometro
struct can_frame Gyro03;        //CAN MSG Giroscopio
struct can_frame Susp_R;       //CAN MSG Susp_Front (Right & Left)

bool estadoLed=true;

bool tmrAccGyro_Enable = false;
bool tmrAccGyro_Overflow = true;
char tmrAccGyro_Count = 0;

bool tmrSusp_Enable = true;
bool tmrSusp_Overflow = true;
char tmrSusp_Count = 0;

bool tmrBlink_Enable = false;
bool tmrBlink_Overflow = true;
char tmrBlink_Count = 0;



void can_Setup();
void Timer1_setup();
void taskScheduler();
void taskAccGyro();
void taskSusp();
void taskBlink();
//void taskTest();

void setup() {
  //Serial.begin(9600);
  SPI.begin();
  can_Setup();
  Timer1_setup();
}

void loop() {
  taskAccGyro();
  taskSusp();
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
  Acc03.can_id=Acc03_ID;
  Acc03.can_dlc=6;
  Gyro03.can_id=Gyro03_ID;
  Gyro03.can_dlc=6;

  //Susp
  Susp_R.can_id=Susp_F_ID;
  Susp_R.can_dlc=4;

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
  int valueSusp_RR= analogRead(SUSP_PIN_RR);
  Susp_R.data[0]=valueSusp_RR &0xFF ;
  Susp_R.data[1]=(valueSusp_RR>>8) &0x03;


  
  int valueSusp_RL= analogRead(SUSP_PIN_RL);
  Susp_R.data[2]=valueSusp_RL &0xFF ;
  Susp_R.data[3]=(valueSusp_RL>>8) & 0x03;

  mcp2515.sendMessage(&Susp_R);
  tmrSusp_Overflow=false;
  //taskTest();
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
