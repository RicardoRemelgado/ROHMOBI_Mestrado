//Inclusão das bibliotecas 
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//Istanciamento de objetos 
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); 

//Declaração de funções 
void writeServos(int nServo, int posicao);
void beginServos(); 
float DegServo_p1(float degree);
float DegServo_t2(float degree);
  

//             0   1   2  3   4   5    6  7   8   9  10 11 12  13  14
int fPos[] = {-6, 10, 3, -9, 5, -3, -3, 9, -3, 12, 8, 9, 7, -5, -2};

void setup(){
  beginServos(); 
  delay(300);
}

float DegServo_p1(float degree){
  return(degree + 90);
}

float DegServo_t2(float degree){
  return(-(degree)+135);
}
float ConvertLeg2(float degree){
  return(180-degree);
}

void loop(){
  //Initial stance
  //1st module
  writeServos(12, DegServo_p1(0 + fPos[12]));
    //Leg1
    writeServos(0, DegServo_t2(39.34 + fPos[0]));
    writeServos(1, 93.31+ fPos[1]);
    //Leg2
    writeServos(2, ConvertLeg2(DegServo_t2(39.34 + fPos[2])));
    writeServos(3, ConvertLeg2(93.31 + fPos[3]));
  //2nd module
  writeServos(13, DegServo_p1(0 + fPos[13]));
    //Leg1
    writeServos(4, DegServo_t2(39.34 + fPos[4]));
    writeServos(5, 93.31 + fPos[5]);
    //Leg2
    writeServos(6, ConvertLeg2(DegServo_t2(39.34 + fPos[6])));
    writeServos(7, ConvertLeg2(93.31 + fPos[7]));
  //3rd module
  writeServos(14, DegServo_p1(0 + fPos[14]));
    //Leg1
    writeServos(8, DegServo_t2(39.34 + fPos[8]));
    writeServos(9, 93.31 + fPos[9]);
    //Leg2
    writeServos(10, ConvertLeg2(DegServo_t2(39.34 + fPos[10])));
    writeServos(11, ConvertLeg2(93.31 + fPos[11]));
    /*
  writeServos(0, 90);
  writeServos(1, 0);
  writeServos(2, ConvertLeg2(90));
  writeServos(3, ConvertLeg2(0));
  writeServos(4, 0);
  writeServos(5, 0);
  writeServos(6, ConvertLeg2(90));
  writeServos(7, ConvertLeg2(0));
  writeServos(8, 90);
  writeServos(9, 0);
  writeServos(10, ConvertLeg2(90));
  writeServos(11, ConvertLeg2(0));
  writeServos(12, 90);
  writeServos(13, 90);
  writeServos(14, 90);*/
  delay(50);
}


// Funções
void writeServos(int nServo, int posicao) {
#define SERVOMIN  115 
#define SERVOMAX  530

  int pos = map ( posicao , 0 , 180 , SERVOMIN, SERVOMAX);
  pwm.setPWM(nServo , 0, pos);
}

void beginServos() {

#define frequence 50 // VALOR DA FREQUENCIA DO SERVO 

  pwm.begin(); // INICIA O OBJETO PWM
  pwm.setPWMFreq(frequence); // DEFINE A FREQUENCIA DE TRABALHO DO SERVO
}
