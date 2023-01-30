#include <Servo.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <MPU9255.h> 

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
MPU9255 mpu;

//Function declaration
void writeServos(int nServo, int posicao);
void beginServos();
float DegServo_p1(float degree);
float DegServo_t2(float degree);
float ConvertLeg2(float degree);
float Convert2rad(float degree);
float Convert2deg(float radian);
float TimeStep(float T);
void ForKin(float phi1, float theta2, float theta3);
///
void FirstSwing(float t);
void FirstSupport(float t);
void SwingPhase(float t);
void SupportPhase(float t);
void StancePhase(float t);

//Define global variables
  //Dimensions of the legs in mm
  float l1 = 50.9;
  float l2 = 58.0;
  //list of control points of Bezier Curve in the first swing
  float P1stswing [4][3] = {{0, 0, -90},
                           {0, 0, -45},
                           {50.9, 0, -58.1},
                           {36, 0, -90}};
//list of control points of Bezier Curve in the first swing
  float P1stsup [3][3] = {{0, 0, -90},
                         {-18, 0, -95},
                         {-36, 0, -90}};
  //List of control joint angles of Bezier Curve in swing phase
  float angle = 0, prev_angle = 0;
  float A[4][3]; 
  //List to save calculated control points of Bezier Curve in swing phase                  
  float P[4][3];
  //List of control points of Bezier Curve in swing phase
  float Psup[3][3] = {{36, 0, -90},
                      {0, 0, -95},
                      {-36, 0, -90}};
  //Generic spatial coordinates
  float x, y, z;  
  float r ;
  //Spatial coordinates from forward kinematics
  float x_fk, y_fk, z_fk;
  //Previous plane coordinates
  float x_prev, y_prev;
  //Generic joint angles 
  float phi1_rad, phi1;
  float alpha2, beta2, theta2; 
  float beta3, theta3;    
  //Angles to send to servos
  float firstswing_p1 = 0, firstswing_t2, firstswing_t3 = 0;
  float firstsup_p1 = 0, firstsup_t2, firstsup_t3 = 0;
  float swing_p1 = 0, swing_t2 = 0, swing_t3 = 0;
  float sup_p1 = 0, sup_t2 = 0, sup_t3 = 0;
  float stance_p1, stance_t2, stance_t3; 
  //Period of a total step
  float T = 2; // default
  //Manual calibration of servos position
//             0   1   2  3   4   5    6  7   8   9  10 11 12  13  14
int fPos[] = {-6, 10, 3, -9, 5-12, -3+2, -3, 9, -3, 12, 8, 9, 7, -5, -2};
  //Mover of phases
  int incomingByte = 0; //s=115, w=119, d=100, a=97, f=102;
  int i = 0;
  //Commands:
    //w = 119: avance
    //s = 115: stop
    //a = 97: turn left
    //d = 100: turn right
    //f = 102: finish 
    //r = 114: restart, returns to stance position   
  //  w   r       119     114
  //a s d f    97 115 100 102
  
// The controller will be updated at a rate of 100Hz
#define UPDATE_FREQUENCY 100
#define UPDATE_TIME (1000 / UPDATE_FREQUENCY)
unsigned long updateTimer = 0;

//IMU variables
float gyro_x_cal;
float gyro_y_cal;
int gyroX;
int gyroY;
float accX_cal;
float accY_cal;
float accZ_cal;
float accX;
float accY;

void setup() {
  Serial.begin(115200);
  
  while(!Serial);
  //Serial.println("Starting program");

  if(mpu.init())
  {
  //Serial.println("IMU Failed initialization");
  incomingByte = 102;
  }
  else
  {
  //Serial.println("IMU Successful initialization");
  }

  beginServos();
  delay(300);
  
//Initial stance
  //1st module
  writeServos(12, DegServo_p1(0 + fPos[12]));
    //Leg1
    writeServos(0, DegServo_t2(52.98 + fPos[0]));
    writeServos(1, 68.85+ fPos[1]);
    //Leg2
    writeServos(2, ConvertLeg2(DegServo_t2(52.98 + fPos[2])));
    writeServos(3, ConvertLeg2(68.85 + fPos[3]));
  //2nd module
  writeServos(13, DegServo_p1(0 + fPos[13]));
    //Leg1
    writeServos(4, DegServo_t2(52.98 + fPos[4]));
    writeServos(5, 68.85 + fPos[5]);
    //Leg2
    writeServos(6, ConvertLeg2(DegServo_t2(52.98 + fPos[6])));
    writeServos(7, ConvertLeg2(68.85 + fPos[7]));
  //3rd module
  writeServos(14, DegServo_p1(0 + fPos[14]));
    //Leg1
    writeServos(8, DegServo_t2(52.98 + fPos[8]));
    writeServos(9, 68.85 + fPos[9]);
    //Leg2
    writeServos(10, ConvertLeg2(DegServo_t2(52.98 + fPos[10])));
    writeServos(11, ConvertLeg2(68.85 + fPos[11]));

  // Get data from IMU (2000x) to obtain an average of the values and subtract the error
  for (int i=0; i<2000; i++){   
    mpu.set_gyro_scale(scale_250dps);
    mpu.read_gyro();//get data from the gyroscope
    gyro_x_cal += (float(mpu.gx)/131);
    gyro_y_cal += (float(mpu.gy)/131);
    accX_cal += float(mpu.ax) /8192 ;
    accY_cal += float(mpu.ay) /8192 ;
    accZ_cal += float(mpu.az) /8192 ;
  }
  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;
  accX_cal /= 2000;
  accY_cal /= 2000;
  accZ_cal /= 2000;
  
  updateTimer = millis();
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

float DegServo_p1(float degree){
  return(degree + 90);
}

float DegServo_t2(float degree){
  return(-(degree)+135);
}
float ConvertLeg2(float degree){
  return(180-degree);
}

float Convert2rad(float degree)
{
    return(PI * (degree /180));
}

float Convert2deg(float radian)
{
    return(radian * (180 / PI));
}

float TimeStep(float T){
  float it, ts;
  it = T / 0.02;
  ts = pow(it, -1);
  return ts;
}

void ForKin(float phi1, float theta2, float theta3){
  x_fk = cos(Convert2rad(phi1))*(l2*cos(Convert2rad(theta2+theta3))+l1*cos(Convert2rad(theta2)));
  y_fk = sin(Convert2rad(phi1))*(l2*cos(Convert2rad(theta2+theta3))+l1*cos(Convert2rad(theta2)));
  z_fk = l2*sin(Convert2rad(theta2+theta3))+l1*sin(Convert2rad(theta2));
}

void ControlPoints(float angle){
  float A[4][3] = {{(0*(angle/3)), 82.57, 54.56},
            {(1*(angle/3)), 15.71, 131.79},
            {(2*(angle/3)), 0, 90},
            {(3*(angle/3)), 38.97, 54.56}};
  for (int i = 0; i<4; i++){
    ForKin(A[i][0], A[i][1], A[i][2]);
      P[i][0] = {x_fk};
      P[i][1] = {y_fk};
      P[i][2] = {z_fk};
  }
  x_prev = P[0][0];
  y_prev = P[0][1];
}
///
void FirstSwing(float t){
  x = pow((1 - t), 3) * P1stswing[0][0] + 3 * pow((1 - t), 2) * t * P1stswing[1][0] + 3 * (1 - t) * pow(t, 2) * P1stswing[2][0] + pow(t, 3) * P1stswing[3][0];
  y = pow((1 - t), 3) * P1stswing[0][1] + 3 * pow((1 - t), 2) * t * P1stswing[1][1] + 3 * (1 - t) * pow(t, 2) * P1stswing[2][1] + pow(t, 3) * P1stswing[3][1];
  z = pow((1 - t), 3) * P1stswing[0][2] + 3 * pow((1 - t), 2) * t * P1stswing[1][2] + 3 * (1 - t) * pow(t, 2) * P1stswing[2][2] + pow(t, 3) * P1stswing[3][2];  
  r = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
    if( x != 0 && y !=0 ){
      phi1_rad = atan(y/x);
      }
    else{ 
      phi1_rad = 0;
      }
  alpha2 = Convert2deg(acos((x / cos(phi1_rad)) / r));
  beta2 = Convert2deg(acos((pow(l1 , 2) + pow(r, 2) - pow(l2, 2)) / (2 * l1 * r)));
  theta2 = alpha2 - beta2;
  beta3 = Convert2deg(acos((pow(l1, 2) + pow(l2, 2) - pow(r ,2)) / (2 * l1 * l2)));
  theta3 = 180 - beta3;
  //store in global variables
  firstswing_p1 = Convert2deg(phi1_rad);
  firstswing_t2 = theta2;
  firstswing_t3 = theta3;
  //Prints 
  /*Serial.print("First Swing: [ ");
  Serial.print(firstswing_p1);
  Serial.print(", ");
  Serial.print(firstswing_t2);
  Serial.print(", ");
  Serial.print(firstswing_t3);
  Serial.println(" ]"); */
}
void FirstSupport(float t){
  x = pow((1 - t), 2) * P1stsup[0][0] + 2 * (1 - t) * t * P1stsup[1][0] + pow(t, 2) * P1stsup[2][0];
  y = pow((1 - t), 2) * P1stsup[0][1] + 2 * (1 - t) * t * P1stsup[1][1] + pow(t, 2) * P1stsup[2][1];
  z = pow((1 - t), 2) * P1stsup[0][2] + 2 * (1 - t) * t * P1stsup[1][2] + pow(t, 2) * P1stsup[2][2];
  r = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
    if( x != 0 && y !=0 ){
      phi1_rad = atan(y/x);
      }
    else{ 
      phi1_rad = 0;
      }
  alpha2 = Convert2deg(acos((x / cos(phi1_rad)) / r));
  beta2 = Convert2deg(acos((pow(l1 , 2) + pow(r, 2) - pow(l2, 2)) / (2 * l1 * r)));
  theta2 = alpha2 - beta2;
  beta3 = Convert2deg(acos((pow(l1, 2) + pow(l2, 2) - pow(r ,2)) / (2 * l1 * l2)));
  theta3 = 180 - beta3;  
  //store in global variables
  firstsup_p1 = Convert2deg(phi1_rad);
  firstsup_t2 = theta2;
  firstsup_t3 = theta3;
  //Prints
  /*Serial.print("First Support: [ ");
  Serial.print(firstsup_p1);
  Serial.print(", ");
  Serial.print(firstsup_t2);
  Serial.print(", ");
  Serial.print(firstsup_t3);
  Serial.println(" ]");*/
}

void SwingPhase(float t){
  x = pow((1 - t), 3) * P[0][0] + 3 * pow((1 - t), 2) * t * P[1][0] + 3 * (1 - t) * pow(t, 2) * P[2][0] + pow(t, 3) * P[3][0];
  y = pow((1 - t), 3) * P[0][1] + 3 * pow((1 - t), 2) * t * P[1][1] + 3 * (1 - t) * pow(t, 2) * P[2][1] + pow(t, 3) * P[3][1];
  z = pow((1 - t), 3) * P[0][2] + 3 * pow((1 - t), 2) * t * P[1][2] + 3 * (1 - t) * pow(t, 2) * P[2][2] + pow(t, 3) * P[3][2];  
  r = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
    if( x != 0 && y != 0 ){
      phi1_rad = atan((y - P[0][1]) / (x - P[0][0]));
      }
    else{ 
      phi1_rad = 0;
      }
  x_prev = x;
  y_prev = y;
  alpha2 = Convert2deg(acos((x / cos(phi1_rad)) / r));
  beta2 = Convert2deg(acos((pow(l1 , 2) + pow(r, 2) - pow(l2, 2)) / (2 * l1 * r)));
  theta2 = alpha2 - beta2;
  beta3 = Convert2deg(acos((pow(l1, 2) + pow(l2, 2) - pow(r ,2)) / (2 * l1 * l2)));
  theta3 = 180 - beta3;
  //store in global variables
  if ( incomingByte == 100){
    swing_p1 =  2 * Convert2deg(phi1_rad) - 0.05; //empirical transformation
  }
  else if ( incomingByte == 97){
    swing_p1 =  2 * Convert2deg(phi1_rad) + 0.05; //empirical transformation
  }
  else{
    swing_p1 = Convert2deg(phi1_rad);
  }
  swing_t2 = theta2;
  swing_t3 = theta3;
  //Prints
  /*Serial.print("Swing: [ ");
  Serial.print(swing_p1);
  Serial.print(", ");
  Serial.print(swing_t2);
  Serial.print(", ");
  Serial.print(swing_t3);
  Serial.println(" ]");*/
}

void SupportPhase(float t){
  x = pow((1 - t), 2) * Psup[0][0] + 2 * (1 - t) * t * Psup[1][0] + pow(t, 2) * Psup[2][0];
  y = pow((1 - t), 2) * Psup[0][1] + 2 * (1 - t) * t * Psup[1][1] + pow(t, 2) * Psup[2][1];
  z = pow((1 - t), 2) * Psup[0][2] + 2 * (1 - t) * t * Psup[1][2] + pow(t, 2) * Psup[2][2];
  r = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
    if( x != 0 && y !=0 ){
      phi1_rad = atan(y/x);
      }
    else{ 
      phi1_rad = 0;
      }
  alpha2 = Convert2deg(acos((x / cos(phi1_rad)) / r));
  beta2 = Convert2deg(acos((pow(l1 , 2) + pow(r, 2) - pow(l2, 2)) / (2 * l1 * r)));
  theta2 = alpha2 - beta2;
  beta3 = Convert2deg(acos((pow(l1, 2) + pow(l2, 2) - pow(r ,2)) / (2 * l1 * l2)));
  theta3 = 180 - beta3;  

  //store in global variables
  sup_p1 = Convert2deg(phi1_rad);
  sup_t2 = theta2;
  sup_t3 = theta3;
  //Prints
  /*Serial.print("Support: [ ");
  Serial.print(sup_p1);
  Serial.print(", ");
  Serial.print(sup_t2);
  Serial.print(", ");
  Serial.print(sup_t3);
  Serial.println(" ]");*/
}

void StancePhase(){
  stance_p1 = 0;
  stance_t2 = 52.98;
  stance_t3 = 68.85;
}

void loop(){
  if (millis() - updateTimer >= UPDATE_TIME){
    updateTimer += UPDATE_TIME;
    if (Serial.available() > 0) {
      // read the incoming byte:
      incomingByte = Serial.read();
    }
  //IMU readings
  mpu.set_acc_scale(scale_4g);
  mpu.read_acc();//get data from the accelerometer 

  float accX_raw = (float(mpu.ax) /8192  - accX_cal) * 9.81;
  float accY_raw = (float(mpu.ay) /8192  - accY_cal) * 9.81;
  float accZ_raw = (float(mpu.az) /8192  - accZ_cal) * 9.81;

  accX = (atan((accY_raw)/sqrt(pow(accX_raw,2)+pow(accZ_raw,2)))) * (180/PI);
  accY = (atan((accX_raw)/sqrt(pow(accY_raw,2)+pow(accZ_raw,2)))) * (180/PI);

  if (millis()>5000){
    mpu.set_gyro_scale(scale_250dps);
    mpu.read_gyro();//get data from the gyroscope
    gyroX =  ((mpu.gx/131-gyro_x_cal));// * (float(interval)/1000);
    gyroY = ((mpu.gy/131-gyro_y_cal)) ;//* (float(interval)/1000);
  }
  
    //Zero step
    if ( i == 0 && incomingByte == 119){
      for (float t=0; t<1; t=t+TimeStep(T)){
        FirstSwing(t);
        FirstSupport(t);
          //1st module
        writeServos(0, DegServo_t2(firstsup_t2));
        writeServos(1, firstsup_t3);
        writeServos(2, ConvertLeg2(DegServo_t2(firstswing_t2)));
        writeServos(3, ConvertLeg2(firstswing_t3));  
          //2nd module
        writeServos(4, DegServo_t2(firstsup_t2));
        writeServos(5, firstsup_t3);
        writeServos(6, ConvertLeg2(DegServo_t2(firstswing_t2)));
        writeServos(7, ConvertLeg2(firstswing_t3));  
          //3rd module
        writeServos(8, DegServo_t2(firstsup_t2));
        writeServos(9, firstsup_t3);
        writeServos(10, ConvertLeg2(DegServo_t2(firstswing_t2)));
        writeServos(11, ConvertLeg2(firstswing_t3));

        Serial.print(int(accX*100)), Serial.print("_"), Serial.print(int(accY*100)), Serial.print("_"), Serial.print(int(gyroX*100)), Serial.print("_"), Serial.println(int(gyroY*100));
        if (Serial.available() > 0) {
          incomingByte = Serial.read();
          if (incomingByte == 115){
            break;
          }
        }
      }
      i = 1;
    }
    //First step
    if ( i == 1 && incomingByte == 119 ){
      ControlPoints(0);
      for (float t=0; t<1; t=t+TimeStep(T)){
        SwingPhase(t);
        SupportPhase(t);
        //First step
          //1st module
        writeServos(0, DegServo_t2(swing_t2));
        writeServos(1, swing_t3);
        writeServos(2, ConvertLeg2(DegServo_t2(sup_t2)));
        writeServos(3, ConvertLeg2(sup_t3)); 
          //2nd module
        writeServos(4, DegServo_t2(swing_t2));
        writeServos(5, swing_t3);
        writeServos(6, ConvertLeg2(DegServo_t2(sup_t2)));
        writeServos(7, ConvertLeg2(sup_t3));
          //3rd module
        writeServos(8, DegServo_t2(swing_t2));
        writeServos(9, swing_t3);
        writeServos(10, ConvertLeg2(DegServo_t2(sup_t2)));
        writeServos(11, ConvertLeg2(sup_t3));

        Serial.print(int(accX*100)), Serial.print("_"), Serial.print(int(accY*100)), Serial.print("_"), Serial.print(int(gyroX*100)), Serial.print("_"), Serial.println(int(gyroY*100));
        if (Serial.available() > 0) {
          incomingByte = Serial.read(); 
          if (incomingByte == 115){
            break;
            }
          }
        }
      i = 2;
      } 
    //Second step
    if ( i == 2 && incomingByte == 119 ){
      ControlPoints(0); 
      for (float t=0; t<1; t=t+TimeStep(T)){
        SwingPhase(t);
        SupportPhase(t);
        //Second step
          //1st module
        writeServos(0, DegServo_t2(sup_t2));
        writeServos(1, sup_t3);
        writeServos(2, ConvertLeg2(DegServo_t2(swing_t2)));
        writeServos(3, ConvertLeg2(swing_t3));  
          //2nd module
        writeServos(4, DegServo_t2(sup_t2));
        writeServos(5, sup_t3);
        writeServos(6, ConvertLeg2(DegServo_t2(swing_t2)));
        writeServos(7, ConvertLeg2(swing_t3));  
          //3rd module
        writeServos(8, DegServo_t2(sup_t2));
        writeServos(9, sup_t3);
        writeServos(10, ConvertLeg2(DegServo_t2(swing_t2)));
        writeServos(11, ConvertLeg2(swing_t3)); 

        Serial.print(int(accX*100)), Serial.print("_"), Serial.print(int(accY*100)), Serial.print("_"), Serial.print(int(gyroX*100)), Serial.print("_"), Serial.println(int(gyroY*100));
        if (Serial.available() > 0) {
          incomingByte = Serial.read();
          if (incomingByte == 115){
            break;
            }
          }
        }     
        i = 1;
      }
      //Turning right
      if (incomingByte == 100){
        angle = 30;
        ControlPoints(angle);
        for (float t=0; t<1; t = t+TimeStep(T)){
          SwingPhase(t);
          StancePhase();
          if ((swing_p1 + prev_angle) > 90){
            //Serial.println("Max angle reached");
            break;
          }
          else{
          //Serial.print("Servo angle: ");
          //Serial.println(swing_p1 + prev_angle);
          //1st module
          writeServos(12, DegServo_p1(swing_p1 + prev_angle));
          writeServos(0, DegServo_t2(stance_t2));
          writeServos(1, stance_t3);
          writeServos(2, ConvertLeg2(DegServo_t2(swing_t2)));
          writeServos(3, ConvertLeg2(swing_t3)); 
          //2nd module
          writeServos(13, DegServo_p1(swing_p1 + prev_angle));
          writeServos(4, DegServo_t2(stance_t2));
          writeServos(5, stance_t3);
          writeServos(6, ConvertLeg2(DegServo_t2(swing_t2)));
          writeServos(7, ConvertLeg2(swing_t3));
          //3rd module
          writeServos(14, DegServo_p1(swing_p1 + prev_angle));
          writeServos(8, DegServo_t2(stance_t2));
          writeServos(9, stance_t3);
          writeServos(10, ConvertLeg2(DegServo_t2(swing_t2)));
          writeServos(11, ConvertLeg2(swing_t3));
          }        
          Serial.print(int(accX*100)), Serial.print("_"), Serial.print(int(accY*100)), Serial.print("_"), Serial.print(int(gyroX*100)), Serial.print("_"), Serial.println(int(gyroY*100));
        }
        if ((swing_p1 + prev_angle) < 90){
          prev_angle = swing_p1 + prev_angle;
        }
        else{       
          prev_angle = 90;
        }
        StancePhase();
          //1st module
        writeServos(0, DegServo_t2(stance_t2 + fPos[0]));
        writeServos(1, stance_t3 + fPos[1]);
        writeServos(2, ConvertLeg2(DegServo_t2(stance_t2 + fPos[2])));
        writeServos(3, ConvertLeg2(stance_t3 + fPos[3])); 
        //2nd module
        writeServos(4, DegServo_t2(stance_t2 + fPos[4]));
        writeServos(5, stance_t3 + fPos[5]);
        writeServos(6, ConvertLeg2(DegServo_t2(stance_t2 + fPos[6])));
        writeServos(7, ConvertLeg2(stance_t3 + fPos[7]));
        //3rd module
        writeServos(8, DegServo_t2(stance_t2 + fPos[8]));
        writeServos(9, stance_t3 + fPos[9]);
        writeServos(10, ConvertLeg2(DegServo_t2(stance_t2 + fPos[10])));
        writeServos(11, ConvertLeg2(stance_t3 + fPos[11]));

        i = 0;
        incomingByte = 115;
        
      }
      //Turning left
      if (incomingByte == 97){
        angle = - 30;
        ControlPoints(angle);
        for (float t=0; t<1; t = t+TimeStep(T)){
          SwingPhase(t);
          StancePhase();
          if ((swing_p1 + prev_angle) < -90){
              //Serial.println("Min angle reached");
              break;
            }  
          else{
            //Serial.print("Servo angle: ");
            //Serial.println(swing_p1 + prev_angle);
            //1st module
            writeServos(12, DegServo_p1(swing_p1 + prev_angle));
            writeServos(0, DegServo_t2(swing_t2));
            writeServos(1, swing_t3);
            writeServos(2, ConvertLeg2(DegServo_t2(stance_t2)));
            writeServos(3, ConvertLeg2(stance_t3)); 
            //2nd module
            writeServos(13, DegServo_p1(swing_p1 + prev_angle));
            writeServos(4, DegServo_t2(swing_t2));
            writeServos(5, swing_t3);
            writeServos(6, ConvertLeg2(DegServo_t2(stance_t2)));
            writeServos(7, ConvertLeg2(stance_t3));
            //3rd module
            writeServos(14, DegServo_p1(swing_p1 + prev_angle));
            writeServos(8, DegServo_t2(swing_t2));
            writeServos(9, swing_t3);
            writeServos(10, ConvertLeg2(DegServo_t2(stance_t2)));
            writeServos(11, ConvertLeg2(stance_t3));
            }     
            Serial.print(int(accX*100)), Serial.print("_"), Serial.print(int(accY*100)), Serial.print("_"), Serial.print(int(gyroX*100)), Serial.print("_"), Serial.println(int(gyroY*100));   
        }
        if ((swing_p1 + prev_angle) > -90){
          prev_angle = swing_p1 + prev_angle;
        }
        else{
          prev_angle = -90;
        }
        StancePhase();
          //1st module
        writeServos(0, DegServo_t2(stance_t2 + fPos[0]));
        writeServos(1, stance_t3 + fPos[1]);
        writeServos(2, ConvertLeg2(DegServo_t2(stance_t2 + fPos[2])));
        writeServos(3, ConvertLeg2(stance_t3 + fPos[3])); 
        //2nd module
        writeServos(4, DegServo_t2(stance_t2 + fPos[4]));
        writeServos(5, stance_t3 + fPos[5]);
        writeServos(6, ConvertLeg2(DegServo_t2(stance_t2 + fPos[6])));
        writeServos(7, ConvertLeg2(stance_t3 + fPos[7]));
        //3rd module
        writeServos(8, DegServo_t2(stance_t2 + fPos[8]));
        writeServos(9, stance_t3 + fPos[9]);
        writeServos(10, ConvertLeg2(DegServo_t2(stance_t2 + fPos[10])));
        writeServos(11, ConvertLeg2(stance_t3 + fPos[11]));
        
        i = 0;          
        incomingByte = 115;
      }
      //Stance pose == Restart
      if (incomingByte == 114){
        i = 0;
        prev_angle = 0;
        StancePhase();
        writeServos(12, DegServo_p1(stance_p1 + fPos[12]));
        writeServos(0, DegServo_t2(stance_t2 + fPos[0]));
        writeServos(1, stance_t3 + fPos[1]);
        writeServos(2, ConvertLeg2(DegServo_t2(stance_t2 + fPos[2])));
        writeServos(3, ConvertLeg2(stance_t3 + fPos[3])); 
        //2nd module
        writeServos(13, DegServo_p1(stance_p1 + fPos[13]));
        writeServos(4, DegServo_t2(stance_t2 + fPos[4]));
        writeServos(5, stance_t3 + fPos[5]);
        writeServos(6, ConvertLeg2(DegServo_t2(stance_t2 + fPos[6])));
        writeServos(7, ConvertLeg2(stance_t3 + fPos[7]));
        //3rd module
        writeServos(14, DegServo_p1(stance_p1 + fPos[14]));
        writeServos(8, DegServo_t2(stance_t2 + fPos[8]));
        writeServos(9, stance_t3 + fPos[9]);
        writeServos(10, ConvertLeg2(DegServo_t2(stance_t2 + fPos[10])));
        writeServos(11, ConvertLeg2(stance_t3 + fPos[11]));
        //Serial.println("Restart position");
        incomingByte = 0; 
      }
      //Stop
      if ( incomingByte == 115){
        //Serial.println("Stop");
        incomingByte = 0; 
      }
      //Finish program
      if ( incomingByte == 102){
        //Serial.println("Finish program");
        delay(2);
        exit(0); 
      }   
  }  
}
