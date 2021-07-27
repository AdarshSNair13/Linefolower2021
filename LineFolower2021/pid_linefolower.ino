#include <QTRSensors.h>

#define Kp 2//0.8///1 experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 20//16//18//14
// experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define MaxSpeed 200//190 max speed of the robot
#define BaseSpeed 180 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define NUM_SENSORS  6     // number of sensors used

#define speedturn 180

#define rightMotor1 3
#define rightMotor2 4
#define rightMotorPWM 5
#define leftMotor1 9
#define leftMotor2 10
#define leftMotorPWM 11
#define motorPower 8

QTRSensorsRC qtrrc((unsigned char[]) {19, 18, 17, 16, 15, 14} ,NUM_SENSORS, 2500, QTR_NO_EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

void setup()
{
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(motorPower, OUTPUT);
  
 // delay(300);
  pinMode(2,OUTPUT); 
  int i;
  digitalWrite(2,HIGH);
  for (int i = 0; i < 200; i++) // calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead
  {
   //comment this part out for automatic calibration 
//    if ( i  < 25 || i >= 75 ) // turn to the left and right to expose the sensors to the brightest and darkest readings that may be encountered
//    {
//      move(1, 150, 1);//motor derecho hacia adelante
//      move(0, 150, 0);//motor izquierdo hacia atras 
//    }
//    else
//    {
//      move(1, 150, 0);//motor derecho hacia atras
//      move(0, 150, 1);//motor izquierdo hacia adelante  
//    }
    qtrrc.calibrate();   
    delay(15);
  }
  digitalWrite(2,LOW);
  wait();
  delay(1000); // wait for 2s to position the bot before entering the main loop 
}  

int lastError = 0;
unsigned int sensors[6];
int position = qtrrc.readLine(sensors);

void loop()
{  
  position = qtrrc.readLine(sensors); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  
  if(position>3000){
    move(1, speedturn, 1);//right motor forward speedturn
    move(0, speedturn, 0);//left engine forward
   // delay(70);
    return;    
  }
  if(position<300){ 
    move(1, speedturn, 0);//motor derecho hacia adelante
    move(0, speedturn, 1);//motor izquierdo hacia adelante
   // delay(70);
    return;
  }


// 
//  if(position<300 && position<3000){ 
//    move(1, speedturn, 0);//motor derecho hacia adelante
//    move(0, speedturn, 1);//motor izquierdo hacia adelante
//    return;
//  }
// if(position>3000 && position>300){
//    move(1, speedturn, 1);//right motor forward
//    move(0, speedturn, 0);//left engine forward
//    return;    
//  }


//  if(position>3000 && position<4800){
//    
//    
//    digitalWrite(rightMotor1, 1);
//    digitalWrite(rightMotor2, 0);
//    analogWrite(rightMotorPWM, 180);
//    digitalWrite(leftMotor1, 0);
//    digitalWrite(leftMotor2, 1);
//    analogWrite(leftMotorPWM, 180);
// 
//    return;    
//  }
//  if(position<300 && position>100){ 
//   digitalWrite(leftMotor1, 1);
//    digitalWrite(leftMotor2, 0);
//    analogWrite(leftMotorPWM, 180);
//    digitalWrite(rightMotor1, 0);
//    digitalWrite(rightMotor2, 1);
//    analogWrite(rightMotorPWM,180);
//
//    return;
//  }
//  if(position<=5000 && position>4800){
//    
//    
//    digitalWrite(rightMotor1, 1);
//    digitalWrite(rightMotor2, 0);
//    analogWrite(rightMotorPWM, 220);
//    digitalWrite(leftMotor1, 0);
//    digitalWrite(leftMotor2, 1);
//    analogWrite(leftMotorPWM, 220);
// 
//    return;    
//  }
//  if(position>=0 && position<100){ 
//   digitalWrite(leftMotor1, 1);
//    digitalWrite(leftMotor2, 0);
//    analogWrite(leftMotorPWM, 220);
//    digitalWrite(rightMotor1, 0);
//    digitalWrite(rightMotor2, 1);
//    analogWrite(rightMotorPWM,220);
//
//    return;
//  }


//  if(position>3000){
//    
//    
//    digitalWrite(rightMotor1, 1);
//    digitalWrite(rightMotor2, 0);
//    analogWrite(rightMotorPWM, 240);
//    digitalWrite(leftMotor1, 0);
//    digitalWrite(leftMotor2, 1);//1
//    analogWrite(leftMotorPWM, 240);
// //delay(50);
//    return;    
//  }
// 
//  if(position<400){ 
//   digitalWrite(leftMotor1, 1);//1
//    digitalWrite(leftMotor2, 0);
//    analogWrite(leftMotorPWM, 240);
//    digitalWrite(rightMotor1, 0);
//    digitalWrite(rightMotor2, 1);//1
//    analogWrite(rightMotorPWM,240);
//   // delay(50);
//  }

  

position = qtrrc.readLine(sensors);
  
  int error = position - 2500;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = BaseSpeed + motorSpeed;
  int leftMotorSpeed = BaseSpeed - motorSpeed;
  
  if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > MaxSpeed ) leftMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0)rightMotorSpeed = 0;    
  if (leftMotorSpeed < 0)leftMotorSpeed = 0;
    
  move(1, rightMotorSpeed, 1);//motor derecho hacia adelante
  move(0, leftMotorSpeed, 1);//motor izquierdo hacia adelante
}
  
void wait(){
  digitalWrite(motorPower, LOW);
}

void move(int motor, int speed, int direction){
  digitalWrite(motorPower, HIGH); //disable standby

  boolean inPin1=HIGH;
  boolean inPin2=LOW;
  
  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }  
  if(direction == 0){
    inPin1 = LOW;
    inPin2 = HIGH;
  }

  if(motor == 0){
    digitalWrite(leftMotor1, inPin1);
    digitalWrite(leftMotor2, inPin2);
    analogWrite(leftMotorPWM, speed);
  }
  if(motor == 1){
    digitalWrite(rightMotor1, inPin1);
    digitalWrite(rightMotor2, inPin2);
    analogWrite(rightMotorPWM, speed);
  }  
}
