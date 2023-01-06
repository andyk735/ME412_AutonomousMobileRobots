#include <RPLidar.h>
#define RPLIDAR_MOTOR 2 // The PWM pin for control the speed of RPLIDAR's motor (MOTOCTRL).

RPLidar lidar;

int motor2pin1 = 10;
int motor2pin2 = 8;

int motor3pin1 = 12;
int motor3pin2 = 13;

int motor4pin1 = 7;
int motor4pin2 = 5;

int motor1pin1 = 22;
int motor1pin2 = 4;

unsigned long previousMillis = 0;
const long interval = 2000;
                      
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);  // For RPLidar
  lidar.begin(Serial1);
  pinMode(RPLIDAR_MOTOR, OUTPUT);  // set pin modes

  // Motor Digital Pins
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(motor3pin1, OUTPUT);
  pinMode(motor3pin2, OUTPUT);
  pinMode(motor4pin1, OUTPUT);
  pinMode(motor4pin2, OUTPUT);

  // Motor PWM Connections
  pinMode(3, OUTPUT);   // Motor 4
  pinMode(6, OUTPUT);   // Motor 3
  pinMode(9, OUTPUT);   // Motor 1
  pinMode(11, OUTPUT);  // Motor 2
}

float minDistance = 100000;
float angleAtMinDist = 0; 
int condition = 0;

void loop() {
  if (IS_OK(lidar.waitPoint())) {
    //perform data processing here... 
    float distance = lidar.getCurrentPoint().distance;
    float angle = lidar.getCurrentPoint().angle;  // 0-360 deg
    unsigned long currentMillis = millis();
    
    if (lidar.getCurrentPoint().startBit) {
       // a new scan, display the previous data...
       printData(angleAtMinDist, minDistance);
       if (minDistance < 370 && angleAtMinDist > 290){
        Serial.print("close left");
        previousMillis = currentMillis;
        turnright(); 
       }
       if (minDistance < 370 && angleAtMinDist < 70){
        Serial.print("close right:  ");
        previousMillis = currentMillis;
        turnleft();
       }
       minDistance = 100000;
       angleAtMinDist = 0;
    } 
    else if (currentMillis - previousMillis >= interval){
        previousMillis = currentMillis;
        moveforward();
    }
    else {
       if ( distance > 0 && distance < minDistance) {
          minDistance = distance;
          angleAtMinDist = angle;
       }
    }
  }
  else {
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
    // Try to detect RPLIDAR
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       // Detected
       lidar.startScan();
       analogWrite(RPLIDAR_MOTOR, 255);
       delay(1000);
    }
  }
}

void printData(float angle, float distance)
{
  Serial.print("dist: ");
  Serial.print(distance);
  Serial.print("    angle: ");
  Serial.println(angle);
}

void moveforward(){
  // Code to move forward
  // Motors 1 & 2 
   
  // Controlling speed (0 = off and 255 = max speed):
  analogWrite(3, 129); // 79 // Motor 1
  analogWrite(9, 118); // 68 // Motor 2

  //Controlling spin direction of motors:
  digitalWrite(motor1pin1, HIGH);  // CCW
  digitalWrite(motor1pin2, LOW); 

  digitalWrite(motor2pin1, LOW); // CW
  digitalWrite(motor2pin2, HIGH);

  // Motors 3 & 4

  // Controlling speed (0 = off and 255 = max speed):
  analogWrite(11, 137); // 74.5 // Motor 3
  analogWrite(6, 135); // 76.4 // Motor 4

  //Controlling spin direction of motors:
  digitalWrite(motor3pin1, HIGH);  //  CW
  digitalWrite(motor3pin2, LOW);

  digitalWrite(motor4pin1, HIGH);  // CCW
  digitalWrite(motor4pin2, LOW);
  delay(450);

  digitalWrite(motor1pin1, LOW);  // CCW
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);  // CCW
  digitalWrite(motor2pin2, LOW);
  digitalWrite(motor3pin1, LOW);  // CCW
  digitalWrite(motor3pin2, LOW);
  digitalWrite(motor4pin1, LOW);  // CCW
  digitalWrite(motor4pin2, LOW);
}

void stopmove(){
  digitalWrite(motor1pin1, LOW);  // CCW
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, LOW);  // CCW
  digitalWrite(motor2pin2, LOW);

    //Controlling spin direction of motors:
  digitalWrite(motor3pin1, LOW);  // CCW
  digitalWrite(motor3pin2, LOW);

  digitalWrite(motor4pin1, LOW);  // CCW
  digitalWrite(motor4pin2, LOW);
}

void turnleft(){
  // Motors 1 & 2 
    //Controlling speed (0 = off and 255 = max speed):
    analogWrite(9, 88); //ENA pin 48 / 68
    analogWrite(11, 94.5); //ENB pin 54.5 / 74.5

    //Controlling spin direction of motors:
    digitalWrite(motor1pin1, LOW);  // CW
    digitalWrite(motor1pin2, HIGH);

    digitalWrite(motor2pin1, LOW);  // CW
    digitalWrite(motor2pin2, HIGH);

    // Motors 3 & 4
    // Controlling speed (0 = off and 255 = max speed):
    analogWrite(3, 99); //ENA pin 53 /  79
    analogWrite(6, 96.4); //ENB pin 53.4 / 76.4

    //Controlling spin direction of motors:
    digitalWrite(motor3pin1, HIGH);  // CW
    digitalWrite(motor3pin2, LOW);

    digitalWrite(motor4pin1, LOW);  // CW
    digitalWrite(motor4pin2, HIGH);
    delay(300);

    digitalWrite(motor1pin1, LOW); 
    digitalWrite(motor1pin2, LOW);
  
    digitalWrite(motor2pin1, LOW); 
    digitalWrite(motor2pin2, LOW);
  
      //Controlling spin direction of motors:
    digitalWrite(motor3pin1, LOW);
    digitalWrite(motor3pin2, LOW);
  
    digitalWrite(motor4pin1, LOW);
    digitalWrite(motor4pin2, LOW);
    delay(300);
}

void turnright(){
     //Controlling speed (0 = off and 255 = max speed):
    analogWrite(9, 88); //ENA pin 48 / 68
    analogWrite(11, 94.5); //ENB pin 54.5 / 74.5
  
    //Controlling spin direction of motors:
    digitalWrite(motor1pin1, HIGH);  // CCW
    digitalWrite(motor1pin2, LOW);
  
    digitalWrite(motor2pin1, HIGH);  // CCW
    digitalWrite(motor2pin2, LOW);
  
    // Motors 3 & 4
    
    // Controlling speed (0 = off and 255 = max speed):
    analogWrite(3, 99); //ENA pin 53 /  79
    analogWrite(6, 96.4); //ENB pin 53.4 / 76.4
  
    //Controlling spin direction of motors:
    digitalWrite(motor3pin1, LOW);  // CCW
    digitalWrite(motor3pin2, HIGH);
  
    digitalWrite(motor4pin1, HIGH);  // CCW
    digitalWrite(motor4pin2, LOW);
    delay(300);
  
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, LOW);
  
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, LOW);
  
      //Controlling spin direction of motors:
    digitalWrite(motor3pin1, LOW);
    digitalWrite(motor3pin2, LOW);
  
    digitalWrite(motor4pin1, LOW); 
    digitalWrite(motor4pin2, LOW);
    delay(300);
}
