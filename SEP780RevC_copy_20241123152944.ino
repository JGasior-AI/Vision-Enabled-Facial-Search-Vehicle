//GOOD SO FAR 

#include <IRremote.h>
#include <Servo.h>
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
 
// Pin definitions
#define PIN_SERVO 2
#define MOTOR_DIRECTION 1
#define PIN_DIRECTION_LEFT 4
#define PIN_DIRECTION_RIGHT 3
#define PIN_MOTOR_PWM_LEFT 6
#define PIN_MOTOR_PWM_RIGHT 5
#define PIN_SONIC_TRIG 7
#define PIN_SONIC_ECHO 8
#define PIN_IRREMOTE_RECV 9
#define PIN_BATTERY A0
#define PIN_BUZZER      A0
 
// Constants
#define IR_UPDATE_TIMEOUT 110
#define IR_CAR_SPEED 100
#define OBSTACLE_DISTANCE 20
#define OBSTACLE_DISTANCE_LOW 10
#define MAX_DISTANCE 1000
#define SONIC_TIMEOUT (MAX_DISTANCE*60)
#define SOUND_VELOCITY 340
 
// IR remote key codes
#define IR_REMOTE_KEYCODE_UP 0xFF02FD
#define IR_REMOTE_KEYCODE_DOWN 0xFF9867
#define IR_REMOTE_KEYCODE_LEFT 0xFFE01F
#define IR_REMOTE_KEYCODE_RIGHT 0xFF906F
#define IR_REMOTE_KEYCODE_CENTER 0xFFA857
#define IR_REMOTE_KEYCODE_1 0xFF30CF
#define IR_REMOTE_KEYCODE_9 0xFF52AD
 
IRrecv irrecv(PIN_IRREMOTE_RECV);
decode_results results;
Servo servo;
 
bool isAutoMode = false;
bool isCarStopped = false;
bool faceFound = false;
int speedOffset = -75;
unsigned long lastIRUpdateTime = 0;
bool autoOn = false;
 
// Define HUSKYLENS
HUSKYLENS huskylens;
SoftwareSerial mySerial(18, 19); // RX, TX
//HUSKYLENS green line >> Pin 10; blue line >> Pin 11

///Define variables
float batteryVoltage = 0;
bool isBuzzered = false;

void setup() {
  pinMode(PIN_DIRECTION_LEFT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_LEFT, OUTPUT);
  pinMode(PIN_DIRECTION_RIGHT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_RIGHT, OUTPUT);
  pinMode(PIN_SONIC_TRIG, OUTPUT);
  pinMode(PIN_SONIC_ECHO, INPUT);
  servo.attach(PIN_SERVO);
  irrecv.enableIRIn();
  calculateVoltageCompensation();

    Serial.begin(9600);
    mySerial.begin(9600);
    while (!huskylens.begin(mySerial))
    {
        Serial.println(F("Begin failed!"));
        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>Serial 9600)"));
        Serial.println(F("2.Please recheck the connection."));
        delay(100);
    }
}
 
void loop() {
  //battery level check. if low, beep
  checkBatteryVoltage();

  if (irrecv.decode(&results)) {
    handleIRCommand(results.value);
    irrecv.resume();
    lastIRUpdateTime = millis();
  } else if (!isAutoMode && millis() - lastIRUpdateTime > IR_UPDATE_TIMEOUT) {
    motorRun(0, 0);
  }
  if (isAutoMode && !isCarStopped) {
    
    huskyLensRun();
    if (!faceFound) {
      updateAutomaticObstacleAvoidance();
    } else {
      isAutoMode = false;
    }
  }

}

void huskyLensOutput(HUSKYLENSResult result) {

    Serial.println(result.ID);
    if (result.ID == 1) {
      faceFound = true;
      resetCarAction();
      alarm(4,1);
    } else {
      faceFound = false;
    }
}

void huskyLensRun() {
    if (!huskylens.request()) Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
    else if(!huskylens.isLearned()) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
    else if(!huskylens.available()) Serial.println(F("No block or arrow appears on the screen!"));
    else
    {
        Serial.println(F("###########"));
        while (huskylens.available())
        {
            HUSKYLENSResult result = huskylens.read();
            huskyLensOutput(result);
            break;
        }    
    }

}

void resetCarAction() {
  motorRun(0, 0);
  setBuzzer(false);
}

void setBuzzer(bool flag) {
  isBuzzered = flag;
  pinMode(PIN_BUZZER, flag);
  digitalWrite(PIN_BUZZER, flag);
}

void alarm(u8 beat, u8 repeat) {
  beat = constrain(beat, 1, 9);
  repeat = constrain(repeat, 1, 255);
  for (int j = 0; j < repeat; j++) {
    for (int i = 0; i < beat; i++) {
      setBuzzer(true);
      delay(100);
      setBuzzer(false);
      delay(100);
    }
    delay(500);
  }
}

void handleIRCommand(unsigned long command) {
  switch (command) {
    case IR_REMOTE_KEYCODE_UP:
      if (!isAutoMode && !isCarStopped) motorRun(IR_CAR_SPEED, IR_CAR_SPEED);
      break;
    case IR_REMOTE_KEYCODE_DOWN:
      if (!isAutoMode && !isCarStopped) motorRun(-IR_CAR_SPEED, -IR_CAR_SPEED);
      break;
    case IR_REMOTE_KEYCODE_LEFT:
      if (!isAutoMode && !isCarStopped) motorRun(-IR_CAR_SPEED, IR_CAR_SPEED);
      break;
    case IR_REMOTE_KEYCODE_RIGHT:
      if (!isAutoMode && !isCarStopped) motorRun(IR_CAR_SPEED, -IR_CAR_SPEED);
      break;
    case IR_REMOTE_KEYCODE_CENTER:
      isCarStopped = !isCarStopped;
      isAutoMode = false;
      motorRun(0, 0);
      break;
    case IR_REMOTE_KEYCODE_1:
      if (!isCarStopped) {
        isAutoMode = !isAutoMode;
        motorRun(0, 0);
      }
      break;
    case IR_REMOTE_KEYCODE_9:
      //reset car
      isAutoMode = false;
      isCarStopped = false;
      faceFound = false;
      autoOn = false;
      alarm(2,1);
  }
}
 
void updateAutomaticObstacleAvoidance() {
  int distance[3], tempDistance[3][5], sumDisntance;
  static u8 leftToRight = 0, servoAngle = 0, lastServoAngle = 0;
  const u8 scanAngle[2][3] = { {150, 90, 30}, {30, 90, 150} };
  int delayTime = 200;
  double speedRatio = 0.90;
  //move car forward until it starts seeing obstacles
  if (autoOn == false) {
    motorRun(90,90);
    autoOn = true;
  }
 
  for (int i = 0; i < 3; i++) {
    servoAngle = scanAngle[leftToRight][i];
    servo.write(servoAngle);
    if (lastServoAngle != servoAngle) {
      delay(130);
    }
    lastServoAngle = servoAngle;
    for (int j = 0; j < 5; j++) {
      tempDistance[i][j] = getSonar();
      delayMicroseconds(2 * SONIC_TIMEOUT);
      sumDisntance += tempDistance[i][j];
    }
    if (leftToRight == 0) {
      distance[i] = sumDisntance / 5;
    } else {
      distance[2 - i] = sumDisntance / 5;
    }
    sumDisntance = 0;
  }
  leftToRight = (leftToRight + 1) % 2;
  
  if (distance[1] < OBSTACLE_DISTANCE) {
    if (distance[0] > distance[2] && distance[0] > OBSTACLE_DISTANCE) {
      motorRun(-(speedRatio * 150), -(speedRatio * 150));
      delay(delayTime);
      motorRun(-(speedRatio * 150), (speedRatio * 150));
    } else if (distance[0] < distance[2] && distance[2] > OBSTACLE_DISTANCE) {
      motorRun(-(speedRatio * 150), -(speedRatio * 150));
      delay(delayTime);
      motorRun((speedRatio * 150), -(speedRatio * 150));
    } else {
      motorRun(-(speedRatio * 150), -(speedRatio * 150));
      delay(delayTime);
      motorRun(-(speedRatio * 150), (speedRatio * 150));
    }
  } else {
    if (distance[0] < OBSTACLE_DISTANCE_LOW) {
      motorRun(-(speedRatio * 150), -(speedRatio * 150));
      delay(delayTime);
      motorRun((speedRatio * 180), (speedRatio * 50));
    } else if (distance[2] < OBSTACLE_DISTANCE_LOW) {
      motorRun(-(speedRatio * 150), -(speedRatio * 150));
      delay(delayTime);
      motorRun((speedRatio * 50), (speedRatio * 180));
    } else {
      motorRun((speedRatio * 80), (speedRatio * 80));
    }
  }
}
 
float getSonar() {
  unsigned long pingTime;
  float distance;
  digitalWrite(PIN_SONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_SONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_SONIC_TRIG, LOW);
  pingTime = pulseIn(PIN_SONIC_ECHO, HIGH, SONIC_TIMEOUT);
  if (pingTime != 0)
    distance = (float)pingTime * SOUND_VELOCITY / 2 / 10000;
  else
    distance = MAX_DISTANCE;
  return distance;
}
 
void motorRun(double speedl, double speedr) {
  int dirL = (speedl > 0) ? (0 ^ MOTOR_DIRECTION) : (1 ^ MOTOR_DIRECTION);
  int dirR = (speedr > 0) ? (1 ^ MOTOR_DIRECTION) : (0 ^ MOTOR_DIRECTION);
  speedl = abs(speedl);
  speedr = abs(speedr);
  digitalWrite(PIN_DIRECTION_LEFT, dirL);
  digitalWrite(PIN_DIRECTION_RIGHT, dirR);
  analogWrite(PIN_MOTOR_PWM_LEFT, speedl);
  analogWrite(PIN_MOTOR_PWM_RIGHT, speedr);
}
 
void calculateVoltageCompensation() {
  float voltageOffset = 8.4 - getBatteryVoltage();
  speedOffset = voltageOffset * 20;
}
 
float getBatteryVoltage() {
  pinMode(PIN_BATTERY, INPUT);
  int batteryADC = analogRead(PIN_BATTERY);
  float batteryVoltage = batteryADC / 1023.0 * 5.0 * 4;
  return batteryVoltage;
}

void checkBatteryVoltage() {
  float batteryVolt = getBatteryVoltage();
  if (batteryVolt > 0) {
      Serial.println(String("Battery Voltage: ") + String(batteryVolt,1));
  }
  //if 
  if (batteryVolt < 6.4) {
      alarm(2,1);
  }
  
}