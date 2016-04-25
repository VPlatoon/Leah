#include <Arduino.h>
#include <Servo.h>
#include "Encoder.h"
#include "SharpIR.h"
void setup();
void loop();
bool is_forward_possible();
void led_blink(int ntimes, int t);
bool is_alternative_route();
void set_adaptive_speed();
void change_moving_direction();
void blind_run();
void run_until_collide();
void scan_best_routes();
void test_frontIR ();
void test_turretIR ();
#line 1 "src/mini_driver_firmware.ino"
//#include <Servo.h>
//#include "Encoder.h"
//#include "SharpIR.h"

const int LEFT_MOTOR_DIR_PIN = 7;	// Direction
const int LEFT_MOTOR_PWM_PIN = 9;	// Pulse Width Modulation
const int RIGHT_MOTOR_DIR_PIN = 8;
const int RIGHT_MOTOR_PWM_PIN = 10;
const int PAN_SERVO_PIN = 6;
const int GRIP_SERVO_PIN = 11;

const int IRSHARP_NEAR_RANGE_PIN = A1; // GP2Y0A21YK: analog output from 3.1V at 10cm to 0.4V at 80cm
const int IRSHARP_LONG_RANGE_PIN = A2; // GP2Y0A02YK0F:  analog output from 2.8V at 15cm to 0.4V at 150cm

const int IRSHARP_NEAR_RANGE_MODELID = 430;
const int IRSHARP_LONG_RANGE_MODELID = 20150;
const int IRSHARP_NUM_READING = 25;
const int IRSHARP_DIFF_PERCENT = 93;

const int DANGER_DISTANCE_AHEAD = 30; // centimeters
const int DANGER_DISTANCE_BEHIND = 30; // centimeters

const int LED_PIN = 13;

const int MOTOR_PWM_80HZ_OCR2 = 62;     // Fast PWM used for fast motor speeds
const int MOTOR_PWM_20HZ_OCR2 = 255;    // Slow PWM used for slow motor speeds
const int MOTOR_PWM_80HZ_DUTY_CYCLE = 50;
const int MOTOR_PWM_20HZ_DUTY_CYCLE = 5;

const int ABSOLUTE_MIN_PWM = 400;
const int ABSOLUTE_MAX_PWM = 2600;

const unsigned long MOTOR_COMMAND_TIMEOUT_MS = 500;

// This class is provided because the Arduino Servo library maps minimum and
// maximum bounds to a single byte for some reason.
class ServoLimits
{
  // MIN_PULSE_WIDTH and MAX_PULSE_WIDTH come from Servo.h
  public: ServoLimits( int minPWM=MIN_PULSE_WIDTH, int maxPWM=MAX_PULSE_WIDTH ) {
    setLimits( minPWM, maxPWM );
  }

  public: void setLimits( int minPWM, int maxPWM ) {
    mMinPWM = constrain( minPWM, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM );
    mMaxPWM = constrain( maxPWM, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM );
  }

  public: int convertAngleToPWM( int angle ) {
    angle = constrain( angle, 0, 180 );
    return map( angle, 0, 180, mMinPWM, mMaxPWM );
  }

  public: int getMinPWM() const { return mMinPWM; }
  public: int getMaxPWM() const { return mMaxPWM; }

  private: int mMinPWM;
  private: int mMaxPWM;
};

void blind_run();
void scan_best_routes();
void run_until_collide();

enum eMotorDirection {
  eMD_Forwards,
  eMD_Backwards
};
 enum eTurretDirection {
   eTD_Left,
   eTD_Right
 };

Servo gPanServo;
ServoLimits gPanServoLimits;
Servo gGripServo;
ServoLimits gGripServoLimits;

// Declare two IR sensor objects, the first is the front long-range sensor,
// the second is the turret short-range sensor.
SharpIR gFrontIR(IRSHARP_LONG_RANGE_PIN, IRSHARP_NUM_READING, IRSHARP_DIFF_PERCENT, IRSHARP_LONG_RANGE_MODELID);
SharpIR gTurretIR(IRSHARP_NEAR_RANGE_PIN, IRSHARP_NUM_READING, IRSHARP_DIFF_PERCENT, IRSHARP_NEAR_RANGE_MODELID);

uint8_t gLeftMotorDutyCycle = 0;
uint8_t gRightMotorDutyCycle = 0;

// Some initialization for global variables
eMotorDirection gLeftMotorDirection = eMD_Forwards;
eMotorDirection gRightMotorDirection = eMD_Forwards;

uint8_t gPanServoAngle = 180;
uint8_t gGripServoAngle = 90;
bool gRunFirstTime = true;

int gLongestDistance = -1;
int gBestRelativeAngle = 0;
int gTurretDirection = eTD_Left;
long gLastCommandTime;
volatile uint8_t gCurMotorWaveTick = 0;

void setup() {
  // Set up timer 2 in CTC mode with a prescaler of clk/32, so with the chip
  // running at 16MHz this gives 500,000 clock ticks per second. By varying
  // the value of OCR2 we will generate an interrupt from roughly 8000 times a
  // second to 2000 times a second which gives a motor PWM frequency of 80Hz
  // to 20Hz. Lower PWM frequencies give choppier movement
  noInterrupts();
  TCCR2 = (1 << 3) | 3;      // Activate timer in CTC mode with a prescaler of clk/32
  OCR2 = 0xFF;

  TIMSK |= (1 << OCIE2);     // Activate timer 2 output compare interrupt

  interrupts();

  gPanServo.attach( PAN_SERVO_PIN );
  gGripServo.attach( GRIP_SERVO_PIN );

  pinMode( LEFT_MOTOR_DIR_PIN, OUTPUT );
  pinMode( LEFT_MOTOR_PWM_PIN, OUTPUT );
  pinMode( RIGHT_MOTOR_DIR_PIN, OUTPUT );
  pinMode( RIGHT_MOTOR_PWM_PIN, OUTPUT );

  // Make all sensor pins inputs
  pinMode( 12, INPUT );
  pinMode( 13, INPUT );
  pinMode( A0, INPUT );
  pinMode( A1, INPUT );
  pinMode( A2, INPUT );
  pinMode( A3, INPUT );
  pinMode( A4, INPUT );
  pinMode( A5, INPUT );

  gLastCommandTime = millis();
  /*randomSeed(analogRead(0));*/
  Serial.begin(9600);
}

void loop() {
  if (!gRunFirstTime) {
    run_until_collide();
    scan_best_routes();
  }

  // Turn off the motors if we haven't received a command for a while
  if ( millis() - gLastCommandTime > MOTOR_COMMAND_TIMEOUT_MS ) {
    gLeftMotorDutyCycle = 0;
    gRightMotorDutyCycle = 0;
    gLeftMotorDirection = eMD_Forwards;
    gRightMotorDirection = eMD_Forwards;
  }
  // Update the frequency of the motor PWM
  uint8_t lowestDutyCycle = ( gLeftMotorDutyCycle < gRightMotorDutyCycle
    ? gLeftMotorDutyCycle : gRightMotorDutyCycle );

  // set the motor speed by interpreting MotorDutyCycle into PWM
  if ( lowestDutyCycle <= MOTOR_PWM_20HZ_DUTY_CYCLE ) {
    OCR2 = MOTOR_PWM_20HZ_OCR2;
  } else if ( lowestDutyCycle >= MOTOR_PWM_80HZ_DUTY_CYCLE ) {
    OCR2 = MOTOR_PWM_80HZ_OCR2;
  } else {
    // Linear interpolation from 20Hz to 80Hz
    long int maxOcrChange = MOTOR_PWM_80HZ_OCR2 - MOTOR_PWM_20HZ_OCR2;
    int ocrDiff = (int)(maxOcrChange
      *(long int)((int)lowestDutyCycle - (int)MOTOR_PWM_20HZ_DUTY_CYCLE)
      /(long int)((int)MOTOR_PWM_80HZ_DUTY_CYCLE - (int)MOTOR_PWM_20HZ_DUTY_CYCLE));
      OCR2 = (uint8_t)((int)MOTOR_PWM_20HZ_OCR2 + ocrDiff);
  }
  // Update the servo angles
  gPanServo.writeMicroseconds(gPanServoLimits.convertAngleToPWM(gPanServoAngle));
  gGripServo.writeMicroseconds(gGripServoLimits.convertAngleToPWM(gGripServoAngle));
  // Update the motor directions
  digitalWrite( LEFT_MOTOR_DIR_PIN, (eMD_Forwards == gLeftMotorDirection ? HIGH : LOW));
  digitalWrite( RIGHT_MOTOR_DIR_PIN, (eMD_Forwards == gRightMotorDirection ? HIGH : LOW));
  gRunFirstTime = false;
}

bool is_forward_possible() {
  if (gPanServoAngle == 180) {
    int dist = gTurretIR.distance();
    Serial.print("Turret: ");
    Serial.println(dist);
    if (dist > DANGER_DISTANCE_AHEAD) {
      gLongestDistance = dist;
      return true;
    } else
      return false;
  } else {
    int dist = gFrontIR.distance();
    Serial.println("Front: ");
    Serial.println(dist);
    if (dist > DANGER_DISTANCE_AHEAD) {
      gLongestDistance = dist;
      /*gPanServoAngle = 180;*/
      return true;
    } else
      return false;
  }
}

void led_blink(int ntimes, int t) {
  for (int i=0; i < ntimes; ++i) {
    digitalWrite(LED_PIN, HIGH);
    delay(t);
    digitalWrite(LED_PIN, LOW);
    delay(t);
  }
}

bool is_alternative_route() {
  if (gLongestDistance > DANGER_DISTANCE_AHEAD &&
      gLongestDistance > DANGER_DISTANCE_BEHIND &&
      gBestRelativeAngle > 0) {
    return true;
  } else
    return false;
}

void set_adaptive_speed() {
  if (gLongestDistance <= 200) {
    gLeftMotorDutyCycle = 30;
    gRightMotorDutyCycle = 30;
  } else {
    gLeftMotorDutyCycle = 50;
    gRightMotorDutyCycle = 50;
  }
}

void change_moving_direction() {
  gLeftMotorDirection = eMD_Backwards;
  gRightMotorDirection = eMD_Forwards;
  gLeftMotorDutyCycle = 20;
  gRightMotorDutyCycle = 20;
  gBestRelativeAngle = 0;
  gPanServoAngle = 180;
}

void blind_run() {
  gLeftMotorDirection = eMD_Forwards;
  gRightMotorDirection = eMD_Forwards;
  gLeftMotorDutyCycle = 50;
  gRightMotorDutyCycle = 50;
}

void run_until_collide() {
  if (is_forward_possible()) {
    Serial.println("Forward possible.");
    gLastCommandTime = millis();
    gLeftMotorDirection = eMD_Forwards;
    gRightMotorDirection = eMD_Forwards;
    set_adaptive_speed();
  } else if (is_alternative_route()) {
    Serial.println("Alternative way possible.");
    gLastCommandTime = millis();
    change_moving_direction();
  } else { // stop. I am trapped!
    gLeftMotorDutyCycle = 0;
    gRightMotorDutyCycle = 0;
  }
}

void scan_best_routes() {
  int alternative_dist = gTurretIR.distance();
  int dist = gFrontIR.distance();
  if (alternative_dist > dist) {
    gLongestDistance = alternative_dist;
    gBestRelativeAngle = 180 - gPanServoAngle;
    Serial.print("Front: "); Serial.println(dist);
    Serial.print("Turret: "); Serial.println(alternative_dist);
    Serial.println("Best route found");
  } else {
    if (gPanServoAngle >= 180)
      gTurretDirection = eTD_Left;
    if (gPanServoAngle <= 0)
      gTurretDirection = eTD_Right;
    if (gTurretDirection == eTD_Left)
      gPanServoAngle -= 10;
    else
      gPanServoAngle += 10;
  }
}

void test_frontIR () {
  int dist = gFrontIR.distance();
  Serial.print("Front IR: ");
  Serial.print(dist);
  Serial.print("\n");
}

void test_turretIR () {
  int dist = gTurretIR.distance();
  Serial.print("Turret IR: ");
  Serial.print(dist);
  Serial.print("\n");
}

ISR( TIMER2_COMP_vect ) {
  gCurMotorWaveTick++;
  digitalWrite( LEFT_MOTOR_PWM_PIN, ( gCurMotorWaveTick > gLeftMotorDutyCycle ? LOW : HIGH ) );
  digitalWrite( RIGHT_MOTOR_PWM_PIN, ( gCurMotorWaveTick > gRightMotorDutyCycle ? LOW : HIGH ) );
  if ( gCurMotorWaveTick >= 100 )
    gCurMotorWaveTick = 0;
}
