#include <util/atomic.h>

// Pins
#define ENCA 2
#define ENCB 3
#define IN1 4
#define IN2 5
#define PWM 6

// globals
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float maxspeed=200;
float eintegral = 0;

void setup() {
  Serial.begin(19200);

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA),
                  readEncoder,RISING);
}

void loop() {

  // read the position in an atomic block
  // to avoid potential misreads
  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
  }

  // Compute velocity
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  float velocity = (pos - posPrev) / deltaT;
  posPrev = pos;
  prevT_i = currT;

  // Convert count/s to RPM
  float v1 = velocity / 600.0 * 60.0;

  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev;
  v1Prev = v1;

  // Set a target
  float target = 0.9*maxspeed;

  // Compute the control signal u
  float kp = 5;
  float ki = 0;
  float e = target - v1Filt;
  eintegral = eintegral + e * deltaT;

  float u = kp * e + ki * eintegral;

  // Set the motor speed and direction
  int pwr = (int)fabs(u);
  if (pwr > 255) {
    pwr = 255;
  }
  setMotor(pwr, PWM, IN1, IN2);

  // Serial.print(target);
  Serial.print(" ");
  Serial.print(v1Filt);
  Serial.println();
  delay(100);
}

void setMotor(int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal); // Motor speed
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void readEncoder() {
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB);
  int increment = 0;
  if (b > 0) {
    // If B is high, increment forward
    increment = 1;
  } else {
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;
}
