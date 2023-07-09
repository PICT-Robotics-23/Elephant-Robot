/*   

                      Head
|-------|                               |-------|
|       |                               |       |
|   MA  |                               |   MD  |
|       |                               |       |
|-------|                               |-------|
    |                                       |
    |                                       |
    |                                       |
    |                                       |
    |                                       |
    |                                       |
    |                                       |
    |                                       |
    |                                       |
    |                                       |
    |                                       |
    |                                       |
    |                                       |
|-------|                               |-------|
|       |                               |       |
|   MB  |-------------------------------|   MC  |
|       |                               |       |
|-------|                               |-------|


                    PINOUT

EN	                |		|   23          BLUE 
BA1			        36  |		|	  22	        GREEN
BB1			        39	|		|	  1	          TX	
BA2 			      34 	|		|	  3	          RX	
BB2	  		      35	|		|	  21  	      RED	
LM2_PWM 		    32	|		|	  19	        BM1_DIR
LM2_DIR 		    33	|		|	  18  	      BM1_PWM
LM1_PWM 		    25	|		|	  5	          BM2_DIR	
LM1_DIR 		    26	|		|	  17    	    BM2_PWM
LM4_PWM 		    27	|		|	  16	        AM2_DIR
LM4_DIR 		    14	|		|	  4	          AM2_PWM
LM3_PWM 	  	  12	|		|	  2	          SM1_PWM
LM3_DIR 	  	  13	|		|	  15	        SM2_PWM
GND	                |		|	  GND
VCC	                |		|	  3.3V
		
*/

#include <PS4Controller.h>
#include <CytronMotorDriver.h>

// locomotion
#define loco_dirA_pin 26
#define loco_pwmA_pin 25
#define loco_dirB_pin 33
#define loco_pwmB_pin 32
#define loco_dirC_pin 13
#define loco_pwmC_pin 12
#define loco_dirD_pin 14
#define loco_pwmD_pin 27

int joystick[4] = { 0, 0, 0, 0 };             // left_X. left_y, right_x, right_y
float locomotion_powers[4] = { 0, 0, 0, 0 };  // A, B, C, D

CytronMD motorA(PWM_DIR, loco_pwmA_pin, loco_dirA_pin, 0);  // PWM, DIR
CytronMD motorB(PWM_DIR, loco_pwmB_pin, loco_dirB_pin, 1);
CytronMD motorC(PWM_DIR, loco_pwmC_pin, loco_dirC_pin, 2);
CytronMD motorD(PWM_DIR, loco_pwmD_pin, loco_dirD_pin, 3);

// base flaps
#define baseflapleft_dirpin 19
#define baseflapleft_pwmpin 18
#define baseflapright_dirpin 5
#define baseflapright_pwmpin 17

int base_in = 0;
int base_out = 0;
int left_base_motor_pwm = 0;
float right_base_motor_pwm = 0;
int base_pwm = 100;
float corr_fact = 1.012;

CytronMD leftBase(PWM_DIR, baseflapleft_pwmpin, baseflapleft_dirpin, 4);
CytronMD rightBase(PWM_DIR, baseflapright_pwmpin, baseflapright_dirpin, 5);

// angle
#define angle_pwm_pin 4
#define angle_dir_pin 16

int angle_up = 0;
int angle_down = 0;
int angle_motor_pwm = 0;
int angle_pwm = 255;

CytronMD angle(PWM_DIR, angle_pwm_pin, angle_dir_pin, 6);

// shooting
#define shooting_pwm_pin1 2
#define shooting_pwm_pin2 15

int l1 = 0;
int r1 = 0;
float shooting_speed = 0;

// loading
int load_in = 0;
int load_out = 0;

// picking
int pick_up = 0;
int pick_down = 0;

// general
byte deadband_threshold = 10;
byte i = 0;
int ack = 6;
const long interval = 20; 
unsigned long previousMillis = 0; 



//-----------------------------------------------------------------------------------------------//

void setup() {
  Serial.begin(115200);

  //blue ps4
  PS4.begin("d0:49:7c:9f:da:a1");

  // shooting
  ledcSetup(7, 5000, 8);
  ledcAttachPin(shooting_pwm_pin1, 7);
  ledcSetup(8, 5000, 8);
  ledcAttachPin(shooting_pwm_pin2, 8);
}


//-----------------------------------------------------------------------------------------------//


void loop() {
  unsigned long currentMillis = millis();
  // locomotion
  joystick[0] = PS4.LStickX();
  joystick[1] = PS4.LStickY();
  joystick[2] = PS4.RStickX();
  // base flaps
  base_out = PS4.Down();
  base_in = PS4.Left();
  // angle
  angle_down = PS4.Circle();
  angle_up = PS4.Triangle();
  // shooting
  l1 = PS4.L1();
  r1 = PS4.R1();
  // loading
  load_in = PS4.R2();
  load_out = PS4.L2();
  // picking
  pick_up = PS4.Cross();
  pick_down = PS4.Square();
  // deadband
  
  for (i = 0; i < 4; i++) {
    joystick[i] = deadBand(joystick[i]);
  }

  if (PS4.isConnected()) {

    // locomotion
    locomotion_powers[0] = 2 * (joystick[1] + joystick[0] + joystick[2]);  // A
    locomotion_powers[1] = 2 * (joystick[1] - joystick[0] + joystick[2]);  // B
    locomotion_powers[2] = 2 * (joystick[1] + joystick[0] - joystick[2]);  // C
    locomotion_powers[3] = 2 * (joystick[1] - joystick[0] - joystick[2]);  // D
    // exponential mode
    for (i = 0; i < 4; i++) {
      locomotion_powers[i] = expo(locomotion_powers[i]);
    }

    // base flaps
    left_base_motor_pwm = base_pwm * (base_out || base_in) * (base_in - base_out);
    right_base_motor_pwm = corr_fact *  base_pwm * (base_out || base_in) * (base_out - base_in);

    // angle
    angle_motor_pwm = angle_pwm * (angle_up || angle_down) * (angle_up - angle_down);

    // shooting
    shooting_speed = (shooting_speed + r1 * 0.01 - l1 * 0.01);
    shooting_speed = constrain(shooting_speed, 0, 220);

  } else {
    for (i = 0; i < 4; i++) {
      locomotion_powers[i] = 0;
    }

    left_base_motor_pwm = 0;
    right_base_motor_pwm = 0;
    angle_motor_pwm = 0;
    shooting_speed = 0;
  }

  // drive locomotion
  motorA.setSpeed(locomotion_powers[0]);
  motorB.setSpeed(locomotion_powers[1]);
  motorC.setSpeed(locomotion_powers[2]);
  motorD.setSpeed(locomotion_powers[3]);

  //drive base flaps
  leftBase.setSpeed(left_base_motor_pwm);
  rightBase.setSpeed((int)right_base_motor_pwm);

  // drive angle
  angle.setSpeed(angle_motor_pwm);

  // drive shooting
  ledcWrite(7, shooting_speed);
  ledcWrite(8, shooting_speed);

  // send UART data
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    sendData();
  }

  // display data
  // displayData();
}


//-----------------------------------------------------------------------------------------------//


int deadBand(int a) {
  if (a < deadband_threshold && a > -deadband_threshold) {
    a = 0;
  }
  return a;
}

float expo(float a) {
  if (a < 130 && a > -130) {
    a = a / 1.5;
  } else if (a >= 130 && a < 255) {
    a = (a * 1.36) - 91.8;
  } else if (a <= -130 && a > -255) {
    a = (a * 1.36) + 91.8;
  }
  return (a);
}

void sendData(){
  Serial.print(ack);
  Serial.print("u");
  Serial.print(pick_up);
  Serial.print("d");
  Serial.print(pick_down);
  Serial.print("i");
  Serial.print(load_in);
  Serial.print("o");
  Serial.print(load_out);
  Serial.print("s");
  Serial.println((int)shooting_speed);
}

void displayData(){
  Serial.print(load_in);
  Serial.print("\t");
  Serial.print(pick_up);
  Serial.print("\t");
  Serial.print(pick_down);
  Serial.print("\t");
  Serial.print(angle_up);
  Serial.print("\t");
  Serial.print(angle_down);
  Serial.print("\t");
  Serial.print(l1);
  Serial.print("\t");
  Serial.print(r1);
  Serial.print("\t");
  Serial.print(base_in);
  Serial.print("\t");
  Serial.println(base_out);
}
//-----------------------------------------------------------------------------------------------//