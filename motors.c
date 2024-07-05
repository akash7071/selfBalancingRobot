/* ========================================================================== */
/*                                                                            */
//
//
//Edited by Akash Patil and Vidhya Palaniappan
//Edited on: 5/5/2024
//Edited for RTES Project: Self-balancing robot prototype
//References:https://github.com/wennycooper/mySegway/tree/master
//
/* ========================================================================== */


#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <softPwm.h>

#define RANGE	250             ///max value for denominator of PWM
#define INITIAL_VALUE 0         //start pwm at 0% duty cycle

#define AIN1 0                  //direction control pin for left motor
#define PWMA 26                 //speed control pin for left motor
#define BIN1 3                  //direction control pin for right motor
#define PWMB 23                 //speed control pin for right motor
#define STBY 6                  //enable pin for driver IC


//init GPIO for motors, digital and PWM HW module
void init_motors()
{
  wiringPiSetup();

  //PWM for left and right motor
  pinMode(PWMA,PWM_OUTPUT);
  pinMode(PWMB,PWM_OUTPUT);
  
  //digital pin init for left motor
  pinMode(AIN1, OUTPUT);
  digitalWrite(AIN1, HIGH);
  
  
  //digital pin init for right motor
  pinMode(BIN1, OUTPUT);
  digitalWrite(BIN1, HIGH);
  
  //enable motor driver IC
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  pwmSetRange(RANGE); // PWM range (1000 steps)
  pwmSetClock(192); // PWM clock divisor (19.2 MHz / 192 = 100 kHz)
}

//function to stop all motors by setting speed to zero
void stop_motors()
{
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  digitalWrite(PWMA, LOW);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  digitalWrite(PWMB, LOW);

  // exit(1);

}


//function to drive motor forward by setting both direction pins to HIGH and some finite speed
void move_forward()
{
  pwmWrite(PWMA, 70);
  pwmWrite(PWMB, 70);
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, HIGH);
    
    
}


double left_speed;
double right_speed;


//function to set speed of motor and direction
void motors(double speed, double left_offset, double right_offset)
{



//calibrate for offsets
  left_speed = speed + left_offset;
  right_speed = speed + right_offset;

  // drive left motor backward
  if (left_speed < 0)  
  {
    //softPwmWrite(PWMA, (int) -left_speed);
    pwmWrite(PWMA,(int) -left_speed);
    digitalWrite(AIN1, HIGH);
  }
  //drive left motor forward
  else
  if (left_speed > 0)  
  {
    
    //softPwmWrite(PWMA, left_speed);
    pwmWrite(PWMA, left_speed);
    digitalWrite(AIN1, LOW);
    
  }

  // drive right motor backward
  if (right_speed < 0)  
  {
    pwmWrite(PWMB, (int) -right_speed);    
    digitalWrite(BIN1, HIGH);
    
  }
  //drive right motor forward
  else
  if (right_speed > 0)  
  {
    
    pwmWrite(PWMB, right_speed);
    digitalWrite(BIN1, LOW);
  }
}



