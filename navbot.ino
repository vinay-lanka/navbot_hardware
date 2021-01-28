//AGV Machine - Vinay Lanka

//Import Motor - Cytron SPG30E-30K
#include "Motor.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include<PID_v1.h>

ros::NodeHandle  nh;

Motor right(11,10,20,21);
Motor left(9,8,18,19);

volatile long encoder0Pos = 0;    // encoder 1
volatile long encoder1Pos = 0;    // encoder 2

double left_kp = 3.8 , left_ki = 0 , left_kd = 0.03;             // modify for optimal performance
double right_kp = 4 , right_ki = 0 , right_kd = 0.03;

double right_input = 0, right_output = 0, right_setpoint = 0;
PID rightPID(&right_input, &right_output, &right_setpoint, right_kp, right_ki, right_kd, DIRECT);  

double left_input = 0, left_output = 0, left_setpoint = 0;
PID leftPID(&left_input, &left_output, &left_setpoint, left_kp, left_ki, left_kd, DIRECT);  

float demandx=0;
float demandz=0;

double demand_speed_left;
double demand_speed_right;

unsigned long currentMillis;
unsigned long prevMillis;

float encoder0Diff;
float encoder1Diff;

float encoder0Error;
float encoder1Error;

float encoder0Prev;
float encoder1Prev;

void cmd_vel_cb( const geometry_msgs::Twist& twist){
  demandx = twist.linear.x;
  demandz = twist.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb );

void setup() {
  nh.initNode();
  nh.subscribe(sub);
//  Serial.begin(115200);
  
  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(1);
  rightPID.SetOutputLimits(-100, 100);

  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(1);
  leftPID.SetOutputLimits(-100, 100);
  
//  Serial.println("Basic Encoder Test:");
  attachInterrupt(digitalPinToInterrupt(left.en_a), change_left_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left.en_b), change_left_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right.en_a), change_right_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right.en_b), change_right_b, CHANGE);
}

void loop() {
  currentMillis = millis();
  if (currentMillis - prevMillis >= 10){
    prevMillis = currentMillis;
//    if(Serial.available()>0){
//      char c = Serial.read();
//      if(c == 'a'){
//        demandx = 0.5;
//        demandz=0;
//      }else if (c == 'b'){
//        demandx = -0.5;
//        demandz = 0;
//      }else if(c == 'c'){
//        demandx = 0;
//        demandz = 1;
//      }else if(c == 'd'){
//        demandx = 0;
//        demandz = 1;
//      }else if(c == 'e'){
//        demandx = 0;
//        demandz = 0;
//      }
//    }
    demand_speed_left = demandx - (demandz*0.1075);
    demand_speed_right = demandx + (demandz*0.1075);
  
    /*PID controller for speed control
      Base speed being 1 ms and the demand_speed variables controlling it at fractions of the base.
      The PID controller keeps trying to match the difference 
      in encoder counts to match with the required amount, hence controlling the speed. */
    encoder0Diff = encoder0Pos - encoder0Prev; // Get difference between ticks to compute speed
    encoder1Diff = encoder1Pos - encoder1Prev;
  
    encoder0Error = (demand_speed_left*39.65)-encoder0Diff; // 3965 ticks in 1m = 39.65 ticks in 10ms, due to the 10 millis loop
    encoder1Error = (demand_speed_right*39.65)-encoder1Diff;
  
    encoder0Prev = encoder0Pos; // Saving values
    encoder1Prev = encoder1Pos;
  
    left_setpoint = demand_speed_left*39.65;  //Setting required speed as a mul/frac of 1 m/s
    right_setpoint = demand_speed_right*39.65;
  
    left_input = encoder0Diff;  //Input to PID controller is the current difference
    right_input = encoder1Diff;
    
    leftPID.Compute();
    left.rotate(left_output);
    rightPID.Compute();
    right.rotate(right_output);
//    Serial.print(encoder0Pos);
//    Serial.print(",");
//    Serial.println(encoder1Pos);
  }
  
}



// ************** encoders interrupts **************

// ************** encoder 1 *********************


void change_left_a(){  

  // look for a low-to-high on channel A
  if (digitalRead(left.en_a) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(left.en_b) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(left.en_b) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
 
}

void change_left_b(){  

  // look for a low-to-high on channel B
  if (digitalRead(left.en_b) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(left.en_a) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(left.en_a) == LOW) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  

}

// ************** encoder 2 *********************

void change_right_a(){  

  // look for a low-to-high on channel A
  if (digitalRead(right.en_a) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(right.en_b) == LOW) {  
      encoder1Pos = encoder1Pos - 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(right.en_b) == HIGH) {   
      encoder1Pos = encoder1Pos - 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;          // CCW
    }
  }
 
}

void change_right_b(){  

  // look for a low-to-high on channel B
  if (digitalRead(right.en_b) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(right.en_a) == HIGH) {  
      encoder1Pos = encoder1Pos - 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(right.en_a) == LOW) {   
      encoder1Pos = encoder1Pos - 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;          // CCW
    }
  }
  

}
