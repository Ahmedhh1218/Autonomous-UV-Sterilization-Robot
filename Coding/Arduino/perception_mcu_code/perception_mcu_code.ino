#include <ros.h>
#include <std_msgs/Int8.h>

#define RIGHT_IR_PIN 2
#define FRONT_RIGHT_IR_PIN 3
#define LEFT_IR_PIN 4
#define FRONT_LEFT_IR_PIN 5

ros::NodeHandle nh;
std_msgs::Int8 flagMsg;
ros::Publisher flagPublisher("/ir_flag", &flagMsg);

void setup() {
  
  nh.initNode();
  nh.advertise(flagPublisher);
  pinMode(RIGHT_IR_PIN, INPUT);
  pinMode(FRONT_RIGHT_IR_PIN, INPUT);
  pinMode(LEFT_IR_PIN, INPUT);
  pinMode(FRONT_LEFT_IR_PIN, INPUT);
}

void loop() {
  int rightIR = digitalRead(RIGHT_IR_PIN);
  int frontRightIR = digitalRead(FRONT_RIGHT_IR_PIN);
  int leftIR = digitalRead(LEFT_IR_PIN);
  int frontLeftIR = digitalRead(FRONT_LEFT_IR_PIN);

  if (frontLeftIR == 0 || frontRightIR == 0) 
  {
    flagMsg.data = 1;
  }
//  else if (leftIR == 0) 
//  {
//    flagMsg.data = 2;
//  }
//
//  else if (rightIR == 0) 
//  {
//    flagMsg.data = 3;
//  }
  else 
  {
    flagMsg.data = 0;
  }
  flagPublisher.publish(&flagMsg);

  nh.spinOnce();
  delay(100);
}
