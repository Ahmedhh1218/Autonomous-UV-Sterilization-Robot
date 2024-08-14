#include <CleanRTOS.h>        // Include CleanRTOS library for multitasking
#include <PID_v1_bc.h>        // Include PID library for PID control
#include <ESP32Encoder.h>     // Include ESP32 encoder library for motor encoder readings
#include <Wire.h>             // Include Wire library for I2C communication
#include <ros.h>
#include <std_msgs/Int8.h>

// Define motor pins for each motor
#define FRW_PWM_PIN       0  // PWM pin for forward right wheel
#define FRW_IN1_PIN       25  // Motor driver input pin 1 for forward right wheel
#define FRW_IN2_PIN       26  // Motor driver input pin 2 for forward right wheel

#define FLW_PWM_PIN       21  // PWM pin for forward left wheel
#define FLW_IN1_PIN       22  // Motor driver input pin 1 for forward left wheel
#define FLW_IN2_PIN       23  // Motor driver input pin 2 for forward left wheel

#define BRW_PWM_PIN       12  // PWM pin for backward right wheel
#define BRW_IN1_PIN       14  // Motor driver input pin 1 for backward right wheel
#define BRW_IN2_PIN       27  // Motor driver input pin 2 for backward right wheel

#define BLW_PWM_PIN       13  // PWM pin for backward left wheel
#define BLW_IN1_PIN       2   // Motor driver input pin 1 for backward left wheel
#define BLW_IN2_PIN       15  // Motor driver input pin 2 for backward left wheel

#define LEFT_IR           34
#define MIDDLE_IR         36
#define RIGHT_IR          35

int left_ir_state;
int middle_ir_state;
int right_ir_state;

#define black             1
#define white             0

#define no_front_obstacle       0
#define front_dynamic_obstacle  1
#define front_static_obstacle   2

#define no_right_obstacle       3
#define right_dynamic_obstacle  4
#define right_static_obstacle   5

#define no_left_obstacle        6
#define left_dynamic_obstacle   7
#define left_static_obstacle    8

int obstacle_state = no_front_obstacle;

// Flags Definition
#define STOP_STATE              1
#define FORWARD_STATE           2
#define BACKWARD_STATE          3
#define RIGHT_STATE             4
#define LEFT_STATE              5
#define CLOCKWISE_STATE         6
#define COUNTER_CLOCKWISE_STATE 7
#define SLIGHT_RIGHT_STATE      8
#define SLIGHT_LEFT_STATE       9
            
int current_state;
int sterilization_action = 0;
int current_behaviour = 0;

// Define task handles for each motor
TaskHandle_t LineFollower_TaskHandle, FRW_pidTaskHandle, FLW_pidTaskHandle, BRW_pidTaskHandle, BLW_pidTaskHandle;

// Function prototype for callback function
void irFlagCallback(const std_msgs::Int8& msg);
void Scanned_Callback(const std_msgs::Int8& msg);

// ROS node handle
ros::NodeHandle nh;

// ROS subscriber for ir_flag topic
std_msgs::Int8 ir_flag_msg;
ros::Subscriber<std_msgs::Int8> ir_flag_sub("ir_flag", &irFlagCallback);

//Subsriber for camera
std_msgs::Int8 Scanned_Aruco;
ros::Subscriber<std_msgs::Int8> Scanning_Aruco("aruco_marker_flag", &Scanned_Callback);

//Publisher for camera
std_msgs::Int8 Scan_Aruco;
ros::Publisher pub("/CameraCheck",&Scan_Aruco);

// Tasks prototypes
void LineFollower_Task(void *pvParameters);
void FRW_pidTask(void *pvParameters);
void FLW_pidTask(void *pvParameters);
void BRW_pidTask(void *pvParameters);
void BLW_pidTask(void *pvParameters);

void setup() {
  
  nh.initNode();
  nh.subscribe(ir_flag_sub);
  nh.advertise(pub);
  nh.subscribe(Scanning_Aruco);
  
  pinMode(LEFT_IR, INPUT);
  pinMode(MIDDLE_IR, INPUT);
  pinMode(RIGHT_IR, INPUT);

  pinMode(FRW_PWM_PIN, OUTPUT);
  pinMode(FRW_IN1_PIN, OUTPUT);
  pinMode(FRW_IN2_PIN, OUTPUT);

  pinMode(FLW_PWM_PIN, OUTPUT);
  pinMode(FLW_IN1_PIN, OUTPUT);
  pinMode(FLW_IN2_PIN, OUTPUT);

  pinMode(BRW_PWM_PIN, OUTPUT);
  pinMode(BRW_IN1_PIN, OUTPUT);
  pinMode(BRW_IN2_PIN, OUTPUT);

  pinMode(BLW_PWM_PIN, OUTPUT);
  pinMode(BLW_IN1_PIN, OUTPUT);
  pinMode(BLW_IN2_PIN, OUTPUT);

  // Create line follower leading task
  xTaskCreate(
    LineFollower_Task,        // Task function
    "LineFollower_Task",      // Task name
    10000,                    // Stack size
    NULL,                     // Task parameters
    1,                        // Priority
    &LineFollower_TaskHandle  // Task handle
  );

  // Create tasks for each motor
  xTaskCreate(
    FRW_pidTask,              // Task function
    "FRW_PID_Task",           // Task name
    10000,                    // Stack size
    NULL,                     // Task parameters
    1,                        // Priority
    &FRW_pidTaskHandle        // Task handle
  );
  // Create tasks for each motor
  xTaskCreate(
    FLW_pidTask,              // Task function
    "FLW_PID_Task",           // Task name
    10000,                    // Stack size
    NULL,                     // Task parameters
    1,                        // Priority
    &FLW_pidTaskHandle        // Task handle
  );
  xTaskCreate(
    BRW_pidTask,              // Task function
    "BRW_PID_Task",           // Task name
    10000,                    // Stack size
    NULL,                     // Task parameters
    1,                        // Priority
    &BRW_pidTaskHandle        // Task handle
  );
  xTaskCreate(
    BLW_pidTask,              // Task function
    "BLW_PID_Task",           // Task name
    10000,                    // Stack size
    NULL,                     // Task parameters
    1,                        // Priority
    &BLW_pidTaskHandle        // Task handle
  );
}

void loop() {
  // Main loop, tasks are handled by CleanRTOS tasks
}

void irFlagCallback(const std_msgs::Int8& msg) {
  // Update current_state based on received message
  if (msg.data == 1) 
  {
    obstacle_state = front_dynamic_obstacle;
  } 
  else if (msg.data == 0)
  {
    obstacle_state = no_front_obstacle;
  }
}

void Scanned_Callback(const std_msgs::Int8& msg) 
{
  if(msg.data==1)
  {
    sterilization_action = 1;
  }
  else if (msg.data == 2)
  {
    sterilization_action = 2;
  }
}


void LineFollower_Task(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    Scan_Aruco.data = 0;
    right_ir_state = digitalRead(RIGHT_IR);
    left_ir_state = digitalRead(LEFT_IR);
    
    if (sterilization_action == 0)
    {
      if (obstacle_state == no_front_obstacle)
      {
        if ((right_ir_state & left_ir_state) == black)
        {
          current_state = STOP_STATE;
          delay(500);
          Scan_Aruco.data = 1;
        }
        else if ( ((left_ir_state | right_ir_state) == white))
        {
          current_state = FORWARD_STATE;
        }
        else if ((left_ir_state == black) && ((right_ir_state) == white))
        {
          current_state = SLIGHT_LEFT_STATE;
        }
        else if ((right_ir_state == black) && ((left_ir_state) == white))
        {
          current_state = SLIGHT_RIGHT_STATE;
        }
      }
      else if (obstacle_state == front_dynamic_obstacle)
      {
        current_state = STOP_STATE;
      }
    }
    
    else if (sterilization_action == 1)
    {
      switch (current_behaviour)
      {
        case 0:
          current_state = FORWARD_STATE;
          delay(1000);
          current_state = CLOCKWISE_STATE;
          delay(2600);
          current_behaviour = 1;
          break;
        case 1:
          if (obstacle_state == no_front_obstacle)
          {
            if ((right_ir_state & left_ir_state) == black)
            {
              current_state = STOP_STATE;
              current_behaviour = 2;
            }
            else if ( ((left_ir_state | right_ir_state) == white))
            {
              current_state = FORWARD_STATE;
            }
            else if ((left_ir_state == black) && ((right_ir_state) == white))
            {
              current_state = SLIGHT_LEFT_STATE;
            }
            else if ((right_ir_state == black) && ((left_ir_state) == white))
            {
              current_state = SLIGHT_RIGHT_STATE;
            }
          }
          else if (obstacle_state == front_dynamic_obstacle)
          {
            current_state = STOP_STATE;
          }
          break;
        case 2:
          current_state = STOP_STATE;
          delay(3000);
          current_state = CLOCKWISE_STATE;
          delay(5300);
          current_behaviour = 3;
          break;
        case 3:
          if (obstacle_state == no_front_obstacle)
          {
            if ((right_ir_state & left_ir_state) == black)
            {
              current_state = STOP_STATE;
              current_behaviour = 4;
            }
            else if ( ((left_ir_state | right_ir_state) == white))
            {
              current_state = FORWARD_STATE;
            }
            else if ((left_ir_state == black) && ((right_ir_state) == white))
            {
              current_state = SLIGHT_LEFT_STATE;
            }
            else if ((right_ir_state == black) && ((left_ir_state) == white))
            {
              current_state = SLIGHT_RIGHT_STATE;
            }
          }
          else if (obstacle_state == front_dynamic_obstacle)
          {
            current_state = STOP_STATE;
          }
          break;
        case 4:
          current_state = STOP_STATE;
          delay(1000);
          current_state = FORWARD_STATE;
          delay(900);
          current_state = STOP_STATE;
          delay(1000);
          current_state = CLOCKWISE_STATE;
          delay(2850);
          current_behaviour = 0;
          sterilization_action = 0;
          break;
      }      
    }
    else if (sterilization_action == 2)
    {
      current_state = FORWARD_STATE;
      delay(1000);
      sterilization_action = 0;
    }
    pub.publish(&Scan_Aruco);
    nh.spinOnce();
    vTaskDelay(20);
  }
}

// Task to control FRW motor using PID
void FRW_pidTask(void *pvParameters) {
  (void)  pvParameters;

  for (;;) {
    if (current_state == STOP_STATE)
    {
      digitalWrite(FRW_IN1_PIN, LOW);
      digitalWrite(FRW_IN2_PIN, LOW);
      analogWrite(FRW_PWM_PIN, 0);
    }
    else if (current_state == FORWARD_STATE) 
    {
      digitalWrite(FRW_IN1_PIN, HIGH);      // Forward direction
      digitalWrite(FRW_IN2_PIN, LOW);
      analogWrite(FRW_PWM_PIN, 120);
    } 
    else if(current_state == BACKWARD_STATE)
    {
      digitalWrite(FRW_IN1_PIN, LOW);       // Reverse direction
      digitalWrite(FRW_IN2_PIN, HIGH);
      analogWrite(FRW_PWM_PIN, 120);
    }
    else if(current_state == RIGHT_STATE)
    {
      digitalWrite(FRW_IN1_PIN, LOW);       // Reverse direction
      digitalWrite(FRW_IN2_PIN, HIGH);
      analogWrite(FRW_PWM_PIN, 220);
    }
    else if(current_state == LEFT_STATE)
    {
      digitalWrite(FRW_IN1_PIN, HIGH);      // Forward direction
      digitalWrite(FRW_IN2_PIN, LOW);
      analogWrite(FRW_PWM_PIN, 220);
    }
    else if(current_state == CLOCKWISE_STATE)
    {
      digitalWrite(FRW_IN1_PIN, LOW);       // Reverse direction
      digitalWrite(FRW_IN2_PIN, HIGH);
      analogWrite(FRW_PWM_PIN, 120);
    }
    else if(current_state == COUNTER_CLOCKWISE_STATE)
    {
      digitalWrite(FRW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(FRW_IN2_PIN, LOW);
      analogWrite(FRW_PWM_PIN, 120);
    }
    else if(current_state == SLIGHT_RIGHT_STATE)
    {
      digitalWrite(FRW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(FRW_IN2_PIN, LOW);
      analogWrite(FRW_PWM_PIN, 50);
    }
    else if(current_state == SLIGHT_LEFT_STATE)
    {
      digitalWrite(FRW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(FRW_IN2_PIN, LOW);
      analogWrite(FRW_PWM_PIN, 120);
    }
    nh.spinOnce();
    // Delay for task execution
    vTaskDelay(20);
  }
}

// Task to control FLW motor using PID
void FLW_pidTask(void *pvParameters) {
  (void)  pvParameters;

  for (;;) {
    if (current_state == STOP_STATE)
    {
      digitalWrite(FLW_IN1_PIN, LOW);
      digitalWrite(FLW_IN2_PIN, LOW);
      analogWrite(FLW_PWM_PIN, 0);
    }
    else if (current_state == FORWARD_STATE) 
    {
      digitalWrite(FLW_IN1_PIN, HIGH);      // Forward direction
      digitalWrite(FLW_IN2_PIN, LOW);
      analogWrite(FLW_PWM_PIN, 125);
    } 
    else if(current_state == BACKWARD_STATE)
    {
      digitalWrite(FLW_IN1_PIN, LOW);       // Reverse direction
      digitalWrite(FLW_IN2_PIN, HIGH);
      analogWrite(FLW_PWM_PIN, 125);
    }
    else if(current_state == RIGHT_STATE)
    {
      digitalWrite(FLW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(FLW_IN2_PIN, LOW);
      analogWrite(FLW_PWM_PIN, 225);
    }
    else if(current_state == LEFT_STATE)
    {
      digitalWrite(FLW_IN1_PIN, LOW);       // Reverse direction
      digitalWrite(FLW_IN2_PIN, HIGH);
      analogWrite(FLW_PWM_PIN, 225);
    }
    else if(current_state == CLOCKWISE_STATE)
    {
      digitalWrite(FLW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(FLW_IN2_PIN, LOW);
      analogWrite(FLW_PWM_PIN, 125);
    }
    else if(current_state == COUNTER_CLOCKWISE_STATE)
    {
      digitalWrite(FLW_IN1_PIN, LOW);       // Reverse direction
      digitalWrite(FLW_IN2_PIN, HIGH);
      analogWrite(FLW_PWM_PIN, 125);
    }
    else if(current_state == SLIGHT_RIGHT_STATE)
    {
      digitalWrite(FLW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(FLW_IN2_PIN, LOW);
      analogWrite(FLW_PWM_PIN, 150);
    }
    else if(current_state == SLIGHT_LEFT_STATE)
    {
      digitalWrite(FLW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(FLW_IN2_PIN, LOW);
      analogWrite(FLW_PWM_PIN, 80);
    }
    nh.spinOnce();
    // Delay for task execution
    vTaskDelay(20);
  }
}

// Task to control BRW motor using PID
void BRW_pidTask(void *pvParameters) {
  (void)  pvParameters;

  for (;;) {
    if (current_state == STOP_STATE)
    {
      digitalWrite(BRW_IN1_PIN, LOW);
      digitalWrite(BRW_IN2_PIN, LOW);
      analogWrite(BRW_PWM_PIN, 0);
    }
    else if (current_state == FORWARD_STATE) 
    {
      digitalWrite(BRW_IN1_PIN, HIGH);      // Forward direction
      digitalWrite(BRW_IN2_PIN, LOW);
      analogWrite(BRW_PWM_PIN, 120);
    } 
    else if(current_state == BACKWARD_STATE)
    {
      digitalWrite(BRW_IN1_PIN, LOW);       // Reverse direction
      digitalWrite(BRW_IN2_PIN, HIGH);
      analogWrite(BRW_PWM_PIN, 120);
    }
    else if(current_state == RIGHT_STATE)
    {
      digitalWrite(BRW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(BRW_IN2_PIN, LOW);
      analogWrite(BRW_PWM_PIN, 220);
    }
    else if(current_state == LEFT_STATE)
    {
      digitalWrite(BRW_IN1_PIN, LOW);       // Reverse direction
      digitalWrite(BRW_IN2_PIN, HIGH);
      analogWrite(BRW_PWM_PIN, 220);
    }
    else if(current_state == CLOCKWISE_STATE)
    {
      digitalWrite(BRW_IN1_PIN, LOW);       // Reverse direction
      digitalWrite(BRW_IN2_PIN, HIGH);
      analogWrite(BRW_PWM_PIN, 120);
    }
    else if(current_state == COUNTER_CLOCKWISE_STATE)
    {
      digitalWrite(BRW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(BRW_IN2_PIN, LOW);
      analogWrite(BRW_PWM_PIN, 120);
    }
    else if(current_state == SLIGHT_RIGHT_STATE)
    {
      digitalWrite(BRW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(BRW_IN2_PIN, LOW);
      analogWrite(BRW_PWM_PIN, 50);
    }
    else if(current_state == SLIGHT_LEFT_STATE)
    {
      digitalWrite(BRW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(BRW_IN2_PIN, LOW);
      analogWrite(BRW_PWM_PIN, 120);
    }
    nh.spinOnce();
    // Delay for task execution
    vTaskDelay(20);
  }
}

// Task to control FLW motor using PID
void BLW_pidTask(void *pvParameters) {
  (void)  pvParameters;

  for (;;) {
    if (current_state == STOP_STATE)
    {
      digitalWrite(BLW_IN1_PIN, LOW);
      digitalWrite(BLW_IN2_PIN, LOW);
      analogWrite(BLW_PWM_PIN, 0);
    }
    else if (current_state == FORWARD_STATE) 
    {
      digitalWrite(BLW_IN1_PIN, HIGH);      // Forward direction
      digitalWrite(BLW_IN2_PIN, LOW);
      analogWrite(BLW_PWM_PIN, 150);
    } 
    else if(current_state == BACKWARD_STATE)
    {
      digitalWrite(BLW_IN1_PIN, LOW);       // Reverse direction
      digitalWrite(BLW_IN2_PIN, HIGH);
      analogWrite(BLW_PWM_PIN, 150);
    }
    else if(current_state == RIGHT_STATE)
    {
      digitalWrite(BLW_IN1_PIN, LOW);       // Reverse direction
      digitalWrite(BLW_IN2_PIN, HIGH);
      analogWrite(BLW_PWM_PIN, 250);
    }
    else if(current_state == LEFT_STATE)
    {
      digitalWrite(BLW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(BLW_IN2_PIN, LOW);
      analogWrite(BLW_PWM_PIN, 250);
    }
    else if(current_state == CLOCKWISE_STATE)
    {
      digitalWrite(BLW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(BLW_IN2_PIN, LOW);
      analogWrite(BLW_PWM_PIN, 150);
    }
    else if(current_state == COUNTER_CLOCKWISE_STATE)
    {
      digitalWrite(BLW_IN1_PIN, LOW);       // Reverse direction
      digitalWrite(BLW_IN2_PIN, HIGH);
      analogWrite(BLW_PWM_PIN, 150);
    }
    else if(current_state == SLIGHT_RIGHT_STATE)
    {
      digitalWrite(BLW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(BLW_IN2_PIN, LOW);
      analogWrite(BLW_PWM_PIN, 175);
    }
    else if(current_state == SLIGHT_LEFT_STATE)
    {
      digitalWrite(BLW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(BLW_IN2_PIN, LOW);
      analogWrite(BLW_PWM_PIN, 100);
    }
    nh.spinOnce();
    // Delay for task execution
    vTaskDelay(20);
  }
}
