#include <ros.h>
#include <delta_arduino/cmdAngle.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <Arduino.h>
#include <AccelStepper.h>



#define TRUE                1
#define FALSE               0
#define LED_PIN             13
#define A_STEP_PIN         	54
#define A_DIR_PIN          	55
#define A_ENABLE_PIN       	38
#define A_END_PIN           3
#define B_STEP_PIN         	60
#define B_DIR_PIN          	61
#define B_ENABLE_PIN       	56
#define B_END_PIN          	14
#define C_STEP_PIN        	26
#define C_DIR_PIN          	28
#define C_ENABLE_PIN        24
#define C_END_PIN 	        18

#define EE_RADIUS           30
#define BICEPS              100
#define BASE_RADIUS         80
#define FOREARM             296.71
#define STEPMODE            8
#define RESETANGLE          11


ros::NodeHandle nh;

sensor_msgs::JointState joint_state;

ros::Publisher JointState("delta/joint_state",&joint_state);


static float stepsCircle = 400 * STEPMODE;
float x0,y0,z0;
bool a_set_reset = false, b_set_reset = false, c_set_reset = false;
bool set_reset = false;
bool enable = false;
int write_freq=0;
float factor =0.15;
int loop_iteration = 0;
int sendRate = 5;
bool sendWSMsg = true;

enum state
{
    STATE_OFF,
    STATE_INIT,
    STATE_RESET,
    STATE_WAITING,
    STATE_MOVING,

}state_;
state oldstate_;

AccelStepper 	a_stepper(AccelStepper::DRIVER, A_STEP_PIN, A_DIR_PIN);
AccelStepper 	b_stepper(AccelStepper::DRIVER, B_STEP_PIN, B_DIR_PIN);
AccelStepper 	c_stepper(AccelStepper::DRIVER, C_STEP_PIN, C_DIR_PIN);

void initialize(AccelStepper *stepper, int enable, int end) {
  stepper->setEnablePin(enable);
  stepper->setMinPulseWidth(20);
  stepper->setPinsInverted(false, false, true);
  stepper->setMaxSpeed(stepsCircle*factor);
  stepper->setSpeed(stepsCircle*(factor-0.05));
  stepper->setAcceleration(15000);
  pinMode(end, INPUT);
}
void startCtrl(){
  digitalWrite(LED_PIN, HIGH);
  enable = true;

  a_stepper.enableOutputs();
  b_stepper.enableOutputs();
  c_stepper.enableOutputs();
  nh.loginfo("Motor Control enabled");
}
void stopCtrl(){
  digitalWrite(LED_PIN, LOW);
  enable = false;
  a_stepper.disableOutputs();
  b_stepper.disableOutputs();
  c_stepper.disableOutputs();
  nh.loginfo("Motor Control disabled");
}
void sendAngle(){
  char output[64];
  float a_angle=360/stepsCircle*a_stepper.currentPosition();
  float b_angle=360/stepsCircle*b_stepper.currentPosition();
  float c_angle=360/stepsCircle*c_stepper.currentPosition();
  char angle1str[10];
  char angle2str[10];
  char angle3str[10];

  dtostrf(a_angle, 4, 5, angle1str);
  dtostrf(b_angle, 4, 5, angle2str);
  dtostrf(c_angle, 4, 5, angle3str);
  sprintf(output,"POS %s %s %s",angle1str,angle2str,angle3str);

}
void printAngles(char *output){
  char a_angle_str[10];
  char b_angle_str[10];
  char c_angle_str[10];
  float a_angle = -a_stepper.currentPosition()*360/stepsCircle;
  float b_angle = -a_stepper.currentPosition()*360/stepsCircle;
  float c_angle = -a_stepper.currentPosition()*360/stepsCircle;
  dtostrf(a_angle, 4, 2, a_angle_str);
  dtostrf(b_angle, 4, 2, b_angle_str);
  dtostrf(c_angle, 4, 2, c_angle_str);
  sprintf(output,"t1: %s t2: %s t3: %s",a_angle_str,b_angle_str,c_angle_str);
}
void checkEndstops(){
  //angle to set robot on top of workspace -15.4315 is for z=-285
  //float angleForZ=-15.4315;
  if (digitalRead(A_END_PIN) == LOW){
    a_stepper.setCurrentPosition(0);
    if (set_reset) a_stepper.moveTo(-RESETANGLE*stepsCircle/360);
  }
  if (digitalRead(B_END_PIN) == LOW){
    b_stepper.setCurrentPosition(0);
    if (set_reset) b_stepper.moveTo(-RESETANGLE*stepsCircle/360);
  }
  if (digitalRead(C_END_PIN) == LOW){
    c_stepper.setCurrentPosition(0);
    if (set_reset) c_stepper.moveTo(-RESETANGLE*stepsCircle/360);
  }
}
bool checkWorkspace(){
  float a_angle = -a_stepper.currentPosition()*360/stepsCircle;
  float b_angle = -b_stepper.currentPosition()*360/stepsCircle;
  float c_angle = -c_stepper.currentPosition()*360/stepsCircle;

  if (state_ == STATE_RESET) return true;
  else if (a_angle <= 90.0 && b_angle <= 90.0 && c_angle <= 90.0){
    if (a_angle >= 0.0 && b_angle >= 0.0 && c_angle >= 0.0){
      return true;
    }
    else return false;
  }
  else return false;
}

void moveMotorTo(const delta_arduino::cmdAngle& cmdAngle){

  if(enable){
    if(state_ = STATE_WAITING){

      float cmd_theta1 = cmdAngle.theta1 * stepsCircle/360;
      float cmd_theta2 = cmdAngle.theta2 * stepsCircle/360;
      float cmd_theta3 = cmdAngle.theta3 * stepsCircle/360;

      float act_theta1 = a_stepper.currentPosition();
      float act_theta2 = b_stepper.currentPosition();
      float act_theta3 = c_stepper.currentPosition();

      float theta1_diff = cmd_theta1 - act_theta1;
      float theta2_diff = cmd_theta2 - act_theta2;
      float theta3_diff = cmd_theta3 - act_theta3;

      //Check if commanded angles are numbers and at least one motor should move more than one step
      if (!isnan(cmd_theta1) && !isnan(cmd_theta2) && !isnan(cmd_theta3)){

          if (abs(theta1_diff) >= 1|| abs(theta2_diff) >= 1 || abs(theta3_diff) >= 1){
            a_stepper.move(theta1_diff);
            b_stepper.move(theta2_diff);
            c_stepper.move(theta3_diff);

            oldstate_ = state_;
            state_ = STATE_MOVING;
            nh.loginfo("STATE 'WAITING': Change to STATE 'MOVING'");
          }
      }

      else{
         nh.logerror("Commanded Joint angle not valid");
      }
    }
    else{
      nh.logwarn("Wrong State active");
    }
  }
  else{
    nh.logwarn("Motor Control not enabled");
  }
}
ros::Subscriber<delta_arduino::cmdAngle> subCmdAngle("delta/set_angle",&moveMotorTo);
void commandHandler(const std_msgs::String& cmdString){
  String command = cmdString.data;
  if (command.equals("RESET")){
    nh.loginfo("delta/command received: RESET");
    if(state_ != STATE_RESET){
      oldstate_ = state_;
      state_ = STATE_RESET;
    }
    else{
      nh.logwarn("Already resetting");
    }
  }
  else if(command.equals("CTRLSTOP")){
    nh.loginfo("delta/command received: CTRLSTOP");
    stopCtrl();
  }
  else if(command.equals("CTRLSTART")){
    nh.loginfo("delta/command received: CTRLSTART");
    if (!enable) {
      startCtrl();
     oldstate_ = state_;
     state_ = STATE_INIT;
    }
    else nh.logwarn("Motor Control already enabled");
  }
  else{
    nh.logerror("delta/command not valid!");
  }
}
ros::Subscriber<std_msgs::String> subCmdString("delta/command",&commandHandler);

void stateLoop()
{
    switch(state_)
    {
      case STATE_OFF:
      {
          break;
      }
      case STATE_INIT:
      {
          if(enable){
            oldstate_= state_;
            state_ = STATE_RESET;
            nh.loginfo("STATE 'INIT': Change to STATE 'RESET'");

          }
          break;
      }
      case STATE_RESET:
      {
          if (oldstate_ != state_){
            nh.loginfo("STATE 'RESET': Drive Motors to endstops");
            a_stepper.move(130*stepsCircle/360);
            b_stepper.move(130*stepsCircle/360);
            c_stepper.move(130*stepsCircle/360);
            //a_stepper.setSpeed(stepsCircle/4);
            //b_stepper.setSpeed(stepsCircle/4);
            //c_stepper.setSpeed(stepsCircle/4);
            oldstate_ = state_;
          }
          if (a_stepper.distanceToGo() == 0 && b_stepper.distanceToGo() == 0 && c_stepper.distanceToGo() == 0){

            if(!set_reset){
                set_reset = true;
                nh.loginfo("STATE 'RESET': Move Motors back");
                a_stepper.setCurrentPosition(0);
                b_stepper.setCurrentPosition(0);
                c_stepper.setCurrentPosition(0);
                a_stepper.moveTo(-11*stepsCircle/360);
                b_stepper.moveTo(-11*stepsCircle/360);
                c_stepper.moveTo(-11*stepsCircle/360);
            }
            else{
                a_stepper.setCurrentPosition(0);
                b_stepper.setCurrentPosition(0);
                c_stepper.setCurrentPosition(0);
                set_reset = false;
                state_ = STATE_WAITING;
                nh.loginfo("STATE 'RESET': Change to STATE 'WAITING'");
              }
          }
          break;
        }
        case STATE_WAITING:
         {
            if (oldstate_ != state_){
              nh.loginfo("STATE 'WAITING': Waiting for new goal");
              oldstate_ = state_;
            }
              break;
        }
        case STATE_MOVING:
        {
            if (oldstate_ != state_){
              char output[32];
              char msg[128] = "STATE 'MOVING': Move motors to: ";
              printAngles(output);
              strcat(msg,output);
              nh.loginfo(msg);
              oldstate_ = state_;
            }
            if (a_stepper.distanceToGo() == 0 && b_stepper.distanceToGo() == 0 && c_stepper.distanceToGo() == 0){
              nh.loginfo("STATE 'MOVING': Goal reached");
              state_ = STATE_WAITING;
            }
            break;
        }
    }
}

void setup() {

  pinMode(LED_PIN, OUTPUT);
  //ROS functions
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  //nh.advertise(JointState);
  nh.subscribe(subCmdAngle);
  nh.subscribe(subCmdString);


  joint_state.name_length =  3;
  joint_state.position_length =  3;
  joint_state.velocity_length=   3;
  joint_state.name[0] = (char*)"joint1";
  joint_state.name[1] = (char*)"joint2";
  joint_state.name[2] = (char*)"joint3";

  while (!nh.connected() ){
      nh.spinOnce();
  }

  //Accelstepper inits
  initialize(&a_stepper, A_ENABLE_PIN, A_END_PIN);
  initialize(&b_stepper, B_ENABLE_PIN, B_END_PIN);
  initialize(&c_stepper, C_ENABLE_PIN, C_END_PIN);
  nh.loginfo("All Motors initialized");

  //Deactivate motors at startup
  stopCtrl();
  nh.loginfo("STATE 'OFF': Arduino ready");
  state_ = STATE_OFF;

}

void loop() {


  nh.spinOnce();
  stateLoop();
  checkEndstops();
  if(enable){
    if(checkWorkspace()){
      a_stepper.run();
      b_stepper.run();
      c_stepper.run();
      sendWSMsg = true;
    }
     else if(sendWSMsg){
      char output[32];
      char msg[128] = "Commanded position not valid: ";
      printAngles(output);
      strcat(msg,output);
      nh.logerror(msg);
      sendWSMsg = false;
      }
  }

  /*if(true){

    joint_state.position[0] = a_stepper.currentPosition() / stepsCircle*360;
    joint_state.position[1] = b_stepper.currentPosition() / stepsCircle*360;
    joint_state.position[2] = c_stepper.currentPosition() / stepsCircle*360;
    joint_state.velocity[0] = a_stepper.speed() / stepsCircle*360;
    joint_state.velocity[1] = b_stepper.speed() / stepsCircle*360;
    joint_state.velocity[2] = c_stepper.speed() / stepsCircle*360;
    joint_state.header.stamp = nh.now();

    joint_state.position[0] = 0;
    joint_state.position[1] = 0;
    joint_state.position[2] = 0;
    joint_state.velocity[0] = 0;
    joint_state.velocity[1] = 0;
    joint_state.velocity[2] = 0;
    joint_state.header.stamp = nh.now();


    loop_iteration=0;
    JointState.publish(&joint_state);
  }*/

}
