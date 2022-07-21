#include <ros.h>
#include <std_msgs/String.h>

// motor control pins
#define LR_EN 8
#define LL_EN 10
#define RR_EN 7
#define RL_EN 4

#define LR_PWM 9 
#define LL_PWM 6
#define RR_PWM 5
#define RL_PWM 3

// direction variables
enum DIR
{
  FORWARD = 0, BACKWARD, LEFT, RIGHT, HALT
};

void move_robot(int dir);
void manual_control();
void msg_cb(const std_msgs::String &msg);

int ang_speed = 50;
int speed_delta = 5;
volatile char command = 'h';  // start stopped

// create a rosserial node
ros::NodeHandle nh;

// create a subscriber
ros::Subscriber<std_msgs::String> pi_sub("/move_cmds", &msg_cb);

void setup()
{
  // initialize the node
  nh.initNode();
  nh.subscribe(pi_sub);
  
  pinMode(RR_EN, OUTPUT);
  pinMode(RL_EN, OUTPUT);
  pinMode(LR_EN, OUTPUT);
  pinMode(LL_EN, OUTPUT);
  pinMode(RR_PWM, OUTPUT);
  pinMode(RL_PWM, OUTPUT);
  pinMode(LR_PWM, OUTPUT);
  pinMode(LL_PWM, OUTPUT);

  digitalWrite(RR_EN, HIGH);
  digitalWrite(RL_EN, HIGH);
  digitalWrite(LR_EN, HIGH);
  digitalWrite(LL_EN, HIGH);
}

void loop()
{
  manual_control();
  nh.spinOnce();
  delay(100);
}

void manual_control()
{
  switch(command)
  {
    case 'f':
      move_robot(FORWARD);
      break;
    case 'b':
      move_robot(BACKWARD);
      break;
    case 'l':
      move_robot(LEFT);
      break;
    case 'r':
      move_robot(RIGHT);
      break;
    case 'h':
      move_robot(HALT);
      break;
    case 'I':
      if(ang_speed + speed_delta < 255){ang_speed += speed_delta;}
      break;
    case 'D':
      if(ang_speed - speed_delta > 0){ang_speed -= speed_delta;}
      break;
    default:
      break;
  }
}

void move_robot(int dir)
{
  switch(dir)
  {
    case 0:
      analogWrite(RR_PWM, ang_speed);
      analogWrite(RL_PWM, 0);
      analogWrite(LR_PWM, ang_speed);
      analogWrite(LL_PWM, 0);
      break;
    case 1: 
      analogWrite(RR_PWM, 0);
      analogWrite(RL_PWM, ang_speed);
      analogWrite(LR_PWM, 0);
      analogWrite(LL_PWM, ang_speed);
      break;
    case 2:
      analogWrite(RR_PWM, ang_speed);
      analogWrite(RL_PWM, 0);
      analogWrite(LR_PWM, 0);
      analogWrite(LL_PWM, ang_speed);
      break;
    case 3: 
      analogWrite(RR_PWM, 0);
      analogWrite(RL_PWM, ang_speed);
      analogWrite(LR_PWM, ang_speed);
      analogWrite(LL_PWM, 0);
      break;
    case 4:
      analogWrite(RR_PWM, 0);
      analogWrite(RL_PWM, 0);
      analogWrite(LR_PWM, 0);
      analogWrite(LL_PWM, 0);
      break;
    default:
      break;
  }
}

void msg_cb(const std_msgs::String &msg)
{
  command = msg.data[0];
}
