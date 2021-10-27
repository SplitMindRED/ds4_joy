#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/Joy.h>

/* * * BUTTONS * * *
0 - X
1 - O
2 - triangle
3 - square
4 - L1
5 - R1
arrows in "axes" array
left - axes[6] = 1
right - axes[6] = -1
down - axes[7] = -1
up - axes[7] = 1
 * * * * * * * * * */

/* * * AXES (JOYSTICKS) * * *
 *                   neutral /  min / max
0 - Left Horizontal        0 /  1.0 / -1.0
1 - Left Vertical          0 / -1.0 /  1.0
2 - L2                   1.0 / -1.0 /  1.0
3 - Right Horizontal     0.0 /  1.0 / -1.0
4 - Right Vertical       0.0 / -1.0 /  1.0
5 - R2                   1.0 / -1.0 /  1.0
 * * * * * * * * * */


#define MAX_LIN_VEL       0.2
#define MAX_ANG_VEL       0.8
#define MAX_NUMBER_OF_ROBOTS  2

enum robot_counter
{
  KURSANT_1, //0
  KURSANT_3  //1
};

static robot_counter robot_number = KURSANT_1;

static sensor_msgs::Joy joy;
static bool is_joy_updated[MAX_NUMBER_OF_ROBOTS];
static bool is_emergency_stop[MAX_NUMBER_OF_ROBOTS];
static bool is_safety_trigger[MAX_NUMBER_OF_ROBOTS];

static ros::Publisher cmd_vel[MAX_NUMBER_OF_ROBOTS];
static ros::Subscriber joy_sub;

static ros::ServiceClient hard_stop[MAX_NUMBER_OF_ROBOTS];
static ros::ServiceClient disable_hard_stop[MAX_NUMBER_OF_ROBOTS];

static int number_of_robots = 1;

//set 0 value to all arrays
void init()
{
  for(uint8_t i = 0; i < MAX_NUMBER_OF_ROBOTS; i++)
  {
    is_joy_updated[i] = false;
    is_emergency_stop[i] = false;
    is_safety_trigger[i] = false;
  }
}

//adjustable delay timer
void delayTimer(uint16_t dt)
{
  ros::Rate timer_rate(1000);
  uint32_t timer = 0;

  while(timer < dt)
  {
    timer_rate.sleep();
    timer++;
  }
}

void joyCallback(sensor_msgs::Joy msg)
{
  joy = msg;

  if(number_of_robots > 1)
  {
    if(msg.axes[6] > 0.8 && robot_number != KURSANT_1)
    {
      robot_number = KURSANT_1;
      ROS_WARN("Switch to Kursant %d", KURSANT_1+1);
    }
    else if(msg.axes[6] < -0.8 && robot_number != KURSANT_3)
    {
      robot_number = KURSANT_3;
      ROS_WARN("Switch to Kursant %d", KURSANT_3+1);
    }
  }

  //press triangle - hard stop
  if(joy.buttons[2] == 1 && joy.buttons[4] == 0 && joy.buttons[5] == 0 && is_emergency_stop[robot_number] == false)
  {
    std_srvs::Trigger srv;

    if(hard_stop[robot_number].call(srv))
    {
      ROS_WARN("Kursant %d: HARD STOP!", robot_number+1);
    }
    else
    {
      ROS_ERROR("Kursant %d: Can't call hard stop service!", robot_number+1);
    }

    is_emergency_stop[robot_number] = true;
  }
  //press L1+R1+triangle - disable hard stop
  else if(joy.buttons[2] == 1 && joy.buttons[4] == 1 && joy.buttons[5] == 1 && is_emergency_stop[robot_number] == true)
  {
    is_emergency_stop[robot_number] = false;

    std_srvs::Trigger srv;

    if(disable_hard_stop[robot_number].call(srv))
    {
      ROS_WARN("Kursant %d: HARD STOP DISABLED!", robot_number+1);
    }
    else
    {
      ROS_ERROR("Kursant %d: Can't call disable hard stop service!", robot_number+1);
    }
  }
  else if(joy.axes[5] < -0.8)
  {
    is_safety_trigger[robot_number] = true;
    is_joy_updated[robot_number] = true;
  }

  if(joy.axes[5] > -0.8 && is_safety_trigger[robot_number] == true)
  {
    is_safety_trigger[robot_number] = false;

    ROS_INFO("Kursant %d: Safety stop", robot_number+1);

    geometry_msgs::Twist msg;

    msg.linear.x = 0;

    msg.angular.z = 0;

    delayTimer(150);

    cmd_vel[robot_number].publish(msg);

    delayTimer(150);
  }
}

//convert speed form meters to mm, round and convert back to meters
float speedStepControl(float vel_m)
{
  int16_t vel_cm = 0;

  vel_cm = vel_m * 100;

  return float(vel_cm)/100;
}

void move()
{
  if(is_joy_updated[robot_number] == true && is_safety_trigger[robot_number] == true)
  {
    geometry_msgs::Twist msg;

//    msg.linear.x = joy.axes[4] * MAX_LIN_VEL;
//    msg.angular.z = joy.axes[3] * MAX_ANG_VEL;

    msg.linear.x = speedStepControl(joy.axes[4] * MAX_LIN_VEL);
    msg.angular.z = speedStepControl(joy.axes[3] * MAX_ANG_VEL);

    cmd_vel[robot_number].publish(msg);

    is_joy_updated[robot_number] = false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ds4_control");
  ros::NodeHandle n;
  ros::Rate rate(10);

  ROS_INFO("Initialization...");

//  int MAX_NUMBER_OF_ROBOTS = MAX_NUMBER_OF_ROBOTS;

  n.getParam("/ds4/number_of_robots", number_of_robots);

  std::cout << "number_of_robots: " << number_of_robots << std::endl;

  joy_sub = n.subscribe("/joy", 1, joyCallback);

  if(number_of_robots == 1)
  {
    cmd_vel[0] = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    hard_stop[0] = n.serviceClient<std_srvs::Trigger>("/hard_stop");
    disable_hard_stop[0] = n.serviceClient<std_srvs::Trigger>("/disable_hard_stop");
  }

  if(number_of_robots == 2)
  {
    cmd_vel[0] = n.advertise<geometry_msgs::Twist>("/r1/cmd_vel", 1);
    hard_stop[0] = n.serviceClient<std_srvs::Trigger>("/r1/hard_stop");
    disable_hard_stop[0] = n.serviceClient<std_srvs::Trigger>("/r1/disable_hard_stop");

    cmd_vel[1] = n.advertise<geometry_msgs::Twist>("/r2/cmd_vel", 1);
    hard_stop[1] = n.serviceClient<std_srvs::Trigger>("/r2/hard_stop");
    disable_hard_stop[1] = n.serviceClient<std_srvs::Trigger>("/r2/disable_hard_stop");
  }

  ROS_INFO("Enter loop");
  while (ros::ok())
  {
    if(is_emergency_stop[robot_number] == false)
    {
      move();
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
