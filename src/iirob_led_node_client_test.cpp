#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <iirob_led/PlayLedGoal.h>
#include <iirob_led/BlinkyGoal.h>
#include <iirob_led/BlinkyAction.h>
#include <iirob_led/PoliceGoal.h>
#include <iirob_led/FourMusketeersGoal.h>
#include <iirob_led/RunningBunnyGoal.h>
#include <iirob_led/ChangelingGoal.h>
#include <iirob_led/SpinnerGoal.h>
#include <iirob_led/SetLedDirectory.h>

#include "LEDStrip.h"

#define DURATION_ON 3
#define DURATION_OFF 2
#define BLINKS 5

typedef actionlib::SimpleActionClient<iirob_led::BlinkyAction> Client;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iirob_led_node_test_client");
    std::string actionServerSubscription;
    ros::NodeHandle nh;
    nh.param<std::string>("leds_client/as", actionServerSubscription, "leds/blinky");

    ROS_INFO_STREAM("Subscribed to " << actionServerSubscription << " action server");
    Client client(actionServerSubscription, true); // true -> don't need ros::spin() (see note below SimpleActionClient in C++ tutorial)
    if(!client.waitForServer(ros::Duration(10, 0))) // If client fails to connect to action server within 10 seconds
    {
        ROS_ERROR("Failed to connect to action server!");
        return 1;
    }

    ROS_INFO("Sending Blinky goal");
    iirob_led::BlinkyGoal goal;
    goal.color.r = 1;
    goal.color.g = 0;
    goal.color.b = 0;
    goal.color.a = 1;
    goal.blinks = BLINKS;
    goal.duration_on = DURATION_ON;
    goal.duration_off = DURATION_OFF;
    goal.start_led = 200;
    goal.end_led = 250;
    // Fill in goal here
    client.sendGoal(goal);
    ROS_INFO("Goal sent");
    client.waitForResult(ros::Duration(DURATION_ON * DURATION_OFF * BLINKS + 5, 0));
    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        printf("Wohooo! LEDs blinked!");
    ROS_INFO("Current State: %s\n", client.getState().toString().c_str());

    return 0;
}
