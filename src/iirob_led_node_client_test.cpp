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

#include <LEDStrip.h>
#include <RGBConverter.h>

#define DURATION_ON 1
#define DURATION_OFF .5
#define BLINKS 2

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
    goal.color.r = 255;
    goal.color.g = 0;
    goal.color.b = 0;
    goal.color.a = 0;

    float r = goal.color.r;
    float g = goal.color.g;
    float b = goal.color.b;
    float h, s, l;
    RGBConverter::rgbToHsl(r, g, b, &h, &s, &l);
    ROS_INFO("RGB[%.2f %.2f %.2f] -> HSL[%.2f %.2f %.2f]", r, g, b, h, s, l);

    float saturationLvls[10];
    float _s = 1.0;
    for(int i = 0; i < 10; i++, _s -= .1) {
        saturationLvls[i] = _s;
        ROS_INFO("Added saturation level of %f%%", saturationLvls[i]);
    }

    for(int i = 0; i < 10; i++) {
        RGBConverter::hslToRgb(h, saturationLvls[i], l, &r, &g, &b);
        ROS_INFO("HSL[%.2f %.2f %.2f] -> RGB[%.2f %.2f %.2f]", h, saturationLvls[i], l, r, g, b);
        goal.color.r = r;
        goal.color.g = g;
        goal.color.b = b;

        goal.blinks = BLINKS;
        goal.duration_on = DURATION_ON;
        goal.duration_off = DURATION_OFF;
        goal.start_led = 200;
        goal.end_led = 250;
        // Fill in goal here
        client.sendGoal(goal);
        ROS_INFO("Goal sent");
        client.waitForResult(ros::Duration(5, 0));
        if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            printf("Wohooo! LEDs blinked!");
        ROS_INFO("Current State: %s\n", client.getState().toString().c_str());
        ROS_INFO("Reducing saturation by 10%%");
    }

    /*hsl2rgb(h, s, l, &r, &g, &b);
    ROS_INFO("Setting lightness to 30");
    ROS_INFO("HSL[%.2f %.2f %.2f] -> RGB[%.2f %.2f %.2f]", h, s, l, r, g, b);

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
    ROS_INFO("Current State: %s\n", client.getState().toString().c_str());*/

    return 0;
}
