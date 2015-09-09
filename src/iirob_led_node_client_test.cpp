#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <iirob_led/BlinkyGoal.h>
#include <iirob_led/BlinkyAction.h>
#include <iirob_led/PoliceGoal.h>
#include <iirob_led/FourMusketeersGoal.h>
#include <iirob_led/RunningBunnyGoal.h>
//#include <iirob_led/ChangelingGoal.h>
//#include <iirob_led/SpinnerGoal.h>
#include <iirob_led/SetLedDirectory.h>

#include <LEDStrip.h>
#include <RGBConverter.h>
#include <cstdlib>
#include <vector>

#define DURATION_ON 1
#define DURATION_OFF .5
#define BLINKS 1

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

    float r, g, b;
    float valueReductionStep;
    if(argc == 5) {
        ROS_INFO("Using cmd args");
        RGBConverter::rgbIntToFloat(atoi(argv[1]), atoi(argv[2]), atoi(argv[3]), &r, &g, &b);
        valueReductionStep = atof(argv[4])/100;

        if(valueReductionStep < 0.1) valueReductionStep = 0.1;
    } else {
        ROS_INFO("Using built-in args");
        r = 1.;
        g = 0.;
        b = 0.;
        valueReductionStep = 0;
    }

    ROS_INFO("Sending Blinky goal");
    iirob_led::BlinkyGoal goal;

    if(valueReductionStep) {
        float h, s, v;
        RGBConverter::rgbToHsv(r, g, b, &h, &s, &v);
        ROS_INFO("Value (HSV) reduction activated | step: %.2f", valueReductionStep);

        goal.color.r = r;
        goal.color.g = g;
        goal.color.b = b;
        goal.color.a = 0;
        goal.blinks = BLINKS;
        goal.duration_on = DURATION_ON;
        goal.duration_off = DURATION_OFF;
        goal.start_led = 190;
        goal.end_led = 300;

        client.sendGoal(goal);
        ROS_INFO("Goal sent");
        client.waitForResult();
        ROS_INFO("Received result");
        if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO("Wohooo! LEDs blinked!");
        ROS_INFO("Current State: %s\n", client.getState().toString().c_str());


        RGBConverter::rgbToHsv(r, g, b, &h, &s, &v);
        ROS_INFO("Converting RGB(%.2f %.2f %.2f) to HSV(%.2f %.2f %.2f)", r, g, b, h, s, v);
        v -= valueReductionStep;
        RGBConverter::hsvToRgb(h, s, v, &r, &g, &b);
        ROS_INFO("Converting HSL(%.2f %.2f %.2f) to RGB(%.2f %.2f %.2f)", h, s, v, r, g, b);

        goal.color.r = r;
        goal.color.g = g;
        goal.color.b = b;
        goal.color.a = 0;
        goal.blinks = BLINKS;
        goal.duration_on = DURATION_ON;
        goal.duration_off = DURATION_OFF;
        goal.start_led = 190;
        goal.end_led = 300;

        client.sendGoal(goal);
        ROS_INFO("Goal sent");
        client.waitForResult();
        ROS_INFO("Received result");
        if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO("Wohooo! LEDs blinked!");
        ROS_INFO("Current State: %s\n", client.getState().toString().c_str());

        RGBConverter::rgbToHsv(r, g, b, &h, &s, &v);
        ROS_INFO("Converting RGB(%.2f %.2f %.2f) to HSV(%.2f %.2f %.2f)", r, g, b, h, s, v);
        v -= valueReductionStep;
        RGBConverter::hsvToRgb(h, s, v, &r, &g, &b);
        ROS_INFO("Converting HSL(%.2f %.2f %.2f) to RGB(%.2f %.2f %.2f)", h, s, v, r, g, b);

        goal.color.r = r;
        goal.color.g = g;
        goal.color.b = b;
        goal.color.a = 0;
        goal.blinks = BLINKS;
        goal.duration_on = DURATION_ON;
        goal.duration_off = DURATION_OFF;
        goal.start_led = 190;
        goal.end_led = 300;

        client.sendGoal(goal);
        ROS_INFO("Goal sent");
        client.waitForResult();
        ROS_INFO("Received result");
        if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO("Wohooo! LEDs blinked!");
        ROS_INFO("Current State: %s\n", client.getState().toString().c_str());

        RGBConverter::rgbToHsv(r, g, b, &h, &s, &v);
        ROS_INFO("Converting RGB(%.2f %.2f %.2f) to HSV(%.2f %.2f %.2f)", r, g, b, h, s, v);
        v -= valueReductionStep;
        RGBConverter::hsvToRgb(h, s, v, &r, &g, &b);
        ROS_INFO("Converting HSL(%.2f %.2f %.2f) to RGB(%.2f %.2f %.2f)", h, s, v, r, g, b);

        goal.color.r = r;
        goal.color.g = g;
        goal.color.b = b;
        goal.color.a = 0;
        goal.blinks = BLINKS;
        goal.duration_on = DURATION_ON;
        goal.duration_off = DURATION_OFF;
        goal.start_led = 190;
        goal.end_led = 300;

        client.sendGoal(goal);
        ROS_INFO("Goal sent");
        client.waitForResult();
        ROS_INFO("Received result");
        if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO("Wohooo! LEDs blinked!");
        ROS_INFO("Current State: %s\n", client.getState().toString().c_str());
    } else {
        goal.color.r = r;
        goal.color.g = g;
        goal.color.b = b;
        goal.color.a = 0;
        goal.blinks = BLINKS;
        goal.duration_on = DURATION_ON;
        goal.duration_off = DURATION_OFF;
        goal.start_led = 190;
        goal.end_led = 300;
        client.sendGoal(goal);
        ROS_INFO("Goal sent");
        client.waitForResult(ros::Duration(DURATION_ON + DURATION_OFF + 1, 0));
        ROS_INFO("Received result");
        if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO("Wohooo! LEDs blinked!");
        ROS_INFO("Current State: %s\n", client.getState().toString().c_str());
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
