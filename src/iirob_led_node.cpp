#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>

#include <iirob_led/BlinkyAction.h>
#include <iirob_led/PoliceAction.h>
#include <iirob_led/FourRegionsAction.h>
#include <iirob_led/ChaserLightAction.h>
#include <iirob_led/SetLedDirectory.h>
#include <iirob_led/DirectionWithForce.h>
#include <RGBConverter.h>

#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Vector3.h>

#include <string>
//#include <cmath>

#include "iirob_led_rectangle.h"
#include "iirob_led_cross.h"

int main(int argc, char **argv)
{
    std::string port;
    int led_num;
	bool override;
    std::string type;         // Type specifies which (rectangle or cross LED strip) will be enabled
    std::string link;

	ros::init(argc, argv, "iirob_led_node");
    ros::NodeHandle nh("~");

    // Display warning when port not specified
    // Rectangular platform (bottom)
    nh.getParam("port", port);
    if(port.empty()) {
        ROS_WARN("Given port is empty. Falling back to default: /dev/ttyUSB0");
        nh.param<std::string>("port", port, "/dev/ttyUSB0");
    }
    // Cross
    //nh.param<std::string>("port", port, "/dev/ttyUSB1");
    nh.param<int>("led_num", led_num, 1);
    nh.param<bool>("override", override, false);
    nh.param<std::string>("link", link);

    ROS_INFO("IIROB-LED node %s: Setting override to %s", ros::this_node::getName().c_str(), (override ? (char *)"on" : (char *)"off"));

    // If more then 8 LEDs are connected to the mC it will burn out
    if (!override && (led_num>8)) led_num = 8;
    ROS_INFO("IIROB-LED node %s: Setting number of LEDs to %d", ros::this_node::getName().c_str(), led_num);

    nh.getParam("type", type);
    if(!type.compare("cross")) {
        ROS_INFO("IIROB-LED node %s: Initializing ledNode for cross strip on port %s with %d LEDs", ros::this_node::getName().c_str(), port.c_str(), led_num);
        IIROB_LED_Cross *crossStrip = new IIROB_LED_Cross(nh, port, led_num);
        if(crossStrip->getStatus()) crossStrip->spin();
        delete crossStrip;
    }
    else {
        if(!type.compare("rectangle")) ROS_INFO("IIROB-LED node %s: Initializing ledNode for rectangular strip on port %s with %d LEDs", ros::this_node::getName().c_str(), port.c_str(), led_num);
        else ROS_INFO("IIROB-LED node %s: Unknown type. Falling back to ledNode for rectangular strip on port %s with %d LEDs", ros::this_node::getName().c_str(), port.c_str(), led_num);

        IIROB_LED_Rectangle *rectangleStrip = new IIROB_LED_Rectangle(nh, port, led_num, link);
        if(rectangleStrip->getStatus()) rectangleStrip->spin();
        delete rectangleStrip;
    }

    return 0;
}
