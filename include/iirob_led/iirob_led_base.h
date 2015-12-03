#ifndef IIROB_LED_BASE_H
#define IIROB_LED_BASE_H

#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>

#include <iirob_led/BlinkyAction.h>
#include <iirob_led/PoliceAction.h>
#include <iirob_led/FourRegionsAction.h>
#include <iirob_led/ChaserLightAction.h>
#include <iirob_led/SetLedDirectory.h>
#include <iirob_led/DirectionWithForce.h>
#include <iirob_led/TurnLedsOnOff.h>

#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>

#include "LEDStrip.h"
#include "RGBConverter.h"
#include "iirob_led_math.h"
/*
 * ============================================================
 * TASKS:
 * ============================================================
 * TODO Check if everywhere the duration parameter of a message is set properly (value > 0). Otherwise duration takes the default value from the message (=0), which equals infinity and node can only be stopped by escalating to SIGTERM
 * TODO override - when ready set to false
 * TODO Check return value for all iirob::hardware functions (setAllRGBf(), setRangeRGBf() etc.)
 * TODO Add a duration parameter (number of steps per cycle should be determined from it) to Changeling (just add two modes)
 * TODO All the checks if start_led or end_led are negative or not can be avoided by simply using unsigned chars (as far as I can tell internally the LED library is actually using this type)
 * TODO Remove all ROS_INFO (wherever it makes sense) and replace it with ROS_DEBUG
 * TODO Add message that supports range + color parameters for lighting up one or more LEDs
 * TODO Add launch parameter "steps_per_fade_cycle" in order to give the user the ability to override the default value: 100
 * TODO Measure the time between LEDStrip::send() and LEDStrip::receive() and combine it with the timing in order to obtain better results. These measurements can also be integrated into the code - upon execution do the measurements (using the rectangle and cross classes) and use the results to adapt the delays that occur)
 */

/**
 * @brief The IIROB_LED_Base class implements general functionality that is to be used by derived classes such as IIROB_LED_Rectangle and IIROB_LED_Cross
 */
class IIROB_LED_Base
{
protected:
    int mMSec;                 ///<
    int mNumLeds;              ///< Number of LEDs the strip has
    iirob_hardware::LEDStrip* mLed;    ///< Used for accessing the underlying hardware
    int mShowDir;              ///<
    bool status;                ///< Contains the return result from init()
    std::string port;           ///< Serial port
    int num;                    ///< Number of LEDs
    std::string localFrame;
    double maxForce;
    double _scalingFactor;
    int stepsPerFadeCycle;   ///< Determines the granularity of the fading (the greater the value, the more fluent the HSV fade-effect will be)
    // Subscribers
    ros::Subscriber subTurnLedsOnOff;   ///< Allows setting given range of (or a single) LEDs to a specific color with (0,0,0) being equal to "turn off"
    ros::Subscriber subForce;           ///< Gives visual feedback for the magnitude (for the rectangle only) and direction of an applied force (represented as a 3D vector)
    // Action servers
    actionlib::SimpleActionServer<iirob_led::BlinkyAction> blinkyAS;    ///< Handles Blinky goal messages
    actionlib::SimpleActionServer<iirob_led::PoliceAction> policeAS;
    actionlib::SimpleActionServer<iirob_led::FourRegionsAction> fourRegionsAS;  ///< Handles FourRegions goal messages
    actionlib::SimpleActionServer<iirob_led::ChaserLightAction> chaserLightAS;  ///< Handles ChaserLight goal messages
public:
    /**
     * @brief IIROB_LED_Base Constructor that initializes the hardware
     * @param nodeHandle
     * @param _port Port as string
     * @param _m_numLeds Number of LEDs
     */
    IIROB_LED_Base(ros::NodeHandle nodeHandle, std::string const& port, int const& mNumLeds, double maxForce, int maxForceLeds, std::string link);

    /**
     * @brief Destructor turns off all LEDs and shuts down all action servers and subscribers
     */
    ~IIROB_LED_Base();

    /**
     * @brief checkLimits checks if the given start and end of a range of LEDs is withing some hardcoded limits and changes those accordingly
     * @param start_led The beginning of a range of LEDs
     * @param end_led The end of a range of LEDs
     */
    void checkLimits(int *start_led, int *end_led);

    /**
     * @brief init Initializes the hardware and starts the action servers
     * @param port Port as string
     * @param num Number of LEDs
     * @return
     */
    bool init(std::string const& port, int const& mNumLeds);

    /**
     * @brief spin Simply calls ros::spin()
     */
    void spin();

    /**
     * @brief getStatus Retrieves the status of the initialization procedure called inside the constructor
     * @return True if init() was successful
     */
    bool getStatus();

    // Callbacks for all action servers and subscribers
    /**
     * @brief turnLedOnOffCallback Turns on or off a given range of (or a single) LEDs
     * @param led_onoff_msg
     */
    void turnLedOnOffCallback(const iirob_led::TurnLedsOnOff::ConstPtr& led_onoff_msg);

    /**
     * @brief blinkyCallback processes BlinkyActionGoal messages - it turns on and off a give stripe of LEDs
     * @param goal
     */
    void blinkyCallback(const iirob_led::BlinkyGoal::ConstPtr& goal);

    /**
     * @brief fourRegionsCallback processes FourRegionsGoal messages - it turns on and off a give stripe of LEDs which is split into 4 separate simultaniously blinking sections with different colors
     * @param goal
     */
    void fourRegionsCallback(const iirob_led::FourRegionsGoal::ConstPtr& goal);

    /**
     * @brief chaserLightCallback processes ChaserLightGoal messages - a given stripe of LEDs traverses the whole strip in circle N times
     * @param goal
     */
    void chaserLightCallback(const iirob_led::ChaserLightGoal::ConstPtr& goal);

    /**
     * @brief policeCallback processes PoliceActionGoal messages - it turns on and off a give stripe of LEDs mimicing a police light
     * @param goal
     */
    virtual void policeCallback(const iirob_led::PoliceGoal::ConstPtr& goal) = 0;

    /**
     * @brief forceCallback Retrieves a vector with a TF2 frame and displays it using the LED system (rectangle and cross)
     * @param led_force_msg
     */
    virtual void forceCallback(const iirob_led::DirectionWithForce::ConstPtr& led_force_msg) = 0;
    // TODO Add virtual policeCallback and start action server here
};

#endif // IIROB_LED_BASE_H
