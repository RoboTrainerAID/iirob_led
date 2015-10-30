#ifndef IIROB_LED_Cross_H
#define IIROB_LED_Cross_H
#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>

#include <iirob_led/BlinkyAction.h>
#include <iirob_led/PoliceAction.h>
#include <iirob_led/FourRegionsAction.h>
#include <iirob_led/ChaserLightAction.h>
#include <iirob_led/SetLedDirectory.h>
#include <iirob_led/DirectionWithForce.h>

#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Vector3.h>

#include "LEDStrip.h"
#include "iirob_led_rectangle.h" ///< For the class inheritance

#define CORNER_CROSS_FRONT_LEFT   0
#define CORNER_CROSS_BACK_LEFT    108
#define CORNER_CROSS_BACK_RIGHT   192
#define CORNER_CROSS_FRONT_RIGHT  300

// TODO Sturcture the constants and #defines in a better way
/*const int long_side = 108;
const int short_side = 84;

const int led_start = 0;
const int led_end = 2*long_side + 2*short_side;

const int led_corner_front_right = led_end;
const int led_corner_front_left = short_side;
const int led_corner_back_right = short_side + long_side;
const int led_corner_back_left = short_side + 2*long_side;*/

/**
 * @brief The IIROB_LED_Cross class controls the LED strip mounted around the edges of the bottom platform of the SR2
 */

class IIROB_LED_Cross
{
private:
    const int MAX_FORCE;        ///< defines the maximum number of LEDs that can be lit (currently set to 10). If changed don't forget to redefine the corners to fit the new bounds
    int m_msec;                 ///<
    int m_numLeds;              ///< Number of LEDs the strip has
    iirob_hardware::LEDStrip* m_led;    ///< Used for accessing the underlying hardware
    int m_showDir;              ///<
    bool status;                ///< Contains the return result from init()
    std::string port;           ///< Serial port
    int num;                    ///< Number of LEDs

    ros::Subscriber subForce;   ///< Gives visual feedback for the magnitude and direction of an applied force (represented as a 3D vector)
    // Action servers
    actionlib::SimpleActionServer<iirob_led::BlinkyAction> blinkyAS;    ///< Handles Blinky goal messages
    actionlib::SimpleActionServer<iirob_led::PoliceAction> policeAS;    ///< Handles Police goal messages
    actionlib::SimpleActionServer<iirob_led::FourRegionsAction> fourRegionsAS;  ///< Handles FourRegions goal messages
    actionlib::SimpleActionServer<iirob_led::ChaserLightAction> chaserLightAS;  ///< Handles ChaserLight goal messages

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
    bool init(std::string const& port, int const& m_numLeds);
public:
    /**
     * @brief IIROB_LED_Cross Constructor that initializes the hardware
     * @param nodeHandle
     * @param _port Port as string
     * @param _m_numLeds Number of LEDs
     */
    IIROB_LED_Cross(ros::NodeHandle nodeHandle, std::string const& _port, int const& _m_numLeds);

    /**
     * @brief Destructor turns off all LEDs and shuts down all action servers and subscribers
     */
    ~IIROB_LED_Cross();

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
     * @brief forceCallback
     * @param led_force_msg
     */
    void forceCallback(const iirob_led::DirectionWithForce::ConstPtr& led_force_msg);

    /**
     * @brief blinkyCallback processes BlinkyActionGoal messages - it turns on and off a give stripe of LEDs; previous name: lightActionCallback_2 with argument std_msgs::Float32MultiArray::ConstPtr&
     * @param goal
     */
    void blinkyCallback(const iirob_led::BlinkyGoal::ConstPtr& goal);

    /**
     * @brief policeCallback processes PoliceActionGoal messages - it turns on and off a give stripe of LEDs mimicing a police light; previous name: lightActionCallback_3 with argument std_msgs::Float32MultiArray::ConstPtr&
     * @param goal
     */
    void policeCallback(const iirob_led::PoliceGoal::ConstPtr& goal);

    /**
     * @brief fourMusketeersCallback processes FourMusketeersGoal messages - it turns on and off a give stripe of LEDs which is split into 4 separate simultaniously blinking sections with different colors; previous name: lightActionCallback_4 with argument std_msgs::Float32MultiArray::ConstPtr&
     * @param goal
     */
    void fourRegionsCallback(const iirob_led::FourRegionsGoal::ConstPtr& goal);

    /**
     * @brief chaserLightCallback processes ChaserLightGoal messages - a given stripe of LEDs traverses the whole strip in circle N times; previous name (maybe? never managed to make it work...): lightActionCallback_6 with argument std_msgs::Float32MultiArray::ConstPtr&
     * @param goal
     */
    void chaserLightCallback(const iirob_led::ChaserLightGoal::ConstPtr& goal);
};

#endif // IIROB_LED_Cross_H
