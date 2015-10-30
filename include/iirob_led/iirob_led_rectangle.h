#ifndef IIROB_LED_STRIP_H
#define IIROB_LED_STRIP_H
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

/*
 * NOTES:
 * =========================================================================================
 * SR2 platform corners and quadrants:
 *   (3rd quadrant)         (2nd quadrant)
 * CORNER_BACK_LEFT ----- CORNER_FRONT_LEFT
 *         |                       |
 *         |           x           |_____________[key switch]
 *         |    (none quadrant)    |
 *         |                       |
 * CORNER_BACK_RIGHT ---- CORNER_FRONT_RIGHT
 *   (4th quadrant)         (1st quadrant)
 * QUADRANT_NONE      x = y = 0
 * QUADRANT_FIRST     1st quadrant: +x, +y
 * QUADRANT_SECOND    2nd quadrant: -x, +y
 * QUADRANT_THIRD     3rd quadrant: -x, -y
 * QUADRANT_FOURTH    4th quadrant: +x, -y
 *
 *
 *
 *
tf2_ros::Buffer *p_tfBuffer;
    tf2_ros::TransformListener* p_tfListener;
    tf2::Transform transform_ee_base;
    geometry_msgs::TransformStamped transform_ee_base_stamped;


p_tfBuffer = new tf2_ros::Buffer();
    p_tfListener = new tf2_ros::TransformListener(*p_tfBuffer, true);
 *
 *
void ForceTorqueNode::updateFTData(const ros::TimerEvent& event)
{
//     ros::Time start = ros::Time::now();

    int status = 0;
    double Fx, Fy, Fz, Tx, Ty, Tz = 0;

    p_Ftc->ReadSGData(status, Fx, Fy, Fz, Tx, Ty, Tz);

    geometry_msgs::WrenchStamped msg, msg_transformed;

    msg.header.frame_id = frame_id;
    msg.header.stamp = ros::Time::now();
    msg.wrench.force.x = Fx-F_avg[0];
    msg.wrench.force.y = Fy-F_avg[1];
    msg.wrench.force.z = Fz-F_avg[2];
    msg.wrench.torque.x = Tx-F_avg[3];
    msg.wrench.torque.y = Ty-F_avg[4];
    msg.wrench.torque.z = Tz-F_avg[5];
    topicPub_ForceData_.publish(msg);


    try{
    transform_ee_base_stamped = p_tfBuffer->lookupTransform(transform_frame_id, frame_id, ros::Time(0));
    }
    catch (tf2::TransformException ex ){
    ROS_ERROR("%s",ex.what());
    }

    geometry_msgs::Vector3Stamped temp_vector_in, temp_vector_out;

    temp_vector_in.header = msg.header; ------------------------------------------------------------------------IMPORTANT
    temp_vector_in.vector = msg.wrench.force;
    tf2::doTransform(temp_vector_in, temp_vector_out, transform_ee_base_stamped);
    msg_transformed.header.stamp = msg.header.stamp;
    msg_transformed.header.frame_id = temp_vector_out.header.frame_id;
    msg_transformed.wrench.force = temp_vector_out.vector;

    temp_vector_in.vector = msg.wrench.torque;
    tf2::doTransform(temp_vector_in, temp_vector_out, transform_ee_base_stamped);
    msg_transformed.wrench.torque = temp_vector_out.vector;-----------------------------------------------------IMPORTANT

    topicPub_ForceDataTrans_.publish(msg_transformed);

//     ROS_INFO("Duration time of calcuation: %f'", (ros::Time::now() - start).toSec());
//     ROS_INFO("Time between calls: %f", (event.current_real - event.last_real).toSec());
//     ROS_INFO("Error: %f", (event.current_expected - event.current_real).toSec());
}
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

#define QUADRANT_NONE   0   // x = y = 0
#define QUADRANT_FIRST  1   // 1st quadrant: +x, +y
#define QUADRANT_SECOND 2   // 2nd quadrant: -x, +y
#define QUADRANT_THIRD  3   // 3rd quadrant: -x, -y
#define QUADRANT_FOURTH 4   // 4th quadrant: +x, -y

// Ruben -> take his caluclations for the LEDs
#define CORNER_FRONT_LEFT   0
#define CORNER_BACK_LEFT    108
#define CORNER_BACK_RIGHT   192
#define CORNER_FRONT_RIGHT  300

const int long_side = 108;
const int short_side = 84;

const int led_start = 0;
const int led_end = 2*long_side + 2*short_side;

const int led_corner_front_right = led_end;
const int led_corner_front_left = short_side;
const int led_corner_back_right = short_side + long_side;
const int led_corner_back_left = short_side + 2*long_side;

// TODO override - when ready set to false
// TODO forceCallback() needs to light up the corners first and then each side of the chosen corner (work on this AFTER you find a proper way of lighting up a single LED)
// TODO Check return value for all iirob::hardware functions (setAllRGBf(), setRangeRGBf() etc.)
// TODO Add a duration parameter (number of steps per cycle should be determined from it) to Changeling (just add two modes)
// TODO All the checks if start_led or end_led are negative or not can be avoided by simply using unsigned chars (as far as I can tell internally the LED library is actually using this type)
// TODO Remove all ROS_INFO (wherever it makes sense) and replace it with ROS_DEBUG
// TODO Add message that supports range + color parameters for lighting up one or more LEDs
// NOTE Except for color values (ColorRGBA requires float) use double (float64 in ROS) for more accuracy

/**
 * @brief The IIROB_LED_Rectangle class controls the LED strip mounted around the edges of the bottom platform of the SR2
 */

class IIROB_LED_Rectangle
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
     * @brief IIROB_LED_Rectangle Constructor that initializes the hardware
     * @param nodeHandle
     * @param _port Port as string
     * @param _m_numLeds Number of LEDs
     */
    IIROB_LED_Rectangle(ros::NodeHandle nodeHandle, std::string const& _port, int const& _m_numLeds);

    /**
     * @brief Destructor turns off all LEDs and shuts down all action servers and subscribers
     */
    ~IIROB_LED_Rectangle();

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

#endif // IIROB_LED_STRIP_H
