#ifndef IIROB_LED_Cross_H
#define IIROB_LED_Cross_H
#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>

#include <iirob_led/PoliceAction.h>
#include <iirob_led/DirectionWithForce.h>

#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Vector3.h>

#include "iirob_led_rectangle.h" ///< For the class inheritance

/* SR2 cross devided into quadrants:
 *
 *   left side   right side
 *          forward(top)
 *              __
 *             |14|
 *             |  |
 *      _______|  |_______
 * left|7____13|22|0_____6|right
 *             |  |
 *             |  |
 *             |31|
 *
 *        backward(back)
 */


// TODO Ask what top, bottom, left and right should represent vector-wise
#define NONE            0   // x = y = 0
#define FORWARD_TOP     1   // 1st quadrant: +x, +y
#define BACKWARD_BOTTOM 2   // 2nd quadrant: -x, +y
#define LEFT            3   // 3rd quadrant: -x, -y
#define RIGHT           4   // 4th quadrant: +x, -y

/**
 * @brief The IIROB_LED_Cross class controls the LED strip mounted around the edges of the bottom platform of the SR2
 */
class IIROB_LED_Cross : public IIROB_LED_Base
{
private:
    // TODO Move as defines
    const int horizontal_side;      ///< All horizontal LEDs from corner to center without the center itself
    const int horizontal_no_center; ///< All horizontal LEDs without the center of the cross
    const int horizontal_all;       ///< All horizontal LEDs without the center of the cross
    const int vertical_side;        ///< All vertical LEDs from corner to center without the center itself
    const int vertical_no_center;   ///< All vertical LEDs without the center of the cross
    const int vertical_all;         ///< All vertical LEDs including the center of the cross

    // Note: The indexing starts from 0 and ends at 383 for the rectangle. However we need not substract -1 for all corners due to the alignment of the strips
    const int led_right;                    // 6
    const int led_left;                     // 7
    const int led_forward_top;              // 14
    const int led_center;                   // 22
    const int led_backward_bottom;          // 31

    // Note: The indexing starts from 0 and ends at 31 for the cross.
    ros::Subscriber subForce;           ///< Gives visual feedback for the magnitude and direction of an applied force (represented as a 3D vector)
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

    // Callbacks for all action servers and subscribers
    /**
     * @brief forceCallback
     * @param led_force_msg
     */
    void forceCallback(const iirob_led::DirectionWithForce::ConstPtr& led_force_msg);

    /**
     * @brief policeCallback processes PoliceActionGoal messages - it turns on and off a give stripe of LEDs mimicing a police light
     * @param goal
     */
    void policeCallback(const iirob_led::PoliceGoal::ConstPtr& goal);
};

#endif // IIROB_LED_Cross_H
