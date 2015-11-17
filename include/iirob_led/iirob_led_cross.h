#ifndef IIROB_LED_Cross_H
#define IIROB_LED_Cross_H
#include <iirob_led/PoliceAction.h>
#include <iirob_led/DirectionWithForce.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "iirob_led_rectangle.h" ///< For the class inheritance

/* SR2 cross devided into quadrants:
 *
 *   left side   right side
 *              +y
 *             |14|
 *             |  |
 *      _______|  |_______
 * -x  |7____13|22|0_____6| +x
 *             |  |
 *             |  |
 *             |31|
 *              -y
 *              |
 *              |
 *          <KINECT>
 */


// TODO Ask what top, bottom, left and right should represent vector-wise
#define NONE            0   // x = y = 0
#define FORWARD_TOP     1   // 1st quadrant: +x, +y
#define BACKWARD_BOTTOM 2   // 2nd quadrant: -x, +y
#define LEFT            3   // 3rd quadrant: -x, -y
#define RIGHT           4   // 4th quadrant: +x, -y

#define H_SIDE                  7
#define V_SIDE                  8

#define H_ALL_NO_CENTER         (H_SIDE * 2)
#define H_ALL                   ((H_SIDE * 2) + 1)
#define V_ALL_NO_CENTER         (V_SIDE * 2)
#define V_ALL                   ((V_SIDE * 2) + 1)

#define CROSS_START             0
#define CROSS_CENTER            (H_ALL_NO_CENTER + V_SIDE)
#define CROSS_END               (H_ALL_NO_CENTER + V_ALL)

#define H_LEFT_XPLUS_START      CROSS_START
#define H_LEFT_XPLUS_END        (H_LEFT_XPLUS_START + (H_SIDE - 1))
#define H_RIGHT_XMINUS_START    H_SIDE
#define H_RIGHT_XMINUS_END      (H_ALL_NO_CENTER - 1)

#define V_UPPER_YPLUS_START     H_ALL_NO_CENTER
#define V_UPPER_YPLUS_END       (CROSS_CENTER - 1)
#define V_BOTTOM_YMINUS_START   (CROSS_CENTER + 1)
#define V_BOTTOM_YMINUS_END     (CROSS_END - 1)

#define H_START                 CROSS_START
#define H_END                   (H_ALL_NO_CENTER - 1)
#define V_START                 V_UPPER_YPLUS_START
#define V_END                   (CROSS_END - 1)


/*horizontal_side(7), vertical_side(8),
  horizontal_no_center(horizontal_side*2), horizontal_all(horizontal_no_center+1),
  vertical_no_center(vertical_side*2), vertical_all(vertical_no_center+1),
  led_right(horizontal_side-1), led_left(horizontal_side),
  led_forward_top(horizontal_side*2),
  led_center(led_forward_top + vertical_side),
  led_backward_bottom(led_center + vertical_side)*/

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
    actionlib::SimpleActionServer<iirob_led::PoliceAction> policeAS;  ///< Handles Police goal messages
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
