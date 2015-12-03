#ifndef IIROB_LED_CROSS_H
#define IIROB_LED_CROSS_H

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "iirob_led_base.h" ///< For the class inheritance

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
#define NONE                    0   // x = y = 0
#define TOP                     1   // +y | corresponds to the +x of the rectangular LED strip (front)
#define BOTTOM                  2   // -y | corresponds to the -x of the rectangular LED strip (back)
#define LEFT                    3   // +x | corresponds to the +y of the rectangular LED strip (left)
#define RIGHT                   4   // -x | corresponds to the -y of the rectangular LED strip (right)

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

#define MAX_NUM_LEDS_CROSS 7   ///< 7 + 1 (center) LEDs (note that the upper and bottom halves of the vertical LED strip actually have 8 and not 7 (like their horizontal counterparts) but we want to keep the symmetry)

/**
 * @brief The IIROB_LED_Cross class controls the LED strip mounted around the edges of the bottom platform of the SR2
 */
class IIROB_LED_Cross : public IIROB_LED_Base
{
private:
    tf2_ros::Buffer *buf;
    tf2_ros::TransformListener *tfl;

    // Note: The indexing starts from 0 and ends at 31 for the cross.

public:
    /**
     * @brief IIROB_LED_Cross Constructor that initializes the hardware
     * @param nodeHandle
     * @param _port Port as string
     * @param _m_numLeds Number of LEDs
     */
    IIROB_LED_Cross(ros::NodeHandle nodeHandle, std::string const& _port, int const& _mNumLeds, double maxForce, std::string link);

    /**
     * @brief Destructor turns off all LEDs and shuts down all action servers and subscribers
     */
    ~IIROB_LED_Cross();

    // Callbacks for all action servers and subscribers
    /**
     * @brief forceCallback
     * @param led_force_msg
     */
    void forceCallback(const iirob_led::DirectionWithForce::ConstPtr& led_force_msg) override final;

    /**
     * @brief policeCallback processes PoliceActionGoal messages - it turns on and off a give stripe of LEDs mimicing a police light
     * @param goal
     */
    void policeCallback(const iirob_led::PoliceGoal::ConstPtr& goal) override final;
};

#endif // IIROB_LED_CROSS_H
