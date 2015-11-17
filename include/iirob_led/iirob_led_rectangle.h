#ifndef IIROB_LED_RECTANGLE_H
#define IIROB_LED_RECTANGLE_H
#include <iirob_led/DirectionWithForce.h>
#include <iirob_led/PoliceAction.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "iirob_led_base.h"

/*
 * ============================================================
 * NOTES:
 * ============================================================
 *
 * SR2 rectangle devided into quadrants:
 *
 *                  LEFT
 *                  0,+Y
 *                    |
 *   (2nd quadrant)   |     (1st quadrant)
 * CORNER_BACK_LEFT --|-- CORNER_FRONT_LEFT
 *         |                       |
 *      -X,0          x           +X,0___________[key switch]
 *      BACK    (none quadrant)  FRONT
 *         |                       |
 * CORNER_BACK_RIGHT--|-- CORNER_FRONT_RIGHT
 *   (3rd quadrant)   |     (4th quadrant)
 *                    |
 *                  0,-Y
 *                 RIGHT
 *
 * FRONT              +x, y = 0
 * BACK               -x, y = 0
 * LEFT               x = 0, +y
 * RIGHT              x = 0, -y
 *
 * 1st quadrant: +x, +y
 * 2nd quadrant: -x, +y
 * 3rd quadrant: -x, -y
 * 4th quadrant: +x, -y
 */

// Rectangle strip length of each side
#define RECT_LONG_SIDE              108
#define RECT_SHORT_SIDE             84

// Rectangle strip length
#define RECT_START                  0
#define RECT_END                    ((2 * RECT_LONG_SIDE) + (2 * RECT_SHORT_SIDE))

// Rectangle sides (one of two axes == 0)
#define RECT_FRONT                  (RECT_LONG_SIDE + RECT_SHORT_SIDE + RECT_LONG_SIDE + (int)(RECT_SHORT_SIDE / 2))
#define RECT_BACK                   (RECT_LONG_SIDE + (int)(RECT_SHORT_SIDE / 2))
#define RECT_LEFT                   ((int)(RECT_LONG_SIDE / 2))
#define RECT_RIGHT                  ((RECT_LONG_SIDE - 1) + RECT_SHORT_SIDE + (int)(RECT_LONG_SIDE / 2))

// Rectangle corners
#define RECT_CORNER_FRONT_RIGHT     (RECT_LONG_SIDE + RECT_SHORT_SIDE + RECT_LONG_SIDE)
#define RECT_CORNER_FRONT_LEFT      (RECT_END - 1)
#define RECT_CORNER_BACK_RIGHT      ((RECT_LONG_SIDE - 1) + RECT_SHORT_SIDE)
#define RECT_CORNER_BACK_LEFT       (RECT_LONG_SIDE)

/**
 * @brief The IIROB_LED_Rectangle class controls the LED strip mounted around the edges of the bottom platform of the SR2
 */
class IIROB_LED_Rectangle : public IIROB_LED_Base
{
private:
    std::string local_frame;

    tf2_ros::Buffer *buf;
    tf2_ros::TransformListener *tfl;

    actionlib::SimpleActionServer<iirob_led::PoliceAction> policeAS;  ///< Handles Police goal messages
    ros::Subscriber subForce;           ///< Gives visual feedback for the magnitude and direction of an applied force (represented as a 3D vector)
    ros::Publisher pubForceTransformed; ///< Publishes the force retrieved by the subscriber but in the local frame
public:
    /**
     * @brief IIROB_LED_Rectangle Constructor that initializes the hardware
     * @param nodeHandle
     * @param _port Port as string
     * @param _m_numLeds Number of LEDs
     */
    IIROB_LED_Rectangle(ros::NodeHandle nodeHandle, std::string const& _port, int const& _m_numLeds, std::string link);

    /**
     * @brief Destructor turns off all LEDs and shuts down all action servers and subscribers
     */
    ~IIROB_LED_Rectangle();

    // Callbacks for all action servers and subscribers
    /**
     * @brief forceCallback Retrieves a vector with a TF2 frame and lights up one of the corners of the platform based on the force and its direction represented by the vector
     * @param led_force_msg
     */
    void forceCallback(const iirob_led::DirectionWithForce::ConstPtr& led_force_msg);

    /**
     * @brief policeCallback processes PoliceActionGoal messages - it turns on and off a give stripe of LEDs mimicing a police light
     * @param goal
     */
    void policeCallback(const iirob_led::PoliceGoal::ConstPtr& goal);
};

#endif // IIROB_LED_RECTANGLE_H
