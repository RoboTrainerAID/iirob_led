#ifndef IIROB_LED_RECTANGLE_H
#define IIROB_LED_RECTANGLE_H

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "iirob_led_base.h"

/*
 * We combine the LED indexing (front right corner: LED[0] ... back right corner: LED[0+108] ... ... ... front right corner: LED[383])
 * and the degrees we get from the atan2 function
 * We observe two major cases here:
 * 1)one of the axes is equal to 0 - in this case we don't need to calculate atan2() and we can directly go to the LED indexing part (this can also be done for other scenarios too such as when we hit the corners of the rectangle; see the macros RECT_CORNER_XXX)
 * 2)both axes are unequal to 0 - in this case we have to calculate atan2()
 *
 * The way the platform currently moves is as follows:
 * -y : move to the right (the side where the kinect is mounted)
 * +y : move to the left
 * +x : move forwards (the side where the on/off key is)
 * -x : move backwards
 *
 * We define a coordinate system with its center being located in the center of the platform and
 * its first quadrant (+x,+y) being located at the fron
 *
 *               FRONT (0 DEG) [key_switch]
 *   [0],[383]    +x        ______SR2
 *         \  _____|_____  / __________unit circle
 *          \/     |     \/ /
 *          /|-----|-----|\/
 *         / |     |     | \
 *         | |     |     | |
 *LEFT +y__|_|_____0_____|_|__-y RIGHT
 *(+90 DEG,| |     |     | |     (+270 DEG, 90 DEG)
 *-270 DEG)| |     |     | |
 *         \ |     |     | /
 *          \|-----|-----|/
 *           \_____|_____/
 *                 |
 *                -x
 *               BACK
 *             (+180 DEG, 180 DEG)
 *
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

#define MAX_NUM_LEDS_RECTANGLE 12

/**
 * @brief The IIROB_LED_Rectangle class controls the LED strip mounted around the edges of the bottom platform of the SR2
 */
class IIROB_LED_Rectangle : public IIROB_LED_Base
{
private:
    tf2_ros::Buffer *buf;
    tf2_ros::TransformListener *tfl;

public:
    /**
     * @brief IIROB_LED_Rectangle Constructor that initializes the hardware
     * @param nodeHandle
     * @param _port Port as string
     * @param _mNumLeds Number of LEDs
     */
    IIROB_LED_Rectangle(ros::NodeHandle nodeHandle, std::string const& port, int const& mNumLeds, double maxForce, std::string link);

    /**
     * @brief Destructor turns off all LEDs and shuts down all action servers and subscribers
     */
    ~IIROB_LED_Rectangle();

    // Callbacks for all action servers and subscribers
    /**
     * @brief forceCallback Retrieves a vector with a TF2 frame and lights up one of the corners of the platform based on the force and its direction represented by the vector
     * @param led_force_msg
     */
    void forceCallback(const iirob_led::DirectionWithForce::ConstPtr& ledForceMsg) override final;

    /**
     * @brief policeCallback processes PoliceActionGoal messages - it turns on and off a give stripe of LEDs mimicing a police light
     * @param goal
     */
    void policeCallback(const iirob_led::PoliceGoal::ConstPtr& goal) override final;
};

#endif // IIROB_LED_RECTANGLE_H
