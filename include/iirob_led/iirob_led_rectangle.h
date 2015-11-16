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

tf2_ros::Buffer *p_tfBuffer;
    tf2_ros::TransformListener* p_tfListener;
    tf2::Transform transform_ee_base;
    geometry_msgs::TransformStamped transform_ee_base_stamped;


p_tfBuffer = new tf2_ros::Buffer();
    p_tfListener = new tf2_ros::TransformListener(*p_tfBuffer, true);

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

 */

/*
 * ============================================================
 * NOTES:
 * ============================================================
 *
 * SR2 rectangle devided into quadrants:
 *
 *
 *
 *                  LEFT
 *                X=0,Y=+1
 *                    |
 *   (3rd quadrant)   |     (2nd quadrant)
 * CORNER_BACK_LEFT --|-- CORNER_FRONT_LEFT
 *         |                       |
 *    X=-1,Y=0           x      X=+1,Y=0___________[key switch]
 *      BACK    (none quadrant)  FRONT
 *         |                       |
 * CORNER_BACK_RIGHT--|-- CORNER_FRONT_RIGHT
 *   (4th quadrant)   |     (1st quadrant)
 *                    |
 *                X=0,Y=-1
 *                 RIGHT
 *
 * QUADRANT_NONE      x = y = 0
 * FRONT              +x, y = 0
 * BACK               -x, y = 0
 * LEFT               x = 0, +y
 * RIGHT              x = 0, -y
 * QUADRANT_FIRST     1st quadrant: +x, +y
 * QUADRANT_SECOND    2nd quadrant: -x, +y
 * QUADRANT_THIRD     3rd quadrant: -x, -y
 * QUADRANT_FOURTH    4th quadrant: +x, -y
 */

// Rectangle strip length of each side
#define RECT_LONG_SIDE              108
#define RECT_SHORT_SIDE             84

// Rectangle strip length
#define RECT_START                  0
#define RECT_END                    ((2 * RECT_LONG_SIDE) + (2 * RECT_SHORT_SIDE))

// Rectangle sides (one of two axes == 0)
/*#define RECT_FRONT                  (RECT_LONG_SIDE + RECT_SHORT_SIDE + RECT_LONG_SIDE + (int)(RECT_SHORT_SIDE / 2))
#define RECT_BACK                   (RECT_LONG_SIDE + (int)(RECT_SHORT_SIDE / 2))
#define RECT_LEFT                   ((int)(RECT_LONG_SIDE / 2))*/
#define RECT_RIGHT                  ((RECT_LONG_SIDE - 1) + RECT_SHORT_SIDE + (int)(RECT_LONG_SIDE / 2))

// Rectangle corners
#define RECT_CORNER_FRONT_RIGHT     (RECT_LONG_SIDE + RECT_SHORT_SIDE + RECT_LONG_SIDE)
#define RECT_CORNER_FRONT_LEFT      (RECT_END - 1)
#define RECT_CORNER_BACK_RIGHT      ((RECT_LONG_SIDE - 1) + RECT_SHORT_SIDE)
#define RECT_CORNER_BACK_LEFT       (RECT_LONG_SIDE)

//#define QUADRANT_NONE               0
#define FRONT                       1
#define BACK                        2
#define LEFT                        3
#define RIGHT                       4
/*#define QUADRANT_FIRST              5
#define QUADRANT_SECOND             6
#define QUADRANT_THIRD              7
#define QUADRANT_FOURTH             8*/

/**
 * @brief The IIROB_LED_Rectangle class controls the LED strip mounted around the edges of the bottom platform of the SR2
 */
class IIROB_LED_Rectangle : public IIROB_LED_Base
{
private:
    std::string local_frame;

    tf2_ros::Buffer *buf;
    tf2_ros::TransformListener *tfl;
    tf2_ros::TransformBroadcaster *tfb;

    actionlib::SimpleActionServer<iirob_led::PoliceAction> policeAS;  ///< Handles Police goal messages
    ros::Subscriber subForce;           ///< Gives visual feedback for the magnitude and direction of an applied force (represented as a 3D vector)
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
