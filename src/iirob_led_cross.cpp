#include <cmath>
#include "iirob_led_cross.h"
#include "RGBConverter.h"

// TODO Check if everywhere the duration parameter of a message is set properly (value > 0). Otherwise duration takes the default value from the message (=0), which equals infinity and node can only be stopped by escalating to SIGTERM

IIROB_LED_Cross::IIROB_LED_Cross(ros::NodeHandle nodeHandle, std::string const& _port, int const& _m_numLeds, std::string link)
    : policeAS(nodeHandle, "police", boost::bind(&IIROB_LED_Cross::policeCallback, this, _1), false),
      local_frame(link),
      IIROB_LED_Base::IIROB_LED_Base(nodeHandle, _port, _m_numLeds)
{
    policeAS.start();
    ROS_INFO("police action server started");
    ROS_INFO("led_force_cross subscriber started");
    subForce = nodeHandle.subscribe("led_force_cross", 10, &IIROB_LED_Cross::forceCallback, this);
    pubForceTransformed = nodeHandle.advertise<iirob_led::DirectionWithForce>("led_force_cross_local", 1);
    ROS_INFO("led_force_cross_local publisher started");


    // Initialize the tf2-related components
    buf = new tf2_ros::Buffer();
    tfl = new tf2_ros::TransformListener(*buf, nodeHandle);
}

IIROB_LED_Cross::~IIROB_LED_Cross() {
    ROS_INFO("Shutting down police action server and led_force_cross subscriber");
    policeAS.shutdown();
    subForce.shutdown();

    delete buf;
    delete tfl;
}

// Callbacks for all action servers and subscribers
void IIROB_LED_Cross::forceCallback(const iirob_led::DirectionWithForce::ConstPtr& led_force_msg) {
    // TEST
    /*ROS_INFO("Lighting up complete cross");
    m_led->setRangeRGBf(1, 0, 0, m_numLeds, CROSS_START, CROSS_END);
    ros::Duration(5).sleep();
    m_led->setAllRGBf(0, 0, 0, m_numLeds);

    ROS_INFO("Lighting up cross' center");
    m_led->setRangeRGBf(1, 0, 0, m_numLeds, CROSS_CENTER, CROSS_CENTER);
    ros::Duration(5).sleep();
    m_led->setAllRGBf(0, 0, 0, m_numLeds);

    ROS_INFO("Lighting up cross' horizontal");
    m_led->setRangeRGBf(1, 0, 0, m_numLeds, H_START, H_END);
    ros::Duration(5).sleep();
    m_led->setAllRGBf(0, 0, 0, m_numLeds);

    ROS_INFO("Lighting up cross' vertical");
    m_led->setRangeRGBf(1, 0, 0, m_numLeds, V_START, V_END);
    ros::Duration(5).sleep();
    m_led->setAllRGBf(0, 0, 0, m_numLeds);

    ROS_INFO("Lighting up cross' horizontal +x (left)");
    m_led->setRangeRGBf(1, 0, 0, m_numLeds, H_LEFT_XPLUS_START, H_LEFT_XPLUS_END);
    ros::Duration(5).sleep();
    m_led->setAllRGBf(0, 0, 0, m_numLeds);

    ROS_INFO("Lighting up cross' horizontal -x (right)");
    m_led->setRangeRGBf(1, 0, 0, m_numLeds, H_RIGHT_XMINUS_START, H_RIGHT_XMINUS_END);
    ros::Duration(5).sleep();
    m_led->setAllRGBf(0, 0, 0, m_numLeds);

    ROS_INFO("Lighting up cross' vertical +y (top)");
    m_led->setRangeRGBf(1, 0, 0, m_numLeds, V_UPPER_YPLUS_START, V_UPPER_YPLUS_END);
    ros::Duration(5).sleep();
    m_led->setAllRGBf(0, 0, 0, m_numLeds);

    ROS_INFO("Lighting up cross' vertical -y (bottom)");
    m_led->setRangeRGBf(1, 0, 0, m_numLeds, V_BOTTOM_YMINUS_START, V_BOTTOM_YMINUS_END);
    ros::Duration(5).sleep();
    m_led->setAllRGBf(0, 0, 0, m_numLeds);*/

    // Calculate the force
    double x = led_force_msg->force.x;
    double y = led_force_msg->force.y;

    if(x == 0 && y == 0)
    {
        ROS_WARN("Both x and y coordinates equal zero which does not allow visualization in 2D space.");
        return;
    }

    double force = sqrt(pow(led_force_msg->force.x, 2) + pow(led_force_msg->force.y, 2));// + pow(led_force_msg->force.z, 2));
    // Scale the received force to be in the interval between 0 and maxForce (maxFroce rounded up and converted to an integer - it will represent the number of LEDs to be lit)
    // [0] ----- [force] -- [maxForce]
    //int forceRounded = (int)ceil(force);      // 1.5 becomes 2
    int forceRounded = (int)round(force);       // 1.5 becomes 2 but 1.3 becomes 1
    ROS_INFO("Max force: %d", MAX_FORCE_CROSS);
    ROS_INFO("Force: %.3f | Rounded and int: %d", force, forceRounded);
    if(forceRounded > MAX_FORCE_CROSS)
    {
        ROS_ERROR("Received force is of greater magnitude than the set upper bound or exceed the numer of LEDs that can be displayed on each side of the platform | forceRounded(%d), forceMax(%d)", forceRounded, MAX_FORCE_CROSS);
        return;
    }

    m_led->setAllRGBf(0, 0, 0, m_numLeds);

    bool singleAxes = ((!x  && y) || (x && !y));    // If true, then either x or y equal 0
    if(forceRounded == 1)
    {
        m_led->setRangeRGBf(0, 0, 1, m_numLeds, CROSS_CENTER, CROSS_CENTER);
        return;
    }

    forceRounded--;

    if(!singleAxes)
    {
        if(x > 0 && y > 0)
        {
            m_led->setRangeRGBf(1, 0, 0, m_numLeds, H_LEFT_XPLUS_START, H_LEFT_XPLUS_START - 1 + forceRounded, true, true, false);  // +x
            m_led->setRangeRGBf(1, 0, 0, m_numLeds, V_UPPER_YPLUS_START + (V_SIDE - forceRounded), V_UPPER_YPLUS_END);              // +y
        }
        else if(x > 0 && y < 0)
        {
            m_led->setRangeRGBf(1, 0, 0, m_numLeds, H_LEFT_XPLUS_START, H_LEFT_XPLUS_START - 1 + forceRounded, true, true, false);  // +x
            m_led->setRangeRGBf(1, 0, 0, m_numLeds, V_BOTTOM_YMINUS_START, V_BOTTOM_YMINUS_START - 1 + forceRounded);               // -y
        }
        else if(x < 0 && y > 0)
        {
            m_led->setRangeRGBf(1, 0, 0, m_numLeds, H_RIGHT_XMINUS_START + (H_SIDE - forceRounded), H_RIGHT_XMINUS_END, true, true, false); // -x
            m_led->setRangeRGBf(1, 0, 0, m_numLeds, V_UPPER_YPLUS_START + (V_SIDE - forceRounded), V_UPPER_YPLUS_END);                      // +y
        }
        else if(x < 0 && y < 0)
        {
            m_led->setRangeRGBf(1, 0, 0, m_numLeds, H_RIGHT_XMINUS_START + (H_SIDE - forceRounded), H_RIGHT_XMINUS_END, true, true, false); // -x
            m_led->setRangeRGBf(1, 0, 0, m_numLeds, V_BOTTOM_YMINUS_START, V_BOTTOM_YMINUS_START - 1 + forceRounded);                       // -y
        }
    }
    else
    {
        if(x == 0 && y < 0) m_led->setRangeRGBf(1, 0, 0, m_numLeds, V_BOTTOM_YMINUS_START, V_BOTTOM_YMINUS_START - 1 + forceRounded);
        else if(x == 0 && y > 0) m_led->setRangeRGBf(1, 0, 0, m_numLeds, V_UPPER_YPLUS_START + (V_SIDE - forceRounded), V_UPPER_YPLUS_END);
        else if(y == 0 && x > 0) m_led->setRangeRGBf(1, 0, 0, m_numLeds, H_LEFT_XPLUS_START, H_LEFT_XPLUS_START - 1 + forceRounded);
        else if(y == 0 && x < 0) m_led->setRangeRGBf(1, 0, 0, m_numLeds, H_RIGHT_XMINUS_START + (H_SIDE - forceRounded), H_RIGHT_XMINUS_END);
    }

    m_led->setRangeRGBf(0, 0, 1, m_numLeds, CROSS_CENTER, CROSS_CENTER);

    // TODO From here on things will be differen for the cross
    /*int direction;
    double angle;
    double ledPerDeg;
    double translationAlongStrip;
    bool singleAxes = ((!x  && y) || (x && !y));    // If true, then either x or y equal 0
    if(!singleAxes)
    {
        angle = atan2(y,x);  // Angle in radians
        ledPerDeg = m_numLeds/360.;
        translationAlongStrip = (angle*180./M_PI) * ledPerDeg;
        direction = (int)round(RECT_FRONT + translationAlongStrip) % m_numLeds;
    }
    else
    {
        if(x == 0 && y < 0) direction = RECT_RIGHT;
        else if(x == 0 && y > 0) direction = RECT_LEFT;
        else if(y == 0 && x > 0) direction = RECT_FRONT;
        else if(y == 0 && x < 0) direction = RECT_BACK;
    }

    ROS_INFO("XY coordinates: [%.3f , %.3f]\t|\tForce (upper limit of %d): %.3f", x, y, MAX_FORCE_CROSS, force);
    ROS_INFO("Led/Angle: %f | Angle: %frad (=%fdeg) | Translation (num of LEDs): %f | location (LED index): %d", ledPerDeg, angle, angle*180./M_PI, translationAlongStrip, direction);

    m_led->setAllRGBf(0, 0, 0, m_numLeds);

    if(forceRounded == 1)
    {
        m_led->setRangeRGBf(0, 0, 1, m_numLeds, direction, direction);
        return;
    }

    forceRounded--;

    m_led->setRangeRGBf(1, 0, 0, m_numLeds,
                        ((direction - forceRounded) > 0) ? (direction - forceRounded) : m_numLeds - (forceRounded - direction),
                        (direction + forceRounded),
                        true, true, false);
    m_led->setRangeRGBf(0, 0, 1, m_numLeds, direction, direction);

    // Process the transformation information
    geometry_msgs::Vector3Stamped forceIn, forceOut;
    forceIn.vector = led_force_msg->force;
    geometry_msgs::TransformStamped tf_stamped;
    iirob_led::DirectionWithForce led_force_msg_transformed;
    try
    {
        tf_stamped = buf->lookupTransform(local_frame, led_force_msg_transformed.tf_stamped.header.frame_id, ros::Time(0));
        tf2::doTransform(forceIn, forceOut, tf_stamped);
        ROS_INFO("Transforming vec(x: %f, y: %f, z: %f) with trans(x: %f, y: %f, z: %f), rot(x: %f, y: %f, z: %f)",
                 forceIn.vector.x, forceIn.vector.y, forceIn.vector.z,
                 tf_stamped.transform.translation.x, tf_stamped.transform.translation.y, tf_stamped.transform.translation.z,
                 tf_stamped.transform.rotation.x, tf_stamped.transform.rotation.y, tf_stamped.transform.rotation.z);
    }
    catch(tf2::TransformException ex)
    {
        ROS_WARN("%s", ex.what());
    }
    led_force_msg_transformed.tf_stamped.header = led_force_msg->tf_stamped.header;
    led_force_msg_transformed.tf_stamped.header.frame_id = local_frame;
    led_force_msg_transformed.tf_stamped.transform = tf_stamped.transform;
    led_force_msg_transformed.force = forceOut.vector;
    pubForceTransformed.publish(led_force_msg_transformed);*/
}

void IIROB_LED_Cross::policeCallback(const iirob_led::PoliceGoal::ConstPtr& goal) {
    iirob_led::PoliceFeedback feedback;
    iirob_led::PoliceResult result;
    int blinks_left = goal->blinks;
    int fast_blinks_left = goal->fast_blinks;
    // NOTE: We ignore the start, end and number of LEDs and hardcode the behaviour for this action

    int j;
    for(int i = 0; i < goal->blinks; ++i, --blinks_left)
    {
        ROS_INFO(" -- Long blinks left: %d", blinks_left);
        for(j = 0; j < goal->fast_blinks; ++j, --fast_blinks_left)
        {
            ROS_INFO(" -- Short blinks left: %d", fast_blinks_left);
            // Blink the outer subsections
            m_led->setRangeRGBf(goal->color_outer.r, goal->color_outer.g, goal->color_outer.b, m_numLeds, V_UPPER_YPLUS_START, V_UPPER_YPLUS_END, true, true, false);
            m_led->setRangeRGBf(goal->color_outer.r, goal->color_outer.g, goal->color_outer.b, m_numLeds, V_BOTTOM_YMINUS_START, V_BOTTOM_YMINUS_END);
            ros::Duration(0.25).sleep();

            m_led->setRangeRGBf(0, 0, 0, m_numLeds, V_UPPER_YPLUS_START, V_UPPER_YPLUS_END, true, true, false);
            m_led->setRangeRGBf(0, 0, 0, m_numLeds, V_BOTTOM_YMINUS_START, V_BOTTOM_YMINUS_END);
            ros::Duration(0.05).sleep();

            // Blink the inner subsections
            m_led->setRangeRGBf(goal->color_inner.r, goal->color_inner.g, goal->color_inner.b, m_numLeds, H_RIGHT_XMINUS_START, H_RIGHT_XMINUS_END, true, true, false);
            m_led->setRangeRGBf(goal->color_inner.r, goal->color_inner.g, goal->color_inner.b, m_numLeds, H_LEFT_XPLUS_START, H_LEFT_XPLUS_END);
            ros::Duration(0.25).sleep();

            m_led->setRangeRGBf(0, 0, 0, m_numLeds, H_RIGHT_XMINUS_START, H_RIGHT_XMINUS_END, true, true, false);
            m_led->setRangeRGBf(0, 0, 0, m_numLeds, H_LEFT_XPLUS_START, H_LEFT_XPLUS_END);
            ros::Duration(0.05).sleep();

            // Send feedback
            feedback.fast_blinks_left = fast_blinks_left;
            feedback.blinks_left = blinks_left;
            policeAS.publishFeedback(feedback);
        }

        // Make a pause before the next set of fast blinks
        ros::Duration(2.0).sleep();
        fast_blinks_left = goal->fast_blinks;
    }

    //m_led->setAllRGBf(0, 0, 0, m_numLeds);
    result.blinks_left = blinks_left;
    policeAS.setSucceeded(result);
}
