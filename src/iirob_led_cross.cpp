#include <cmath>
#include "iirob_led_cross.h"
#include "RGBConverter.h"

// TODO Check if everywhere the duration parameter of a message is set properly (value > 0). Otherwise duration takes the default value from the message (=0), which equals infinity and node can only be stopped by escalating to SIGTERM

IIROB_LED_Cross::IIROB_LED_Cross(ros::NodeHandle nodeHandle, std::string const& _port, int const& _m_numLeds)
    : horizontal_side(7), vertical_side(8),
      horizontal_no_center(horizontal_side*2), horizontal_all(horizontal_no_center+1),
      vertical_no_center(vertical_side*2), vertical_all(vertical_no_center+1),
      led_right(horizontal_side-1), led_left(horizontal_side),
      led_forward_top(horizontal_side*2),
      led_center(led_forward_top + vertical_side),
      led_backward_bottom(led_center + vertical_side),
      policeAS(nodeHandle, "police", boost::bind(&IIROB_LED_Cross::policeCallback, this, _1), false),
      IIROB_LED_Base::IIROB_LED_Base(nodeHandle, _port, _m_numLeds)
{
    policeAS.start();
    ROS_INFO("police action server started");
    ROS_INFO("led_force_cross subscriber started");
    subForce = nodeHandle.subscribe("led_force_cross", 10, &IIROB_LED_Cross::forceCallback, this);
}

IIROB_LED_Cross::~IIROB_LED_Cross() {
    ROS_INFO("Turning all LEDs off");
    // Turn all LEDs off and delete the m_led
    /*if (m_led) {
        m_led->setAllRGBf(0, 0, 0, m_numLeds);
        m_led->setUniRGB(0, 0, 0);
        delete m_led;
    }*/

    ROS_INFO("Shutting down police action server and led_force_rectangle subscriber");
    policeAS.shutdown();
    subForce.shutdown();
}

// Callbacks for all action servers and subscribers
void IIROB_LED_Cross::forceCallback(const iirob_led::DirectionWithForce::ConstPtr& led_force_msg) {
    ROS_INFO("Lighting up complete cross");
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
    m_led->setAllRGBf(0, 0, 0, m_numLeds);
}

void IIROB_LED_Cross::policeCallback(const iirob_led::PoliceGoal::ConstPtr& goal) {
    iirob_led::PoliceFeedback feedback;
    iirob_led::PoliceResult result;
    int blinks_left = goal->blinks;
    int fast_blinks_left = goal->fast_blinks;
    // We hardcode the start, end and number of LEDs for the cross
    //int start_led = goal->start_led;
    //int end_led = goal->end_led;
    //int num_inner_leds = goal->num_inner_leds;

    //if(start_led < 0) start_led = 0;
    //if(end_led >= m_numLeds) end_led = m_numLeds;
    checkLimits(&start_led, &end_led);

    int totLength = (end_led - start_led);

    // Probably this here needs some rework
    //if(start_led > end_led) { int temp = start_led; start_led = end_led; end_led = temp; }

    double half = totLength/2;
    int start_led_left_outer = start_led;
    int end_led_left_outer = start_led + ((int)half - num_inner_leds);

    int start_led_left_inner = end_led_left_outer;
    int end_led_left_inner = start_led + (int)half;

    int start_led_right_inner = end_led_left_inner;
    int end_led_right_inner = start_led_right_inner + num_inner_leds - 1;

    int start_led_right_outer = end_led_right_inner;
    int end_led_right_outer = end_led;

    ROS_INFO("start_led_left_outer: %d", start_led_left_outer);
    ROS_INFO("end_led_left_outer: %d", end_led_left_outer);

    ROS_INFO("start_led_left_inner: %d", start_led_left_inner);
    ROS_INFO("end_led_left_inner: %d", end_led_left_inner);

    ROS_INFO("start_led_right_inner: %d", start_led_right_inner);
    ROS_INFO("end_led_right_inner: %d", end_led_right_inner);

    ROS_INFO("start_led_right_outer: %d", start_led_right_outer);
    ROS_INFO("end_led_right_outer: %d", end_led_right_outer);


    int j;
    for(int i = 0; i < goal->blinks; ++i, --blinks_left)
    {
        ROS_INFO(" -- Long blinks left: %d", blinks_left);
        for(j = 0; j < goal->fast_blinks; ++j, --fast_blinks_left)
        {
            ROS_INFO(" -- Short blinks left: %d", fast_blinks_left);
            // Blink the outer subsections
            m_led->setRangeRGBf(goal->color_outer.r, goal->color_outer.g, goal->color_outer.b, m_numLeds, start_led_left_outer, end_led_left_outer);
            m_led->setRangeRGBf(goal->color_outer.r, goal->color_outer.g, goal->color_outer.b, m_numLeds, start_led_right_outer, end_led_right_outer);
            ros::Duration(goal->fast_duration_on).sleep();

            m_led->setRangeRGBf(0, 0, 0, m_numLeds, start_led_left_outer, end_led_left_outer);
            m_led->setRangeRGBf(0, 0, 0, m_numLeds, start_led_right_outer, end_led_right_outer);
            ros::Duration(goal->fast_duration_off).sleep();

            // Blink the inner subsections
            m_led->setRangeRGBf(goal->color_inner.r, goal->color_inner.g, goal->color_inner.b, m_numLeds, start_led_left_inner, end_led_left_inner);
            m_led->setRangeRGBf(goal->color_inner.r, goal->color_inner.g, goal->color_inner.b, m_numLeds, start_led_right_inner, end_led_right_inner);
            ros::Duration(goal->fast_duration_on).sleep();

            m_led->setRangeRGBf(0, 0, 0, m_numLeds, start_led_left_inner, end_led_left_inner);
            m_led->setRangeRGBf(0, 0, 0, m_numLeds, start_led_right_inner, end_led_right_inner);
            ros::Duration(goal->fast_duration_off).sleep();

            // Send feedback
            feedback.fast_blinks_left = fast_blinks_left;
            feedback.blinks_left = blinks_left;
            policeAS.publishFeedback(feedback);
        }

        // Make a pause before the next set of fast blinks
        ros::Duration(goal->pause_between_long_blinks).sleep();
        fast_blinks_left = goal->fast_blinks;
    }

    //m_led->setAllRGBf(0, 0, 0, m_numLeds);
    result.blinks_left = blinks_left;
    policeAS.setSucceeded(result);
}
