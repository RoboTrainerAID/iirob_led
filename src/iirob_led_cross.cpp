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
      IIROB_LED_Base::IIROB_LED_Base(nodeHandle, _port, _m_numLeds)
{
    ROS_INFO("led_force_cross subscriber started");
    subForce = nodeHandle.subscribe("led_force_cross", 10, &IIROB_LED_Cross::forceCallback, this);
}

IIROB_LED_Cross::~IIROB_LED_Cross() {
    ROS_INFO("Turning all LEDs off");
    // Turn all LEDs off and delete the m_led
    if (m_led) {
        m_led->setAllRGBf(0, 0, 0, m_numLeds);
        m_led->setUniRGB(0, 0, 0);
        delete m_led;
    }

    ROS_INFO("Shutting down all action servers and subscribers");
    subForce.shutdown();
}

// Callbacks for all action servers and subscribers
void IIROB_LED_Cross::forceCallback(const iirob_led::DirectionWithForce::ConstPtr& led_force_msg) {
    // Calculate the force
    /*double x = led_force_msg->force.x;
    double y = led_force_msg->force.y;
    double force = sqrt(pow(led_force_msg->force.x, 2) + pow(led_force_msg->force.y, 2) + pow(led_force_msg->force.z, 2));
    // Scale the received force to be in the interval between 0 and maxForce (maxFroce rounded up and converted to an integer - it will represent the number of LEDs to be lit)
    // [0] ----- [force] -- [maxForce]
    //int forceRounded = (int)ceil(force);   // 1.5 becomes 2.0 and then 2
    int forceRounded = (int)round(force);   // 1.5 becomes 2.0 and then 2
    ROS_INFO("Max force: %d", MAX_FORCE);
    ROS_INFO("Force: %.3f | Rounded and int: %d", force, forceRounded);
    if(forceRounded > MAX_FORCE) {
        ROS_ERROR("Received force is of greater magnitude than the set upper bound or exceed the numer of LEDs that can be displayed on each side of the platform | forceRounded(%d), forceMax(%d)", forceRounded, MAX_FORCE);
        return;
    }

    // Determine quadrant and corner that will display the force
    int8_t quadrant;
    int corner;
    // TODO Test for case such as x^2 + y^2 + z^2 > 0 and < 1- Should such cases be excluded or the minimal force that can be displayed is 1? Right now I'm ceiling the force hence 0.1 ~ 0.2 ... ~ 0.9 ~ 1.0
    if((x >= 0 && y > 0) || (x > 0 && y >= 0)) quadrant = QUADRANT_FIRST;
    else if(x < 0 && y >= 0) quadrant = QUADRANT_SECOND;
    else if(x < 0 && y < 0) quadrant = QUADRANT_THIRD;
    else if(x >= 0 && y < 0) quadrant = QUADRANT_FOURTH;
    else quadrant = QUADRANT_NONE;  // x = y = 0
    ROS_INFO("XY coordinates: [%.3f , %.3f] (quadrant %d)\t|\tForce (upper limit of %d): %.3f", x, y, quadrant, MAX_FORCE, force);

    switch(quadrant) {
    case QUADRANT_NONE:
        // If X and Y are equal to 0 we cannot determine the corner hence no point in displaying anything
        return;
    case QUADRANT_FIRST:
        corner = led_corner_front_right;
        break;
    case QUADRANT_SECOND:
        corner = led_corner_front_left;
        break;
    case QUADRANT_THIRD:
        corner = led_corner_back_left;
        break;
    case QUADRANT_FOURTH:
        corner = led_corner_back_right;
        break;
    }*/

    /*
     * Corner:
     *
     *                corner------[forceRoundend]
     *                  |
     *                  |
     *                  |
     *           [forceRoundend]
     */

    // Counting direction is again but starting from the last LED: 384, 0, 1, ... , 383
    /*int rangeBeforeCorner;
    int rangeAfterCorner;

    m_led->setRangeRGBf(0, 0, 0, m_numLeds, 383, 383);
    m_led->setRangeRGBf(0, 0, 0, m_numLeds, 108, 108);
    m_led->setRangeRGBf(0, 0, 0, m_numLeds, 84+108-1, 84+108-1);
    m_led->setRangeRGBf(0, 0, 0, m_numLeds, 84+2*108, 84+2*108);

    m_led->setRangeRGBf(1, 0, 0, m_numLeds, 383, 383);
    m_led->setRangeRGBf(0, 1, 0, m_numLeds, 108, 108);
    m_led->setRangeRGBf(0, 0, 1, m_numLeds, 84+108-1, 84+108-1);
    m_led->setRangeRGBf(1, 1, 0, m_numLeds, 84+2*108, 84+2*108);*/

    /*switch(corner) {
    case led_corner_front_right:
        ROS_INFO("CORNER_FRONT_RIGHT");
        if(forceRounded == 1) m_led->setRangeRGBf(1, 0, 0, m_numLeds, 380, 380);
        else {
            // Reduce the forceRound by one since the corner will display part of it
            forceRounded -= 1;
            // front to corner
            rangeBeforeCorner = corner - forceRounded;
            m_led->setRangeRGBf(1, 1, 1, m_numLeds, 380, 384);
            ROS_INFO("[%d : %d)", corner-forceRounded, corner);
            //m_led->setRangeRGBf(1, 1, 1, m_numLeds, corner-forceRounded, corner);
            // corner
            ROS_INFO("[%d]", corner);
            //m_led->setRangeRGBf(0, 1, 0, m_numLeds, corner, corner);
            // corner to back
            ROS_INFO("(%d : %d]", corner, corner+forceRounded);
            //m_led->setRangeRGBf(1, 0, 0, m_numLeds, corner+1, corner+forceRounded);
        }
        break;

        // TODO CASES
    case led_corner_front_left:
        ROS_INFO("CORNER_FRONT_LEFT");
        m_led->setRangeRGBf(0, 0, 1, m_numLeds, corner, corner+forceRounded);
        ROS_INFO("Corner: front left | from %d to %d", corner, corner+forceRounded);
        m_led->setRangeRGBf(1, 0, 0, m_numLeds, m_numLeds-forceRounded, m_numLeds-1);
        ROS_INFO("Corner: front left | from %d to %d", m_numLeds-forceRounded, m_numLeds-1);
        break;
    case led_corner_back_left:
        ROS_INFO("CORNER_BACK_LEFT");
        m_led->setRangeRGBf(1, 0, 0, m_numLeds, corner-forceRounded, corner);
        ROS_INFO("Corner: back left | from %d to %d", corner-forceRounded, corner);
        m_led->setRangeRGBf(0, 0, 1, m_numLeds, corner, corner+forceRounded);
        ROS_INFO("Corner: back left | from %d to %d", corner, corner+forceRounded);
        break;
    case led_corner_back_right:
        ROS_INFO("CORNER_BACK_RIGHT");
        m_led->setRangeRGBf(1, 0, 0, m_numLeds, corner-forceRounded, corner);
        ROS_INFO("Corner: back right | from %d to %d", corner-forceRounded, corner);
        m_led->setRangeRGBf(0, 0, 1, m_numLeds, corner, corner+forceRounded);
        ROS_INFO("Corner: back right | from %d to %d", corner, corner+forceRounded);
        break;
    }*/

    // Test manually
    /*m_led->setRangeRGBf(0, 0, 1, m_numLeds, 0, 3);
    ROS_INFO("Corner: front left | from %d to %d", 0, 3);
    m_led->setRangeRGBf(1, 0, 0, m_numLeds, 420, 422);
    ROS_INFO("Corner: front left | from %d to %d", 420, 422);*/
}

void IIROB_LED_Cross::policeCallback(const iirob_led::PoliceGoal::ConstPtr& goal) {
    iirob_led::PoliceFeedback feedback;
    iirob_led::PoliceResult result;
    int blinks_left = goal->blinks;
    int fast_blinks_left = goal->fast_blinks;
    int start_led = goal->start_led;
    int end_led = goal->end_led;
    int num_inner_leds = goal->num_inner_leds;

    //if(start_led < 0) start_led = 0;
    //if(end_led >= m_numLeds) end_led = m_numLeds;
    checkLimits(&start_led, &end_led);

    int totLength = (end_led - start_led);
    ROS_INFO("SETUP COMPLETED");

    // Probably this here needs some rework
    // TODO Check how this works when the reversed (end_led > start_led) is passed onto this callback. For now force start_led < end_led
    if(start_led > end_led) { int temp = start_led; start_led = end_led; end_led = temp; }

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
