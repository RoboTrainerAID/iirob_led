#include <cmath>
#include "iirob_led_rectangle.h"
#include "iirob_led_base.h"
#include "RGBConverter.h"

IIROB_LED_Rectangle::IIROB_LED_Rectangle(ros::NodeHandle nodeHandle, std::string const& _port, int const& _m_numLeds)
    : IIROB_LED_Base::IIROB_LED_Base(nodeHandle, _port, _m_numLeds)
{
    ROS_INFO("led_force_rectangle subscriber started");
    subForce = nodeHandle.subscribe("led_force_rectangle", 10, &IIROB_LED_Rectangle::forceCallback, this);
}

IIROB_LED_Rectangle::~IIROB_LED_Rectangle() {
    ROS_INFO("Shutting down led_force subscriber");
    subForce.shutdown();
}

// Callbacks for all action servers and subscribers
void IIROB_LED_Rectangle::forceCallback(const iirob_led::DirectionWithForce::ConstPtr& led_force_msg) {
    // Calculate the force
    double x = led_force_msg->force.x;
    double y = led_force_msg->force.y;
    double force = sqrt(pow(led_force_msg->force.x, 2) + pow(led_force_msg->force.y, 2) + pow(led_force_msg->force.z, 2));
    // Scale the received force to be in the interval between 0 and maxForce (maxFroce rounded up and converted to an integer - it will represent the number of LEDs to be lit)
    // [0] ----- [force] -- [maxForce]
    //int forceRounded = (int)ceil(force);      // 1.5 becomes 2.0 and then 2
    int forceRounded = (int)round(force);       // 1.5 becomes 1
    ROS_INFO("Max force: %d", MAX_FORCE);
    ROS_INFO("Force: %.3f | Rounded and int: %d", force, forceRounded);
    if(forceRounded > MAX_FORCE) {
        ROS_ERROR("Received force is of greater magnitude than the set upper bound or exceed the numer of LEDs that can be displayed on each side of the platform | forceRounded(%d), forceMax(%d)", forceRounded, MAX_FORCE);
        return;
    }

    // Determine quadrant and corner that will display the force
    int8_t coordinates;
    int location;

    // Cases where one of both axes equals zero
    if(x == 1 && y == 0) coordinates = LEFT;
    else if(x == -1 && y == 0) coordinates = RIGHT;
    else if(x == 0 && y == 1) coordinates = FRONT;
    else if(x == 0 && y == -1) coordinates = BACK;
    // Cases where both axes are unequal zero
    else if(x > 0 && y > 0) coordinates = QUADRANT_FIRST;
    else if(x < 0 && y > 0) coordinates = QUADRANT_SECOND;
    else if(x < 0 && y < 0) coordinates = QUADRANT_THIRD;
    else if(x > 0 && y < 0) coordinates = QUADRANT_FOURTH;
    else coordinates = QUADRANT_NONE;

    ROS_INFO("XY coordinates: [%.3f , %.3f] (quadrant macro %d)\t|\tForce (upper limit of %d): %.3f", x, y, coordinates, MAX_FORCE, force);

    //
    switch(coordinates) {
    case QUADRANT_NONE:
        // If X and Y are equal to 0 we cannot determine the corner hence no point in displaying anything
        return;
    case FRONT:
        location = RECT_FRONT;
        break;
    case BACK:
        location = RECT_BACK;
        break;
    case LEFT:
        location = RECT_LEFT;
        break;
    case RIGHT:
        location = RECT_RIGHT;
        break;
    case QUADRANT_FIRST:
        location = RECT_CORNER_FRONT_LEFT;
        ROS_INFO("Front left");
        break;
    case QUADRANT_SECOND:
        location = RECT_CORNER_FRONT_RIGHT;
        ROS_INFO("Front right");
        break;
    case QUADRANT_THIRD:
        location = RECT_CORNER_BACK_LEFT;
        ROS_INFO("Back left");
        break;
    case QUADRANT_FOURTH:
        location = RECT_CORNER_BACK_RIGHT;
        ROS_INFO("Back right");
        break;
    }

    m_led->setAllRGBf(0, 0, 0, m_numLeds);
    int LED = 0;

    switch (location) {
    // X or Y == 0
    case RECT_FRONT:
        ROS_INFO("Front");
        m_led->setRangeRGBf(0, 0, 1, m_numLeds, location-2, location+1);
        break;
    case RECT_BACK:
        ROS_INFO("Back");
        m_led->setRangeRGBf(0, 0, 1, m_numLeds, location-2, location+1);
        break;
    case RECT_LEFT:
        ROS_INFO("Left");
        m_led->setRangeRGBf(0, 0, 1, m_numLeds, location-2, location+1);
        break;
    case RECT_RIGHT:
        ROS_INFO("Right");
        m_led->setRangeRGBf(0, 0, 1, m_numLeds, location-2, location+1);
        break;

    // X and Y != 0 (quadrants)
    case RECT_CORNER_FRONT_LEFT:
        if(forceRounded == 1)
        {
            m_led->setRangeRGBf(1, 0, 0, m_numLeds, location, location);
            break;
        }

        forceRounded--;

        m_led->setRangeRGBf(1, 0, 0, m_numLeds, location-forceRounded, location+forceRounded);
        break;
    case RECT_CORNER_FRONT_RIGHT:
        if(forceRounded == 1)
        {
            m_led->setRangeRGBf(1, 0, 0, m_numLeds, location, location);
            break;
        }

        forceRounded--;
        m_led->setRangeRGBf(1, 0, 0, m_numLeds, (m_numLeds-1)-forceRounded, m_numLeds-1);
        m_led->setRangeRGBf(1, 0, 0, m_numLeds, 0, forceRounded-1);
        break;
    case RECT_CORNER_BACK_LEFT:
        if(forceRounded == 1)
        {
            m_led->setRangeRGBf(1, 0, 0, m_numLeds, location, location);
            break;
        }

        forceRounded--;

        m_led->setRangeRGBf(1, 0, 0, m_numLeds, location-forceRounded, location+forceRounded);
        break;
    case RECT_CORNER_BACK_RIGHT:
        if(forceRounded == 1)
        {
            m_led->setRangeRGBf(1, 0, 0, m_numLeds, location, location);
            break;
        }

        forceRounded--;

        m_led->setRangeRGBf(1, 0, 0, m_numLeds, location-forceRounded, location+forceRounded);
        break;
    }

    if(location == RECT_RIGHT || location == RECT_LEFT || location == RECT_FRONT || location == RECT_BACK) m_led->setRangeRGBf(0, 0, 1, m_numLeds, location-2, location+1);
    else {
        if(forceRounded == 1)
        {
            m_led->setRangeRGBf(1, 0, 0, m_numLeds, location, location);
            break;
        }
        forceRounded--;

        if(location == RECT_CORNER_FRONT_RIGHT) {
            m_led->setRangeRGBf(1, 0, 0, m_numLeds, (m_numLeds-1)-forceRounded, m_numLeds-1);
            m_led->setRangeRGBf(1, 0, 0, m_numLeds, 0, forceRounded-1);
        }
    }
}

void IIROB_LED_Rectangle::policeCallback(const iirob_led::PoliceGoal::ConstPtr& goal) {
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
