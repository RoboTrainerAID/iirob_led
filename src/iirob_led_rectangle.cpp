//#include <cmath>
#include <math.h>
#include "iirob_led_rectangle.h"
#include "iirob_led_base.h"
#include "RGBConverter.h"

IIROB_LED_Rectangle::IIROB_LED_Rectangle(ros::NodeHandle nodeHandle, std::string const& _port, int const& _m_numLeds)
    : policeAS(nodeHandle, "police", boost::bind(&IIROB_LED_Rectangle::policeCallback, this, _1), false),
      IIROB_LED_Base::IIROB_LED_Base(nodeHandle, _port, _m_numLeds)
{
    policeAS.start();
    ROS_INFO("police action server started");
    subForce = nodeHandle.subscribe("led_force_rectangle", 10, &IIROB_LED_Rectangle::forceCallback, this);
    ROS_INFO("led_force_rectangle subscriber started");
}

IIROB_LED_Rectangle::~IIROB_LED_Rectangle() {
    ROS_INFO("Shutting down police action server and led_force_rectangle subscriber");
    policeAS.shutdown();
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
    /*if(x > 0  && y == 0) coordinates = LEFT;
    else if(x < 0 && y == 0) coordinates = RIGHT;
    else if(x == 0 && y > 0) coordinates = FRONT;
    else if(x == 0 && y < 0) coordinates = BACK;
    // Cases where both axes are unequal zero
    else if(x > 0 && y > 0) coordinates = QUADRANT_FIRST;
    else if(x < 0 && y > 0) coordinates = QUADRANT_SECOND;
    else if(x < 0 && y < 0) coordinates = QUADRANT_THIRD;
    else if(x > 0 && y < 0) coordinates = QUADRANT_FOURTH;
    else coordinates = QUADRANT_NONE;*/

    if(x == 0  && y < 0) coordinates = RIGHT;
    else if(x == 0 && y > 0) coordinates = LEFT;
    else if(y == 0 && x > 0) coordinates = FRONT;
    else if(y == 0 && x < 0) coordinates = BACK;
    // Cases where both axes are unequal zero
    /*else if(x > 0 && y < 0) coordinates = QUADRANT_FIRST;
    else if(x > 0 && y > 0) coordinates = QUADRANT_SECOND;
    else if(x < 0 && y > 0) coordinates = QUADRANT_THIRD;
    else if(x < 0 && y < 0) coordinates = QUADRANT_FOURTH;
    else coordinates = QUADRANT_NONE;*/

    ROS_INFO("XY coordinates: [%.3f , %.3f] (direction macro: %d)\t|\tForce (upper limit of %d): %.3f", x, y, coordinates, MAX_FORCE, force);

    /*
     * We combine the LED indexing (front right corner: LED[0] ... back right corner: LED[0+108] ... ... ... front right corner: LED[383])
     * and the quadrants:
     *
     *
     */
    //int startingPointOnStrip = 0;
    double ledPerDeg = 0;
    double translationAlongStrip = 0;
    double angle = 0.;  // Angle in radians

    /*
     * If the vector does not lie on one of the axes we need to determine the angle it has relative to the coordinate system that we have defined
     *
     *               FRONT
     *   [0],[383]    +y        ______SR2
     *         \  _____|_____  / __________unit circle
     *          \/     |     \/ /
     *          /|-----|-----|\/
     *         / |     |     | \
     *         | |     |     | |
     *LEFT -x__|_|_____0_____|_|__+x RIGHT
     *         | |     |     | |
     *         | |     |     | |
     *         \ |     |     | /
     *          \|-----|-----|/
     *           \_____|_____/
     *                 |
     *                -y
     *               BACK
     *
     */

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

        // TODO Change coordinate system: new x+ is old y+ and new y+ is old x-

    case QUADRANT_FIRST:
        angle = atan2(y, x)*180/M_PI;
        ROS_INFO("Vector angle relative to defined coordinate system: %.3fdeg", angle);
        location = RECT_CORNER_FRONT_RIGHT;
        ROS_INFO("Front right");
        break;
    case QUADRANT_SECOND:
        //startingPointOnStrip = RECT_RIGHT+1;
        //location = starting_point_on_strip+(int)x;
        angle = atan2(y, x)*180/M_PI;
        ROS_INFO("Vector angle relative to defined coordinate system: %.3fdeg", angle);
        location = RECT_CORNER_FRONT_LEFT;
        ROS_INFO("Front left");
        break;
    case QUADRANT_THIRD:
        angle = atan2(y, x)*180/M_PI;
        ROS_INFO("Vector angle relative to defined coordinate system: %.3fdeg", angle);
        location = RECT_CORNER_BACK_LEFT;
        ROS_INFO("Back left");
        break;
    case QUADRANT_FOURTH:
        angle = atan2(y, x)*180/M_PI;
        ROS_INFO("Vector angle relative to defined coordinate system: %.3fdeg", angle);
        location = RECT_CORNER_BACK_RIGHT;
        ROS_INFO("Back right");
        break;
    }

    angle = atan2(y,x); // Note that at this point we have already dismissed the undefined case where x == y == 0 (QUADRANT_NONE)
    ledPerDeg = m_numLeds/360.;
    translationAlongStrip = (angle*180./M_PI) * ledPerDeg;
    location = (int)round(RECT_RIGHT + translationAlongStrip) % m_numLeds;
    ROS_INFO("Led/Angle: %f | Angle: %frad (=%fdeg) | Translation: %f | location: %d", ledPerDeg, angle, angle*180./M_PI, translationAlongStrip, location);

    m_led->setAllRGBf(0, 0, 0, m_numLeds);

    if(forceRounded == 1)
    {
        m_led->setRangeRGBf(0, 0, 1, m_numLeds, location, location);
        return;
    }

    forceRounded--;
    if(location == RECT_CORNER_FRONT_LEFT) {
        m_led->setRangeRGBf(1, 0, 0, m_numLeds, (m_numLeds-1)-forceRounded, m_numLeds-1);
        m_led->setRangeRGBf(1, 0, 0, m_numLeds, 0, forceRounded-1);
    }
    else m_led->setRangeRGBf(1, 0, 0, m_numLeds, location-forceRounded, location+forceRounded);
    m_led->setRangeRGBf(0, 0, 1, m_numLeds, location, location);
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
            m_led->setRangeRGBf(goal->color_outer.r, goal->color_outer.g, goal->color_outer.b, m_numLeds, start_led_left_outer, end_led_left_outer, true, true, false);
            m_led->setRangeRGBf(goal->color_outer.r, goal->color_outer.g, goal->color_outer.b, m_numLeds, start_led_right_outer, end_led_right_outer);
            ros::Duration(goal->fast_duration_on).sleep();

            m_led->setRangeRGBf(0, 0, 0, m_numLeds, start_led_left_outer, end_led_left_outer, true, true, false);
            m_led->setRangeRGBf(0, 0, 0, m_numLeds, start_led_right_outer, end_led_right_outer);
            ros::Duration(goal->fast_duration_off).sleep();

            // Blink the inner subsections
            m_led->setRangeRGBf(goal->color_inner.r, goal->color_inner.g, goal->color_inner.b, m_numLeds, start_led_left_inner, end_led_left_inner, true, true, false);
            m_led->setRangeRGBf(goal->color_inner.r, goal->color_inner.g, goal->color_inner.b, m_numLeds, start_led_right_inner, end_led_right_inner);
            ros::Duration(goal->fast_duration_on).sleep();

            m_led->setRangeRGBf(0, 0, 0, m_numLeds, start_led_left_inner, end_led_left_inner, true, true, false);
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
