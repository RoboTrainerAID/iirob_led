#include <cmath>
#include "iirob_led_rectangle.h"
#include "iirob_led_base.h"
#include "RGBConverter.h"

IIROB_LED_Rectangle::IIROB_LED_Rectangle(ros::NodeHandle nodeHandle, std::string const& _port, int const& _m_numLeds)
    : /*long_side(108), short_side(84),
      led_start(0), led_end(2*long_side + 2*short_side),
      led_corner_front_right(led_end-1),
      led_corner_front_left(long_side),
      led_corner_back_right(short_side+long_side-1),
      led_corner_back_left(short_side+2*long_side),*/
      IIROB_LED_Base::IIROB_LED_Base(nodeHandle, _port, _m_numLeds)
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
    }

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
    int rangeBeforeCorner;
    int rangeAfterCorner;

    // TODO See how to store the previous state so that when processing a newly received message the previously lit LEDs are turned off
    // Temporary use following
    m_led->setAllRGBf(0, 0, 0, m_numLeds);
    /*m_led->setRangeRGBf(1, 0, 0, m_numLeds, led_corner_front_right, led_corner_front_right);
    m_led->setRangeRGBf(0, 1, 0, m_numLeds, led_corner_front_left, led_corner_front_left);
    m_led->setRangeRGBf(0, 0, 1, m_numLeds, led_corner_back_left, led_corner_back_left);
    m_led->setRangeRGBf(1, 1, 0, m_numLeds, led_corner_back_right, led_corner_back_right);*/

    /*
    m_led->setRangeRGBf(0, 0, 0, m_numLeds, 383, 383);
    m_led->setRangeRGBf(0, 0, 0, m_numLeds, 108, 108);
    m_led->setRangeRGBf(0, 0, 0, m_numLeds, 84+108-1, 84+108-1);
    m_led->setRangeRGBf(0, 0, 0, m_numLeds, 84+2*108, 84+2*108);

     */

    switch(corner) {
        case led_corner_front_right:
            ROS_INFO("CORNER_FRONT_RIGHT");
            if(forceRounded == 1) m_led->setRangeRGBf(1, 0, 0, m_numLeds, corner, corner);
            else {
                // Reduce the forceRound by one since the corner already displays a single unit
                forceRounded -= 1;
                rangeBeforeCorner = corner - forceRounded;
                rangeAfterCorner = 0 + forceRounded-1;
                m_led->setRangeRGBf(0, 1, 0, m_numLeds, rangeBeforeCorner, corner);   // From N to corner-1
                m_led->setRangeRGBf(1, 0, 0, m_numLeds, corner, corner);                // corner to corner
                m_led->setRangeRGBf(0, 0, 1, m_numLeds, 0, rangeAfterCorner);           // From 0 to N  // Replace the 0 with (corner+1)%m_numleds
                ROS_INFO("[%d : %d)", rangeBeforeCorner, corner);
                ROS_INFO("[%d : %d)", 0, rangeAfterCorner);
            }
            break;

        case led_corner_front_left:
            ROS_INFO("CORNER_FRONT_LEFT");
            if(forceRounded == 1) m_led->setRangeRGBf(0, 1, 0, m_numLeds, corner, corner);
            else {
                // Reduce the forceRound by one since the corner already displays a single unit
                rangeBeforeCorner = corner - forceRounded;
                rangeAfterCorner = (corner + forceRounded) % m_numLeds;
                m_led->setRangeRGBf(1, 0, 0, m_numLeds, rangeBeforeCorner, rangeAfterCorner); // From corner-N through corner to corner+N

                /*m_led->setRangeRGBf(0, 0, 1, m_numLeds, rangeBeforeCorner, corner-1);   // From N to corner-1
                m_led->setRangeRGBf(1, 0, 0, m_numLeds, corner, corner);                // corner to corner
                m_led->setRangeRGBf(0, 0, 1, m_numLeds, corner+1, rangeAfterCorner);           // From 0 to N*/
                ROS_INFO("[%d : %d)", rangeBeforeCorner, corner);
                ROS_INFO("[%d : %d)", corner, rangeAfterCorner);
            }
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
        }

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
