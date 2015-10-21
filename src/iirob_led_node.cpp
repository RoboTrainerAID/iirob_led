#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>

#include <iirob_led/BlinkyAction.h>
#include <iirob_led/PoliceAction.h>
#include <iirob_led/FourRegionsAction.h>
#include <iirob_led/ChaserLightAction.h>
#include <iirob_led/SetLedDirectory.h>
#include <iirob_led/DirectionWithForce.h>
#include <RGBConverter.h>

#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Vector3.h>

#include <cmath>

#include "LEDStrip.h"

#define QUADRANT_NONE   0   // x = y = 0
#define QUADRANT_FIRST  1   // 1st quadrant: +x, +y
#define QUADRANT_SECOND 2   // 2nd quadrant: -x, +y
#define QUADRANT_THIRD  3   // 3rd quadrant: -x, -y
#define QUADRANT_FOURTH 4   // 4th quadrant: +x, -y

// FIXME #LEDs - 384 (last LED not lit)
// Ruben -> take his caluclations for the LEDs
#define CORNER_FRONT_LEFT   0
#define CORNER_BACK_LEFT    108
#define CORNER_BACK_RIGHT   192
#define CORNER_FRONT_RIGHT  300

const int long_side = 108;
const int short_side = 84;

const int led_start = 0;
const int led_end = 2*long_side + 2*short_side;

const int led_corner_front_right = short_side;
const int led_corner_back_right = short_side + long_side;
const int led_corner_back_left = short_side + 2*long_side;
const int led_corner_front_left = led_end;

// TODO override - when ready set to false
// TODO directoryCallback, set_led_directory - replace this with a vector (btw it's DIRECTION not directory -_-)
// TODO Check return value for all iirob::hardware functions (setAllRGBf(), setRangeRGBf() etc.)
// TODO Add a duration parameter (number of steps per cycle should be determined from it) to Changeling (just add two modes)
// TODO All the checks if start_led or end_led are negative or not can be avoided by simply using unsigned chars (as far as I can tell internally the LED library is actually using this type)
// TODO Reimplement setRangeRGBX, setRangeRGB and setRangeRGBf. The way those three are working now is - to put it mildly - in a very poor manner.
// NOTE Except for color values (ColorRGBA requires float) use double (float64 in ROS) for more accuracy

class LEDNode
{
    /*
     * SR2 platform corners and quadrants:
     *   (3rd quadrant)         (2nd quadrant)
     * CORNER_BACK_LEFT ----- CORNER_FRONT_LEFT
     *         |                       |
     *         |           x           |_____________[key switch]
     *         |    (none quadrant)    |
     *         |                       |
     * CORNER_BACK_RIGHT ---- CORNER_FRONT_RIGHT
     *   (4th quadrant)         (1st quadrant)
     */
    /*typedef const int8_t QUADRANT;
    QUADRANT QUADRANT_NONE;     // x = y = 0
    QUADRANT QUADRANT_FIRST;    // 1st quadrant: +x, +y
    QUADRANT QUADRANT_SECOND;   // 2nd quadrant: -x, +y
    QUADRANT QUADRANT_THIRD;    // 3rd quadrant: -x, -y
    QUADRANT QUADRANT_FOURTH;   // 4th quadrant: +x, -y*/

    const int MAX_FORCE;        // defines the maximum number of LEDs that can be lit (currently set to 10). If changed don't forget to redefine the corners to fit the new bounds


    int m_msec;
    int m_numLeds;
    iirob_hardware::LEDStrip* m_led;
    int m_showDir;
    bool status;                // Contains the return result from init()
    std::string port;
    int num;

    ros::Subscriber subForce;   //Takes x and y coordinates, calculates the force (length of the vector) and uses the LEDs to show the direction and magnitude of that force on the XY plane of the SR2

    // Action servers
    actionlib::SimpleActionServer<iirob_led::BlinkyAction> blinkyAS;
    actionlib::SimpleActionServer<iirob_led::PoliceAction> policeAS;
    actionlib::SimpleActionServer<iirob_led::FourRegionsAction> fourRegionsAS;
    actionlib::SimpleActionServer<iirob_led::ChaserLightAction> chaserLightAS;

    /**
     * @brief init Initializes the hardware and starts the action servers
     * @param port Port as string
     * @param num Number of LEDs
     * @return
     */
    bool init(std::string const& port, int const& num) {
        m_led = new iirob_hardware::LEDStrip(port);
        if (m_led->ready()) {
            //m_numLeds = num;
            m_led->setAllRGBf(0, 0, 0, m_numLeds);
            m_led->setUniRGB(0, 0, 0);

            ROS_INFO("%s: IIROB-LED action server started", ros::this_node::getName().c_str());
            ROS_INFO("Number of LEDs: %d", m_numLeds);

            blinkyAS.start();
            ROS_INFO("blinky");
            policeAS.start();
            ROS_INFO("police");
            fourRegionsAS.start();
            ROS_INFO("four_regions");
            chaserLightAS.start();
            ROS_INFO("chaser_light");
            return true;
        }
        return false;
    }

public:
    /**
     * @brief LEDNode Constructor
     * @param nodeHandle
     * @param _port Port as string
     * @param num_of_leds Number of LEDs
     */
    LEDNode(ros::NodeHandle nodeHandle, std::string const& _port, int const& num_of_leds)
        : m_led(0), m_msec(1),
          port(_port), m_numLeds(num_of_leds),
          blinkyAS(nodeHandle, "blinky", boost::bind(&LEDNode::blinkyCallback, this, _1), false),
          policeAS(nodeHandle, "police", boost::bind(&LEDNode::policeCallback, this, _1), false),
          fourRegionsAS(nodeHandle, "four_regions", boost::bind(&LEDNode::fourRegionsCallback, this, _1), false),
          chaserLightAS(nodeHandle, "chaser_light", boost::bind(&LEDNode::chaserLightCallback, this, _1), false),
          MAX_FORCE(10)
    {
        status = init(port, m_numLeds);

        if(!status) {
            ROS_ERROR("Initiating port %s with number %d failed!", port.c_str(), m_numLeds);
            return;
        }

        subForce = nodeHandle.subscribe("led_force", 10, &LEDNode::forceCallback, this);
    }

    /**
     * @brief Destructor turns off all LEDs and shuts down all action servers
     */
	~LEDNode(){
        ROS_INFO("Turning all LEDs off");
        // Turn everything off again
        if (m_led) {
			m_led->setAllRGBf(0, 0, 0, m_numLeds);
			m_led->setUniRGB(0, 0, 0);
			delete m_led;
		}

        ROS_INFO("Shutting down all action servers");
        blinkyAS.shutdown();
        policeAS.shutdown();
        fourRegionsAS.shutdown();
        chaserLightAS.shutdown();

        subForce.shutdown();
    }

    // Returns whether init() was successful or not
    bool getStatus() { return status; }

    /**
     * @brief forceCallback
     * @param led_force_msg
     */
    // TODO Use Ruben's variables; also this can be completed AFTER a single LED can be lit up; change in display (force = 1: only corner LED is on, force = 2: corner plus LED on each side is on etc.)
    void forceCallback(const iirob_led::DirectionWithForce::ConstPtr& led_force_msg) {
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
            corner = CORNER_FRONT_RIGHT;
            break;
        case QUADRANT_SECOND:
            corner = CORNER_FRONT_LEFT;
            break;
        case QUADRANT_THIRD:
            corner = CORNER_BACK_LEFT;
            break;
        case QUADRANT_FOURTH:
            corner = CORNER_BACK_RIGHT;
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

        // TODO Remove all ROS_INFOs here
        switch(corner) {
        case CORNER_FRONT_RIGHT:
            // Example: forceRounded = 5, CORNER_FRONT_RIGHT = 300
            // From 300 - 5 = 255 to 299 [LEDs: 255, 296, 297, 298, 299]
            // From 300 to 300 + 4 = 304 [LEDs: 300, 301, 302, 303, 304]
            m_led->setRangeRGBf(1, 0, 0, m_numLeds, corner-forceRounded, corner);
            ROS_INFO("Corner: front right | from %d to %d", corner-forceRounded, corner);
            m_led->setRangeRGBf(0, 0, 1, m_numLeds, corner, corner+forceRounded);
            ROS_INFO("Corner: front right | from %d to %d", corner, corner+forceRounded);
            break;
        case CORNER_FRONT_LEFT:
            // FIXME Same issue as with the RunningBunny - going from 422 to 0 causes only the 0..x to be displayed
            m_led->setRangeRGBf(0, 0, 1, m_numLeds, corner, corner+forceRounded);
            ROS_INFO("Corner: front left | from %d to %d", corner, corner+forceRounded);
            m_led->setRangeRGBf(1, 0, 0, m_numLeds, m_numLeds-forceRounded, m_numLeds-1);
            ROS_INFO("Corner: front left | from %d to %d", m_numLeds-forceRounded, m_numLeds-1);
            break;
        case CORNER_BACK_LEFT:
            m_led->setRangeRGBf(1, 0, 0, m_numLeds, corner-forceRounded, corner);
            ROS_INFO("Corner: back left | from %d to %d", corner-forceRounded, corner);
            m_led->setRangeRGBf(0, 0, 1, m_numLeds, corner, corner+forceRounded);
            ROS_INFO("Corner: back left | from %d to %d", corner, corner+forceRounded);
            break;
        case CORNER_BACK_RIGHT:
            m_led->setRangeRGBf(1, 0, 0, m_numLeds, corner-forceRounded, corner);
            ROS_INFO("Corner: back right | from %d to %d", corner-forceRounded, corner);
            m_led->setRangeRGBf(0, 0, 1, m_numLeds, corner, corner+forceRounded);
            ROS_INFO("Corner: back right | from %d to %d", corner, corner+forceRounded);
            break;
        }

        // Test manually
        /*m_led->setRangeRGBf(0, 0, 1, m_numLeds, 0, 3);
        ROS_INFO("Corner: front left | from %d to %d", 0, 3);
        m_led->setRangeRGBf(1, 0, 0, m_numLeds, 420, 422);
        ROS_INFO("Corner: front left | from %d to %d", 420, 422);*/

        ros::Duration(led_force_msg->duration).sleep();
        m_led->setAllRGBf(0, 0, 0, m_numLeds);
    }

    /**
     * @brief blinkyCallback processes BlinkyActionGoal messages - it turns on and off a give stripe of LEDs; previous name: lightActionCallback_2 with argument std_msgs::Float32MultiArray::ConstPtr&
     * @param goal
     */
    void blinkyCallback(const iirob_led::BlinkyGoal::ConstPtr& goal) {
        iirob_led::BlinkyFeedback feedback;
        iirob_led::BlinkyResult result;
        int blinks_left = goal->blinks;
        int start_led = goal->start_led;
        int end_led = goal->end_led;
        int num_leds = goal->num_leds;
        bool fade_in = goal->fade_in;
        bool fade_out = goal->fade_out;

        ROS_WARN("Due to latency issues if duration_on is set low enough the resulting blinking will take longer. This applies to a greater extent if fade_in and/or fade_out are enabled. When disabled the reposnse is much more accurate.");

        // Check if override of end_led is possible
        if(num_leds != 0) { // num_leds always takes precendence over end_led whenever it is != 0
            int temp_end_led = start_led + num_leds;    // Use the num_leds as offset from start_led to calculate the new end_led
            end_led = temp_end_led;
            ROS_WARN("num_leds contains a on-zero value and will override end_led");
        }

        if(start_led < 0) start_led = 0;
        if(end_led >= m_numLeds) end_led = m_numLeds-1;

        ROS_DEBUG("%s starting Blinky action with color [%f %f %f] [RGB]. Duration(ON): %f sec, Duration(OFF): %f, start led: %d, end led: %d",
                 ros::this_node::getName().c_str(),
                 goal->color.r, goal->color.g, goal->color.b, goal->duration_on, goal->duration_off, start_led, end_led);

        float r = goal->color.r;
        float g = goal->color.g;
        float b = goal->color.b;
        float h, s, v, v_old;
        // Initial conversion
        RGBConverter::rgbToHsv(r, g, b, &h, &s, &v);
        ROS_INFO("Target full color (RGB): %.2f %.2f %.2f", r, g, b);
        ROS_INFO("Target full color (HSV): %.2f %.2f %.2f", h, s, v);
        v_old = v;

        // If fade_in/fade_out is enable we assign a fixed portion of duration_on to it
        double fade_in_duration = 0, fade_out_duration = 0;
        if(fade_in) fade_in_duration = goal->duration_on/4;
        if(fade_out) fade_out_duration = goal->duration_on/4;
        double new_duration = goal->duration_on - (fade_in_duration + fade_out_duration);   // the duration when the LEDs will be light up at their full capacity is recalculated based on whether fade_in and/or fade_out has been turned on
        ROS_INFO("duration_on (%f) will be split into {fade_in: %f | fade_out: %f | full: %f}", goal->duration_on, fade_in_duration, fade_out_duration, new_duration);

        /*double single_step = 0.01;
        double single_step_duration = (goal->duration_on/4)*single_step;
        int steps_per_fade_cycle = (goal->duration_on/4)/single_step_duration;*/

        // It is advisable to use a single step of 0.01 for the V (in HSV) for low durations. For longer consider also adding a connection to the duration so that the granularity can be increased resulting in some eye candy
        const int steps_per_fade_cycle = 100; // hardcoded number of steps per fade cycle; using this we can derive the value of a single step
        double single_step = v / steps_per_fade_cycle;  // this determines the HSV value increment/decrement and is based on the steps per fade cycle
        const double single_step_duration = (goal->duration_on/4) / steps_per_fade_cycle; // the duration of a single step is derived from the duration of the fade cycle devided by the steps per cycle
        ROS_INFO("Single HSV step: %.5f | Single HSV step duration: %.10f | Steps per fade cycle: %d", single_step, single_step_duration, steps_per_fade_cycle);

        for(int i = 0; i < goal->blinks; ++i, --blinks_left)
        {
            v = 0.0;    // Reset value (HSV) since this is the component that we will be working with
            // Fade in phase (if enabled)
            if(fade_in) {
                ROS_INFO("Fade in");
                for(int current_step = 0; current_step < steps_per_fade_cycle; v += single_step, ++current_step) {
                    RGBConverter::hsvToRgb(h, s, v, &r, &g, &b);
                    m_led->setRangeRGBf(r, g, b, m_numLeds, start_led, end_led);
                    ros::Duration(single_step_duration).sleep();
                    ROS_INFO("[%d] HSV: %.2f %.2f %.2f", current_step, h, s, v);
                }
            }

            // Full light phase
            ROS_INFO("Full");
            m_led->setRangeRGBf(goal->color.r, goal->color.g, goal->color.b, m_numLeds, start_led, end_led);
            ros::Duration(new_duration).sleep();

            v = v_old;
            // Fade out phase (if enabled)
            if(fade_out) {
                ROS_INFO("Fade out");
                for(int current_step = steps_per_fade_cycle-1; current_step >= 0; v -= single_step, --current_step) {
                    RGBConverter::hsvToRgb(h, s, v, &r, &g, &b);
                    m_led->setRangeRGBf(r, g, b, m_numLeds, start_led, end_led);
                    ros::Duration(single_step_duration).sleep();
                    ROS_INFO("[%d] HSV: %.2f %.2f %.2f", current_step, h, s, v);
                }
            }

            // Turn off LEDs
            m_led->setRangeRGBf(0, 0, 0, m_numLeds, goal->start_led, goal->end_led);
            ros::Duration(goal->duration_off).sleep();

            feedback.blinks_left = blinks_left;
            ROS_INFO("%s: Blinky will blink %d more times", ros::this_node::getName().c_str(), feedback.blinks_left);
            blinkyAS.publishFeedback(feedback);
        }

        feedback.blinks_left = blinks_left;
        ROS_INFO("%s: Blinky will blink %d more times", ros::this_node::getName().c_str(), feedback.blinks_left);

        m_led->setAllRGBf(0, 0, 0, m_numLeds);
        result.blinks_left = blinks_left;
        blinkyAS.setSucceeded(result);
    }

    /**
     * @brief policeCallback processes PoliceActionGoal messages - it turns on and off a give stripe of LEDs mimicing a police light; previous name: lightActionCallback_3 with argument std_msgs::Float32MultiArray::ConstPtr&
     * @param goal
     */
    void policeCallback(const iirob_led::PoliceGoal::ConstPtr& goal) {
        iirob_led::PoliceFeedback feedback;
        iirob_led::PoliceResult result;
        int blinks_left = goal->blinks;
        int fast_blinks_left = goal->fast_blinks;
        int start_led = goal->start_led;
        int end_led = goal->end_led;
        int num_inner_leds = goal->num_inner_leds;

        if(start_led < 0) start_led = 0;
        if(end_led >= m_numLeds) end_led = m_numLeds-1;
        int totLength = (end_led - start_led);
        ROS_INFO("SETUP COMPLETED");

        // Probably this here needs some rework
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

        m_led->setAllRGBf(0, 0, 0, m_numLeds);
        result.blinks_left = blinks_left;
        policeAS.setSucceeded(result);
    }

    /**
     * @brief fourMusketeersCallback processes FourMusketeersGoal messages - it turns on and off a give stripe of LEDs which is split into 4 separate simultaniously blinking sections with different colors; previous name: lightActionCallback_4 with argument std_msgs::Float32MultiArray::ConstPtr&
     * @param goal
     */
    void fourRegionsCallback(const iirob_led::FourRegionsGoal::ConstPtr& goal) {
        iirob_led::FourRegionsFeedback feedback;
        iirob_led::FourRegionsResult result;
        int blinks_left = goal->blinks;
        int start_led = goal->start_led;
        int end_led = goal->end_led;

        if(start_led < 0) start_led = 0;
        if(end_led >= m_numLeds) end_led = m_numLeds-1;

        if(end_led - start_led < 3) {
            start_led = 0;
            end_led = m_numLeds-1;
        }

        // Split the given interval of LEDs into 4 more or less equal subsections each with its own coloring
        int half = (end_led - start_led)/2;
        int start_ledRegion1 = start_led;
        int end_ledRegion1 = start_ledRegion1 + half/2;

        int start_ledRegion2 = end_ledRegion1;
        int end_ledRegion2 = start_ledRegion2 + half/2;

        int start_ledRegion3 = end_ledRegion2;
        int end_ledRegion3 = start_ledRegion3 + half/2;

        int start_ledRegion4 = end_ledRegion3;
        int end_ledRegion4 = end_led;

        ROS_INFO("%s starting Four Musketeers action with duration(ON): %.2f sec and duration(OFF): %.2f\n\n\t Region1\t\tstart led: %d, end led: %d\tcolor: %.2f %.2f %.2f \n\t Region2\tstart led: %d, end led: %d\tcolor: %.2f %.2f %.2f \n\t Region3\t\tstart led: %d, end led: %d\tcolor: %.2f %.2f %.2f \n\t Region4\tstart led: %d, end led: %d\tcolor: %.2f %.2f %.2f ",
                 ros::this_node::getName().c_str(),
                 goal->duration_on, goal->duration_off,
                 start_ledRegion1, end_ledRegion1,
                 goal->color1.r, goal->color1.g, goal->color1.b,
                 start_ledRegion2, end_ledRegion2,
                 goal->color2.r, goal->color2.g, goal->color2.b,
                 start_ledRegion3, end_ledRegion3,
                 goal->color3.r, goal->color3.g, goal->color3.b,
                 start_ledRegion4, end_ledRegion4,
                 goal->color4.r, goal->color4.g, goal->color4.b);

        for(int i = 0; i < goal->blinks; ++i, --blinks_left)
        {
            // Set selected LEDs and turn them on
            m_led->setRangeRGBf(goal->color1.r, goal->color1.g, goal->color1.b, m_numLeds, start_ledRegion1, end_ledRegion1);
            m_led->setRangeRGBf(goal->color2.r, goal->color2.g, goal->color2.b, m_numLeds, start_ledRegion2, end_ledRegion2);
            m_led->setRangeRGBf(goal->color3.r, goal->color3.g, goal->color3.b, m_numLeds, start_ledRegion3, end_ledRegion3);
            m_led->setRangeRGBf(goal->color4.r, goal->color4.g, goal->color4.b, m_numLeds, start_ledRegion4, end_ledRegion4);
            ros::Duration(goal->duration_on).sleep();
            // Set selected LEDs and turn them off
            m_led->setRangeRGBf(0, 0, 0, m_numLeds, start_ledRegion1, end_ledRegion1);
            m_led->setRangeRGBf(0, 0, 0, m_numLeds, start_ledRegion2, end_ledRegion2);
            m_led->setRangeRGBf(0, 0, 0, m_numLeds, start_ledRegion3, end_ledRegion3);
            m_led->setRangeRGBf(0, 0, 0, m_numLeds, start_ledRegion4, end_ledRegion4);
            ros::Duration(goal->duration_off).sleep();

            feedback.blinks_left = blinks_left;
            ROS_INFO("%s: Four Musketeers will blink %d more times", ros::this_node::getName().c_str(), feedback.blinks_left);
            fourRegionsAS.publishFeedback(feedback);
        }

        feedback.blinks_left = blinks_left;
        ROS_INFO("%s: Four Musketeers will blink %d more times", ros::this_node::getName().c_str(), feedback.blinks_left);

        m_led->setAllRGBf(0, 0, 0, m_numLeds);
        result.blinks_left = blinks_left;
        fourRegionsAS.setSucceeded(result);
    }

    /**
     * @brief chaserLightCallback processes ChaserLightGoal messages - a given stripe of LEDs traverses the whole strip in circle N times; previous name (maybe? never managed to make it work...): lightActionCallback_6 with argument std_msgs::Float32MultiArray::ConstPtr&
     * @param goal
     */
    void chaserLightCallback(const iirob_led::ChaserLightGoal::ConstPtr& goal) {
        iirob_led::ChaserLightFeedback feedback;
        iirob_led::ChaserLightResult result;

        int head = (goal->start_led < 0) ? 0 : goal->start_led;
        int offset = goal->num_of_leds;
        if(offset <= 0) offset = 2;
        int tail;

        int cycles = goal->num_of_cycles;
        double cycle_duration = goal->cycle_duration;
        // TODO Add the feature "reverse direction"
        bool reverse = goal->reverse;

        double single_step_duration = cycle_duration / m_numLeds; // Number of LEDs that the strip will skip

        // TODO See how to determine the speed properly
        // lowest skip_per_step is 1
        int skip_per_step = m_numLeds / cycle_duration; // Speed based on the number of LEDs per cycle and how long it takes to traverse all
        //int skip_per_step = 1;

        ROS_INFO("head: %d | offset: %d | time per step: %f", head, offset, single_step_duration);

        int old_tail;  // will use this to turn off the trail after the strip has moved
        for(int current_cycle = 0; current_cycle < cycles; current_cycle++) {
            // Prepare for the cycle
            head = (goal->start_led < 0) ? 0 : goal->start_led;
            tail = (head - offset);
            if(tail < 0) tail = tail + m_numLeds;
            old_tail = tail;
            // Movement withing a single cycle
            for(int pos = 0; pos < m_numLeds; pos+=skip_per_step) {
                ROS_INFO("Head: %d | Tail: %d | ", head, tail);
                // Light up the strip
                m_led->setRangeRGBf(goal->color.r, goal->color.g, goal->color.b, m_numLeds, tail, head);
                // TODO Add HSV gradient (head is brightest, tail is dimmest). This will be possible only then when we can control a single LED
                // Wait a little bit
                ros::Duration(single_step_duration).sleep();
                // Update the head and tail position of the strip
                head = (head + skip_per_step) % m_numLeds;
                tail = (head - offset);
                if(tail < 0) tail = tail + m_numLeds;
                ROS_INFO("New tail: %d | old tail: %d", tail, old_tail);
                // Turn off all LEDs between the old end position and the new end position of the strip
                m_led->setRangeRGBf(0, 0, 0, m_numLeds, old_tail, tail);

                old_tail = tail;

                feedback.current_start_pos = head;
                chaserLightAS.publishFeedback(feedback);
            }
        }

        m_led->setAllRGBf(0, 0, 0, m_numLeds);
        feedback.current_start_pos = head;
        chaserLightAS.publishFeedback(feedback);
        result.current_start_pos = head;
        chaserLightAS.setSucceeded(result);
    }

    // OLD VERSION (also uses the old RunningBunny format
//    void chaserLightCallback(const iirob_led::ChaserLightGoal::ConstPtr& goal) {
//        iirob_led::ChaserLightFeedback feedback;
//        iirob_led::ChaserLightResult result;

//        int head = goal->head;
//        int body = goal->body;
//        float r = goal->color.r;
//        float g = goal->color.g;
//        float b = goal->color.b;
//        int counter = 0;
//        int circles = goal->num_circles;
//        bool split = false;
//        int _splitPos = 0;
//        int _after_split = m_numLeds-1;
//        int _head, _tail = head - body;
//        int _tail_non_negative = (_tail < 0) ? _tail + m_numLeds : _tail;   //423-1 = 422, 423-2 = 421, 423-3 = 420 ...
//        int _tail_non_negative_old = _tail_non_negative-1;


//        printf("Starting position: (head[%d] - tail[%d])\n-----------------------------------------\n", head, (head - body) % m_numLeds);

//        for(int circle = 0; circle < circles; circle++) {
//            _head = head;
//            _tail = head - body;
//            //_tail_non_negative_old = _tail_non_negative;

//            for(counter = 0; counter < m_numLeds; counter+=goal->skip_leds_per_step) {
//                //_tail_non_negative_old = _tail_non_negative-1;

//                if(_tail < 0) {
//                    split = true;
//                    _tail_non_negative = _tail + m_numLeds;
//                } else {
//                    split = false;
//                    _tail_non_negative = _tail;
//                }

//                ROS_INFO("[%d] : (head[%d] - tail[%d])\tsplit %s", counter, _head, _tail_non_negative, split ? "+" : "-");

//                if(split) {
//                    ROS_INFO("\t\t\\_________(head[%d] - split[%d] | after_split[%d] - tail[%d])\n", _head, _splitPos, _after_split, _tail_non_negative);
//                    //m_led->setRangeRGBf(r, g, b, m_numLeds, _tail_non_negative, _after_split);   // We finish the current LED stripe (x->422)
//                    //m_led->setRangeRGBf(r, g, b, m_numLeds, _splitPos, _head);                   // Then we start the next (0->y)
//                    ROS_INFO("Turning off LEDs %d to %d", _head+1, _tail_non_negative-1);
//                    //m_led->setRangeRGBf(0, 0, 0, m_numLeds, _head+1, _tail_non_negative-1);      // And finally we turn "off" all (y+1->x-1) (I've noticed that when we overlap LEDs it screws up things)
//                } else {
//                    m_led->setRangeRGBf(r, g, b, m_numLeds, _tail_non_negative, _head);
//                    ROS_INFO("Turning off LEDs %d to %d", _tail_non_negative_old, _tail_non_negative);
//                    m_led->setRangeRGBf(0, 0, 0, m_numLeds, _tail_non_negative_old, _tail_non_negative);
//                }

//                feedback.current_head_pos = _head;
//                chaserLightAS.publishFeedback(feedback);

//                _head = (_head + goal->skip_leds_per_step) % m_numLeds;
//                _tail = _head - body;
//                _tail_non_negative_old = _tail_non_negative-1;
//            }

//            ROS_INFO("circle %d completed", circle);
//        }

//        // Test movement for a single LED ("good" range)
//        /*for(int i = 10; i < 200; i++) {
//            m_led->setRangeRGBf(r, g, b, m_numLeds, i-1, i);
//            m_led->setAllRGBf(0, 0, 0, m_numLeds);
//        }*/

//        m_led->setAllRGBf(0, 0, 0, m_numLeds);
//        feedback.current_head_pos = head;
//        chaserLightAS.publishFeedback(feedback);
//        result.current_head_pos = head;
//        chaserLightAS.setSucceeded(result);
//    }

    /* OLD VERSION
     void runningBunnyCallback(const iirob_led::RunningBunnyGoal::ConstPtr& goal) {
        // TODO Use modulo to create a neat transition between previous and next circle!
        // TODO Change RGB values logarithmic
        ROS_INFO("RUNNING BUNNY CALLBACK");
        iirob_led::RunningBunnyFeedback feedback;
        iirob_led::RunningBunnyResult result;
        int circle_counter = goal->num_circles;
        int head = goal->head;
        int body = goal->body;
        int jumpOver = goal->skip_leds_per_step;

        if(!head) head = 1;
        if(!body) body = 1; // Bunny consists of a single LED
        if(jumpOver <= 0) jumpOver = (int)body/2;

        int j;
        for(int i = 0; i < circle_counter; ++i)
        {
            ROS_INFO("Circle: %d", i);
            int _head = head % m_numLeds;
            int tail = (head - body) % m_numLeds;
            int tail_old = tail;
            for(j = _head; j < m_numLeds; j+=jumpOver)
            {
                //if(body < 0) continue;
                //if(head >= m_numLeds) break;

                ROS_INFO("Current head position: %d", _head);
                ROS_INFO("Current body position: %d", tail);
                ROS_INFO("Current old body position: %d", tail_old);

                if(_head > tail)
                    m_led->setRangeRGBf(goal->color.r, goal->color.g, goal->color.b, m_numLeds, tail, _head);
                else if(_head < tail)
                    m_led->setRangeRGBf(goal->color.r, goal->color.g, goal->color.b, m_numLeds, tail, _head);

                // THIS IS USED CURRENTLY
                //m_led->setRangeRGBf(0, 0, 0, m_numLeds, tail, _head);

                // If following is used it gradually fills all LEDs, turns them off at the end and cycle repeats
                // m_led->setRangeRGBf(0, 0, 0, m_numLeds, _head, _body);

                feedback.current_head_pos = _head;
                runningBunnyAS.publishFeedback(feedback);

                tail = (tail + jumpOver) % m_numLeds;
                _head = (_head + jumpOver) % m_numLeds;

                // Turn off the trail from the bunny - the distance between the old tail and the new one; this allows a much smoother transition without blinking the whole bunny
                m_led->setRangeRGBf(0, 0, 0, m_numLeds, tail-tail_old, tail);

                tail_old = tail;
            }

            m_led->setAllRGBf(0, 0, 0, m_numLeds);
        }

        //m_led->setAllRGBf(0, 0, 0, m_numLeds);
        result.current_head_pos = head;
        runningBunnyAS.setSucceeded(result);
    }*/

    // TODO Add bidirectional movement - right now the bunny can run only from 0 to 422 (or similar values) but not the other way around.
    //      The head and body can be used in order to determine in which direction the bunny has to run. If the tail's position < head's position
    //      then the bunny has to run from 0 to 422 (or similar values) otherwise it has to run from 422 to 0 (or similar values)
    /*void runningBunnyCallback(const iirob_led::RunningBunnyGoal::ConstPtr& goal) {
        // TODO Use modulo to create a neat transition between previous and next circle!
        // TODO Change RGB values using the routine from Changeling
        ROS_INFO("RUNNING BUNNY CALLBACK");
        iirob_led::RunningBunnyFeedback feedback;
        iirob_led::RunningBunnyResult result;
        int circle_counter = goal->num_circles;
        int head = goal->head;
        int body = goal->body;
        int jumpOver = goal->skip_leds_per_step;
        float r = goal->color.r;
        float g = goal->color.g;
        float b = goal->color.b;

        // The bunny has a minimal size of 2 LEDs
        if(!head) head = 1;
        if(!body) body = 1; // Bunny consists of a single LED
        if(jumpOver <= 0) jumpOver = (int)body/2;

        int tail = head - body;     // NOTE See bidirectional movement todo. This here will determine which way the bunny has to run

        ROS_INFO("Head: %d\nBody: %d\nTail: %d", head, body, tail);
        // We cut the bunny into two parts (as cruel as it may sound...). The is used when we do the 422->0 transition
        // Split in two intervals - (headSplit_start : headSplit_end) splitPos (tailSplit_start : tailSplit_end) with split = LED at position 0

        // Formular for determining the LED position where we do the split:
        // if we reach 422->0 transtition activate split
        // for each head++ incr

        //ROS_INFO("Bunny split: head[%d - %d] | tail[%d - %d]", headSplit_start, headSplit_end, tailSplit_start, tailSplit_end);

        m_led->setAllRGBf(0, 0, 0, m_numLeds);

        for(int i = 0; i < circle_counter; ++i) {
            int _head = head;
            int _tail = _head - body;
            bool activateSplit = false;
            int headSplit_end = 0;
            int tailSplit_start = m_numLeds-1;
            int _tail_old = _tail;

            ROS_INFO("Circle %d", circle_counter);

            for(int distance = 0; distance < m_numLeds; distance+=jumpOver, _head = (_head + jumpOver) % m_numLeds) {
                ROS_INFO("Bunny has ran %d out of %d LEDs so far", distance, m_numLeds-1);
                _tail = _head - body;
                ROS_INFO("Tail: %d", _tail+(m_numLeds-1));

                // Whenever _tail is negative (meaning: head has passed the 0 position but the body is still behind the 0 mark)
                if(_tail < 0) {
                    activateSplit = true;
                }

                if(activateSplit) {
                    ROS_INFO("Split active");
                    _tail = _tail + m_numLeds;

                    m_led->setRangeRGBf(r, g, b, m_numLeds, headSplit_end, _head);
                    m_led->setRangeRGBf(0, 0, 0, m_numLeds, _head, _tail);
                    m_led->setRangeRGBf(r, g, b, m_numLeds, _tail, tailSplit_start);

//                   Example:
//                   Head on position 1, body length 8 (tail = 417)
//                                       splitPos
//                                     (headSplit_end)    tailSplit_start
//                                           |                   |
//                   headSplit_start         |                   |   tailSplit_end             [x] = head/tail, (0) = splitPos = headSplit_end
//                       |                   |                   |         |
//                   <head[1]><body        [(0)]              [422] : <tail[417]>>            [1]    (0)     422     421     420     419     418     [417]
//                   <head[2]><body[1]      (0)               [422] : <tail[418]>>            [2]     1      (0)     422     421     420     419     [418]
//                   <head[3]><body[2] : [1](0)               [422] : <tail[419]>>            [3]     2       1      (0)     422     421     420     [419]
//                   <head[4]><body[3] : [1](0)               [422] : <tail[420]>>            [4]     3       2      1       (0)     422     421     [420]
//                   <head[5]><body[4] : [1](0)               [422] : <tail[421]>>            [5]     4       3      2        1      (0)     422     [421]
//                   <head[6]><body[5] : [1](0)               [422] : <tail[422]>>            [6]     5       4      3        2       1      (0)     [422]
//                   <head[6]><body[5] :                              <tail[0]>>              [6]     5       4      3        2       1      [0]            <----- splitting OFF
//                    ROS_INFO("Split state : head[%d] - headSplit_end[%d] - tailSplit_start[%d] - tail[%d]", _head, headSplit_end, tailSplit_start, _tail);
                } else {
                    //m_led->setRangeRGBf(r, g, b, m_numLeds, _head, _tail);
                    m_led->setRangeRGBf(r, g, b, m_numLeds, _tail, _head);
                    m_led->setRangeRGBf(0, 0, 0, m_numLeds, _tail-_tail_old, _tail);
                    ROS_INFO("Tail(old): %d", _tail_old);
                    ROS_INFO("Tail(new): %d", _tail);
                    _tail_old = _tail;
                    ROS_INFO("Normal state: head[%d] - tail[%d]", _head, _tail);
                }

                // The tail goes from 422 to 0
                //if(!(_tail % (m_numLeds-1))) activateSplit = false;
                if(_tail == (m_numLeds-1)) {
                    m_led->setAllRGBf(0,0,0, m_numLeds);
                    activateSplit = false;
                }

                feedback.current_head_pos = _head;
                runningBunnyAS.publishFeedback(feedback);
            }
        }

        m_led->setAllRGBf(0, 0, 0, m_numLeds);
        result.current_head_pos = head;
        runningBunnyAS.setSucceeded(result);

    }*/

    void spin() { ros::spin(); }
};

//const int LEDNode::MAX_FORCE = 10;

int main(int argc, char **argv)
{
    std::string port;
    int num_of_leds;
	bool override;

	ros::init(argc, argv, "iirob_led_node");
    ros::NodeHandle nh("~");

    nh.param<std::string>("port", port, "/dev/ttyUSB0");
    nh.param<int>("led_num", num_of_leds, 1);
    nh.param<bool>("override", override, false);

    ROS_INFO("IIROB-LED node %s: Setting override to %s", ros::this_node::getName().c_str(), (override ? (char *)"on" : (char *)"off"));

    // If more then 8 LEDs are connected to the mC it will burn out
    if (!override && (num_of_leds>8)) num_of_leds = 8;
    ROS_INFO("IIROB-LED node %s: Setting number of LEDs to %d", ros::this_node::getName().c_str(), num_of_leds);

    ROS_INFO("IIROB-LED node %s: Initializing ledNode on port %s with %d LEDs.", ros::this_node::getName().c_str(), port.c_str(), num_of_leds);
    LEDNode *ledNode = new LEDNode(nh, port, num_of_leds);
    if(ledNode->getStatus()) ledNode->spin();
    delete ledNode;
    return 0;
}
