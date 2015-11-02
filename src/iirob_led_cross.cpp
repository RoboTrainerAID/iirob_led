#include <cmath>
#include "iirob_led_cross.h"
#include "RGBConverter.h"

// TODO Check if everywhere the duration parameter of a message is set properly (value > 0). Otherwise duration takes the default value from the message (=0), which equals infinity and node can only be stopped by escalating to SIGTERM

IIROB_LED_Cross::IIROB_LED_Cross(ros::NodeHandle nodeHandle, std::string const& _port, int const& _m_numLeds)
    : m_led(0), m_msec(1),
      port(_port), m_numLeds(_m_numLeds),
      // Initialize all action servers and bind them to the respective callbacks
      blinkyAS(nodeHandle, "blinky", boost::bind(&IIROB_LED_Cross::blinkyCallback, this, _1), false),
      policeAS(nodeHandle, "police", boost::bind(&IIROB_LED_Cross::policeCallback, this, _1), false),
      fourRegionsAS(nodeHandle, "four_regions", boost::bind(&IIROB_LED_Cross::fourRegionsCallback, this, _1), false),
      chaserLightAS(nodeHandle, "chaser_light", boost::bind(&IIROB_LED_Cross::chaserLightCallback, this, _1), false),
      MAX_FORCE(10)
{
    // Initialize the hardware
    status = init(port, m_numLeds);

    // If hardware fails to initialize we have to stop
    if(!status) {
        ROS_ERROR("Initiating port %s with number %d failed!", port.c_str(), m_numLeds);
        return;
    }

    ROS_INFO("led_force subscriber started");
    subForce = nodeHandle.subscribe("led_force", 10, &IIROB_LED_Cross::forceCallback, this);
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
    blinkyAS.shutdown();
    policeAS.shutdown();
    fourRegionsAS.shutdown();
    chaserLightAS.shutdown();
}

void IIROB_LED_Cross::checkLimits(int *start_led, int *end_led) {
    if(*start_led > m_numLeds) *start_led = m_numLeds;
    else if(*start_led < 0) *start_led = 0;

    if(*end_led > m_numLeds) *end_led = m_numLeds;
    else if(*end_led < 0) *end_led = 0;
}

bool IIROB_LED_Cross::init(std::string const& port, int const& m_numLeds) {
    // Initialize the hardware
    m_led = new iirob_hardware::LEDStrip(port);
    // Start the action servers only if the hardware has been successfully initialized
    if (m_led->ready()) {
        //m_led->setAllRGBf(0, 0, 0, m_numLeds);
        //m_led->setUniRGB(0, 0, 0);

        ROS_INFO("%s: IIROB-LED action server started", ros::this_node::getName().c_str());
        ROS_INFO("Number of LEDs: %d", m_numLeds);

        blinkyAS.start();
        ROS_INFO("blinky action server started");
        policeAS.start();
        ROS_INFO("police action server started");
        fourRegionsAS.start();
        ROS_INFO("four_regions action server started");
        chaserLightAS.start();
        ROS_INFO("chaser_light action server started");
        return true;
    }
    return false;
}

void IIROB_LED_Cross::spin() { ros::spin(); }

bool IIROB_LED_Cross::getStatus() { return status; }

// Callbacks for all action servers and subscribers
void IIROB_LED_Cross::forceCallback(const iirob_led::DirectionWithForce::ConstPtr& led_force_msg) {
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

    m_led->setRangeRGBf(0, 0, 0, m_numLeds, 383, 383);
    m_led->setRangeRGBf(0, 0, 0, m_numLeds, 108, 108);
    m_led->setRangeRGBf(0, 0, 0, m_numLeds, 84+108-1, 84+108-1);
    m_led->setRangeRGBf(0, 0, 0, m_numLeds, 84+2*108, 84+2*108);

    m_led->setRangeRGBf(1, 0, 0, m_numLeds, 383, 383);
    m_led->setRangeRGBf(0, 1, 0, m_numLeds, 108, 108);
    m_led->setRangeRGBf(0, 0, 1, m_numLeds, 84+108-1, 84+108-1);
    m_led->setRangeRGBf(1, 1, 0, m_numLeds, 84+2*108, 84+2*108);

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

void IIROB_LED_Cross::blinkyCallback(const iirob_led::BlinkyGoal::ConstPtr& goal) {
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
    if(num_leds != 0) { // num_leds always takes precendence over end_led whenever if is != 0 and doesn't exceed the total number of LEDs in the strip
        if(num_leds <= m_numLeds) {
            // Use the num_leds as offset from start_led to calculate the new end_led
            end_led = (start_led + num_leds) % m_numLeds;
            //end_led = (start_led-1 + num_leds) % m_numLeds;   // FIXME num_leds seems to be broken
            //end_led = (start_led + num_leds); // OLD
            ROS_WARN("num_leds contains a non-zero value and will override end_led");
        }
        else
            ROS_WARN("num_leds contains a non-zero value, which however exceeds the total number of available LEDs in the strip. Falling back to end_led.");
    }

    //if(start_led < 0) start_led = 0;
    //if(end_led > m_numLeds) end_led = m_numLeds;
    checkLimits(&start_led, &end_led);

    ROS_INFO("start_led: %d | end_led: %d", start_led, end_led);
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

    // Calculate the speed which the strip will be moving at
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

    //m_led->setAllRGBf(0, 0, 0, m_numLeds);
    result.blinks_left = blinks_left;
    blinkyAS.setSucceeded(result);
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

void IIROB_LED_Cross::fourRegionsCallback(const iirob_led::FourRegionsGoal::ConstPtr& goal) {
    iirob_led::FourRegionsFeedback feedback;
    iirob_led::FourRegionsResult result;
    int blinks_left = goal->blinks;
    int start_led = goal->start_led;
    int end_led = goal->end_led;

    //if(start_led < 0) start_led = 0;
    //if(end_led > m_numLeds) end_led = m_numLeds;
    checkLimits(&start_led, &end_led);

    if(abs(end_led - start_led) < 3) {
        start_led = 0;
        end_led = m_numLeds;
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

    ROS_INFO("%s starting Four Musketeers action with duration(ON): %.2f sec and duration(OFF): %.2f\n\n\t Region1\tstart led: %d, end led: %d\tcolor: %.2f %.2f %.2f \n\t Region2\tstart led: %d, end led: %d\tcolor: %.2f %.2f %.2f \n\t Region3\tstart led: %d, end led: %d\tcolor: %.2f %.2f %.2f \n\t Region4\tstart led: %d, end led: %d\tcolor: %.2f %.2f %.2f ",
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

    //m_led->setAllRGBf(0, 0, 0, m_numLeds);
    result.blinks_left = blinks_left;
    fourRegionsAS.setSucceeded(result);
}

void IIROB_LED_Cross::chaserLightCallback(const iirob_led::ChaserLightGoal::ConstPtr& goal) {
    iirob_led::ChaserLightFeedback feedback;
    iirob_led::ChaserLightResult result;

    int head = (goal->start_led < 0) ? 0 : goal->start_led;
    int offset = goal->num_of_leds;
    if(offset <= 0) offset = 2;
    int tail;

    int cycles = goal->num_of_cycles;
    //double cycle_duration = goal->cycle_duration;
    //if(cycle_duration <= 0) cycle_duration = 10.0;  // Very important to avoid negative and especially 0 values here due to the way skip_per_step is currently caluclated
    // TODO Add the feature "reverse direction"
    bool reverse = goal->reverse;

    //double single_step_duration = cycle_duration / m_numLeds; // Number of LEDs that the strip will skip

    // TODO See how to determine the speed properly. Skipped LEDs per step offer much greater flexibility when it comes to the speed which the snake traverses the circle at. If we
    // use a skip_per_step = 1 and then use ros::Duration() we have a big problem since the loop itself is very slow when we skip a single LED only and adding sleep() will just make things slower
    // lowest skip_per_step is 1
    //int skip_per_step = m_numLeds / cycle_duration; // Speed based on the number of LEDs per cycle and how long it takes to traverse all
    int skip_per_step = goal->leds_per_step;
    if(skip_per_step <= 0 || skip_per_step >= 20) skip_per_step = 3;

    //ROS_DEBUG("head: %d | offset: %d | time per step: %f", head, offset, single_step_duration);

    int old_tail;  // will use this to turn off the trail after the strip has moved
    for(int current_cycle = 0; current_cycle < cycles; current_cycle++) {
        // Prepare for the cycle
        head = (goal->start_led < 0) ? 0 : goal->start_led;
        tail = (head - offset);
        if(tail < 0) tail = tail + m_numLeds;
        old_tail = tail;
        // Movement within a single cycle
        for(int pos = 0; pos < m_numLeds; pos+=skip_per_step) {
            ROS_DEBUG("Head: %d | Tail: %d | ", head, tail);
            // Light up the strip
            m_led->setRangeRGBf(goal->color.r, goal->color.g, goal->color.b, m_numLeds, tail, head);
            // TODO Add HSV gradient (head is brightest, tail is dimmest). This will be possible only then when we can control a single LED
            // Wait a little bit
            //ros::Duration(single_step_duration).sleep();  // required but for now I can't figure out how to combine it with the rest speed-related stuff
            // Update the head and tail position of the strip
            head = (head + skip_per_step) % m_numLeds;
            tail = (head - offset);
            if(tail < 0) tail = tail + m_numLeds;
            ROS_DEBUG("New tail: %d | old tail: %d", tail, old_tail);
            // Turn off all LEDs between the old end position and the new end position of the strip
            m_led->setRangeRGBf(0, 0, 0, m_numLeds, old_tail, tail);

            old_tail = tail;

            feedback.current_start_pos = head;
            chaserLightAS.publishFeedback(feedback);
        }
        ROS_DEBUG("**********************Finished with cycle %d**********************", current_cycle+1);
    }

    m_led->setAllRGBf(0, 0, 0, m_numLeds);
    //m_led->setRangeRGBf(0, 0, 0, m_numLeds, head, tail);
    feedback.current_start_pos = head;
    chaserLightAS.publishFeedback(feedback);
    result.current_start_pos = head;
    chaserLightAS.setSucceeded(result);
}
