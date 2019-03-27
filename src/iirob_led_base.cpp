#include <cmath>
#include "RGBConverter.h"
#include "iirob_led/iirob_led_base.h"

IIROB_LED_Base::IIROB_LED_Base(ros::NodeHandle nodeHandle, std::string const& _port, int const& _mNumLeds, double _maxForce, int maxForceLeds, std::string link)
    : mLed(0), mMSec(1),
      port(_port), mNumLeds(_mNumLeds),
      localFrame(link),
      maxForce(_maxForce),
      // Initialize all action servers and bind them to the respective callbacks (wherever callback is virtual see the inheriting class for more details)
      policeAS(nodeHandle, "police", boost::bind(&IIROB_LED_Base::policeCallback, this, _1), false),
      blinkyAS(nodeHandle, "blinky", boost::bind(&IIROB_LED_Base::blinkyCallback, this, _1), false),
      fourRegionsAS(nodeHandle, "four_regions", boost::bind(&IIROB_LED_Base::fourRegionsCallback, this, _1), false),
      chaserLightAS(nodeHandle, "chaser_light", boost::bind(&IIROB_LED_Base::chaserLightCallback, this, _1), false)
{
    //ROS_INFO("***************** LOCAL FRAME = %s", localFrame.c_str());
    // Initialize the hardware
    status = init(port, mNumLeds);

    // If hardware fails to initialize we have to stop
    if(!status) {
        ROS_ERROR("Initiating port %s with number %d failed!", port.c_str(), mNumLeds);
        return;
    }

    _scalingFactor = scalingFactor(0, maxForce, 0, maxForceLeds);

    subSetLedRange = nodeHandle.subscribe("led_setRange", 10, &IIROB_LED_Base::setLedRangeCallback, this);
    ROS_DEBUG("led_onoff subscriber started");
    subSetLeds = nodeHandle.subscribe("led_setLeds", 10, &IIROB_LED_Base::setLedsCallback, this);
    ROS_DEBUG("led_setLeds subscriber started");
    subForce = nodeHandle.subscribe("/base/threshold_filtered", 10, &IIROB_LED_Base::forceCallback, this);
    ROS_DEBUG("led_force subscriber started");
    subForceWithColor = nodeHandle.subscribe("led_force_with_color", 10, &IIROB_LED_Base::forceWithColorCallback, this);
    ROS_DEBUG("led_force_with_color subscriber started");

    turnOnOffSS = nodeHandle.advertiseService<iirob_led::TurnOnOff::Request, iirob_led::TurnOnOff::Response>("turn_onoff", boost::bind(&IIROB_LED_Base::turnOnOffCallback, this, _1, _2));
}

IIROB_LED_Base::~IIROB_LED_Base() {
    ROS_DEBUG("Turning all LEDs off");
    // Turn all LEDs off and delete the m_led
    if (mLed) {
        mLed->setAllRGBf(0, 0, 0, mNumLeds);
        mLed->setUniRGB(0, 0, 0);
        delete mLed;
    }

    ROS_DEBUG("Shutting down service and action servers and subscribers");
    subSetLedRange.shutdown();
    subForce.shutdown();
    turnOnOffSS.shutdown();
    blinkyAS.shutdown();
    policeAS.shutdown();
    fourRegionsAS.shutdown();
    chaserLightAS.shutdown();
}

void IIROB_LED_Base::checkLimits(int *start_led, int *end_led) {
    if(*start_led > mNumLeds) *start_led = mNumLeds;
    else if(*start_led < 0) *start_led = 0;

    if(*end_led > mNumLeds) *end_led = mNumLeds;
    else if(*end_led < 0) *end_led = 0;
}

bool IIROB_LED_Base::init(std::string const& port, int const& m_numLeds) {
    // Initialize the hardware
    mLed = new iirob_hardware::LEDStrip(port);
    // Start the action servers only if the hardware has been successfully initialized
    if (mLed->ready()) {
        //m_led->setAllRGBf(0, 0, 0, m_numLeds);
        //m_led->setUniRGB(0, 0, 0);

        ROS_DEBUG("%s: IIROB-LED action server started", ros::this_node::getName().c_str());
        ROS_DEBUG("Number of LEDs: %d", m_numLeds);

        blinkyAS.start();
        ROS_DEBUG("blinky action server started");
        policeAS.start();
        ROS_DEBUG("police action server started");
        fourRegionsAS.start();
        ROS_DEBUG("four_regions action server started");
        chaserLightAS.start();
        ROS_DEBUG("chaser_light action server started");
        return true;
    }
    return false;
}

void IIROB_LED_Base::spin() { ros::spin(); }

bool IIROB_LED_Base::getStatus() { return status; }

// TODO Create service that accepts boolean parameter for power on/off + parameter for colour
// ...

// TODO Convert to service
void IIROB_LED_Base::setLedRangeCallback(const iirob_led::SetLedRange::ConstPtr& led_setRange_msg)
{
    // Check if all or too many LEDs have been selected
    if(!led_setRange_msg->color.r && !led_setRange_msg->color.g && !led_setRange_msg->color.b &&
            ((led_setRange_msg->start_led <= 0 && led_setRange_msg->end_led >= mNumLeds) ||
             (led_setRange_msg->start_led >= mNumLeds && led_setRange_msg->end_led <= 0)) ||
              led_setRange_msg->num_leds > mNumLeds) {
        mLed->setAllRGBf(0, 0, 0, mNumLeds);
        return;
    }

    int num_leds = led_setRange_msg->num_leds;
    int start_led = led_setRange_msg->start_led;
    int end_led = led_setRange_msg->end_led;
    checkLimits(&start_led, &end_led);
    if(num_leds <= mNumLeds && num_leds > 0) {
        // Use the num_leds as offset from start_led to calculate the new end_led
        end_led = (start_led + num_leds) % mNumLeds;
        ROS_WARN("num_leds contains a valid non-zero value and will override end_led");
    }
    else
        ROS_WARN("num_leds contains a non-zero value, which exceeds the total number of available LEDs in the strip. Falling back to end_led.");

    mLed->setRangeRGBf(led_setRange_msg->color.r, led_setRange_msg->color.g, led_setRange_msg->color.b, mNumLeds, start_led, end_led);
}

void IIROB_LED_Base::setLedsCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    int pixel_size = sensor_msgs::image_encodings::numChannels(msg->encoding);

    uint step = msg->step;
    int led_index;

    if (step == 0) {
        step = msg->width*pixel_size;
    }

    if (msg->height*msg->width > mNumLeds)
        return;

    for (int i=0; i < msg->height; i++) {
        for (int j=0; j < msg->width; j++) {
            if (pixel_size == 4) {
                if (msg->data[i*step+(j*pixel_size)+3] == 0) {
                    continue;
                }
            }
            led_index = (i*step + j);
            mLed->setRangeRGB(msg->data[i*step+j*pixel_size], msg->data[i*step+j*pixel_size+1], msg->data[i*step+j*pixel_size+2],
                                1, led_index, led_index, false, false, false);
        }
    }
    mLed->triggerSend(mNumLeds);
}

bool IIROB_LED_Base::turnOnOffCallback(iirob_led::TurnOnOff::Request& turnOnOff_Req_msg, iirob_led::TurnOnOff::Response& turnOnOff_Resp_msg)
{
    bool res = true;
    if(turnOnOff_Req_msg.turn_on)
    {
        if(!turnOnOff_Req_msg.color.r && !turnOnOff_Req_msg.color.g && !turnOnOff_Req_msg.color.b)
        {
            ROS_ERROR("Trying to turn all selected LEDs with R = G = B = 0");
            res = false;
        }
        else res = mLed->setAllRGBf(turnOnOff_Req_msg.color.r, turnOnOff_Req_msg.color.g, turnOnOff_Req_msg.color.b, mNumLeds);
    }
    else
    {
        if(turnOnOff_Req_msg.color.r || turnOnOff_Req_msg.color.g || turnOnOff_Req_msg.color.b)
            ROS_WARN("Color selected even though trying to turn all selected LEDs off");
        res = mLed->setAllRGBf(0., 0., 0., mNumLeds);
    }

//    bool res = mLed->setAllRGBf(0, 0, 0, mNumLeds);
//    turnOnOff_msg->success = res;
//    turnOnOff_msg->message = (res) ? "Turning off all LEDs successful" : "Turning off all LEDs failed";

    std::string resp_msg = "";
    if(turnOnOff_Req_msg.turn_on) resp_msg = "on";
    else resp_msg = "off";

    turnOnOff_Resp_msg.message = "Turning " + resp_msg + " LEDs " + ((res) ? "successful" : "failed");
    turnOnOff_Resp_msg.success = res;

    return true;
}

void IIROB_LED_Base::blinkyCallback(const iirob_led::BlinkyGoal::ConstPtr& goal) {
    // FIXME When fade in and out are turned off only LED at index 0 blinks (the rest is lit up all the time throughout the execution of the action)
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
        if(num_leds <= mNumLeds) {
            // Use the num_leds as offset from start_led to calculate the new end_led
            end_led = (start_led + num_leds) % mNumLeds;
            ROS_WARN("num_leds contains a non-zero value and will override end_led");
        }
        else
            ROS_WARN("num_leds contains a non-zero value, which however exceeds the total number of available LEDs in the strip. Falling back to end_led.");
    }

    checkLimits(&start_led, &end_led);

    ROS_DEBUG("start_led: %d | end_led: %d", start_led, end_led);
    ROS_DEBUG("%s starting Blinky action with color [%f %f %f] [RGB]. Duration(ON): %f sec, Duration(OFF): %f, start led: %d, end led: %d",
             ros::this_node::getName().c_str(),
             goal->color.r, goal->color.g, goal->color.b, goal->duration_on, goal->duration_off, start_led, end_led);

    float r = goal->color.r;
    float g = goal->color.g;
    float b = goal->color.b;
    float h, s, v, v_old;
    // Initial conversion
    RGBConverter::rgbToHsv(r, g, b, &h, &s, &v);
    ROS_DEBUG("Target full color (RGB): %.2f %.2f %.2f", r, g, b);
    ROS_DEBUG("Target full color (HSV): %.2f %.2f %.2f", h, s, v);
    v_old = v;

    // If fade_in/fade_out is enable we assign a fixed portion of duration_on to it
    double fade_in_duration = 0, fade_out_duration = 0;
    if(fade_in) fade_in_duration = goal->duration_on/4;
    if(fade_out) fade_out_duration = goal->duration_on/4;
    double new_duration = goal->duration_on - (fade_in_duration + fade_out_duration);   // the duration when the LEDs will be light up at their full capacity is recalculated based on whether fade_in and/or fade_out has been turned on
    ROS_DEBUG("duration_on (%f) will be split into {fade_in: %f | fade_out: %f | full: %f}", goal->duration_on, fade_in_duration, fade_out_duration, new_duration);

    // It is advisable to use a single step of 0.01 for the V (in HSV) for low durations. For longer consider also adding a connection to the duration so that the granularity can be increased resulting in some eye candy
    // TODO Control steps_per_fade_cycle via launch parameter
    const int steps_per_fade_cycle = 100; // hardcoded number of steps per fade cycle; using this we can derive the value of a single step
    double single_step = v / steps_per_fade_cycle;  // this determines the HSV value increment/decrement and is based on the steps per fade cycle
    const double single_step_duration = (goal->duration_on/4) / steps_per_fade_cycle; // the duration of a single step is derived from the duration of the fade cycle devided by the steps per cycle
    ROS_DEBUG("Single HSV step: %.5f | Single HSV step duration: %.10f | Steps per fade cycle: %d", single_step, single_step_duration, steps_per_fade_cycle);

    for(int i = 0; i < goal->blinks; ++i, --blinks_left)
    {
        v = 0.0;    // Reset value (HSV) since this is the component that we will be working with
        // Fade in phase (if enabled)
        if(fade_in) {
            ROS_DEBUG("Fade in");
            for(int current_step = 0; current_step < steps_per_fade_cycle; v += single_step, ++current_step) {
                RGBConverter::hsvToRgb(h, s, v, &r, &g, &b);
                mLed->setRangeRGBf(r, g, b, mNumLeds, start_led, end_led);
                ros::Duration(single_step_duration).sleep();
                ROS_DEBUG("[%d] HSV: %.2f %.2f %.2f", current_step, h, s, v);
            }
        }

        // Full light phase
        ROS_DEBUG("Full");
        mLed->setRangeRGBf(goal->color.r, goal->color.g, goal->color.b, mNumLeds, start_led, end_led);
        ros::Duration(new_duration).sleep();

        v = v_old;
        // Fade out phase (if enabled)
        if(fade_out) {
            ROS_DEBUG("Fade out");
            for(int current_step = steps_per_fade_cycle-1; current_step >= 0; v -= single_step, --current_step) {
                RGBConverter::hsvToRgb(h, s, v, &r, &g, &b);
                mLed->setRangeRGBf(r, g, b, mNumLeds, start_led, end_led);
                ros::Duration(single_step_duration).sleep();
                ROS_DEBUG("[%d] HSV: %.2f %.2f %.2f", current_step, h, s, v);
            }
        }

        // Turn off LEDs
        mLed->setRangeRGBf(0, 0, 0, mNumLeds, goal->start_led, goal->end_led);
        ros::Duration(goal->duration_off).sleep();

        feedback.blinks_left = blinks_left;
        blinkyAS.publishFeedback(feedback);
    }

    feedback.blinks_left = blinks_left;

    mLed->setAllRGBf(0, 0, 0, mNumLeds);
    result.blinks_left = blinks_left;
    blinkyAS.setSucceeded(result);
}

void IIROB_LED_Base::fourRegionsCallback(const iirob_led::FourRegionsGoal::ConstPtr& goal) {
    iirob_led::FourRegionsFeedback feedback;
    iirob_led::FourRegionsResult result;
    int blinks_left = goal->blinks;
    int start_led = goal->start_led;
    int end_led = goal->end_led;

    checkLimits(&start_led, &end_led);

    if(std::abs(end_led - start_led) < 3) {
        start_led = 0;
        end_led = mNumLeds;
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

    ROS_DEBUG("%s starting FourRegions action with duration(ON): %.2f sec and duration(OFF): %.2f\n\n\t Region1\tstart led: %d, end led: %d\tcolor: %.2f %.2f %.2f \n\t Region2\tstart led: %d, end led: %d\tcolor: %.2f %.2f %.2f \n\t Region3\tstart led: %d, end led: %d\tcolor: %.2f %.2f %.2f \n\t Region4\tstart led: %d, end led: %d\tcolor: %.2f %.2f %.2f ",
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
        mLed->setRangeRGBf(goal->color1.r, goal->color1.g, goal->color1.b, mNumLeds, start_ledRegion1, end_ledRegion1, true, true, false);
        mLed->setRangeRGBf(goal->color2.r, goal->color2.g, goal->color2.b, mNumLeds, start_ledRegion2, end_ledRegion2, true, true, false);
        mLed->setRangeRGBf(goal->color3.r, goal->color3.g, goal->color3.b, mNumLeds, start_ledRegion3, end_ledRegion3, true, true, false);
        mLed->setRangeRGBf(goal->color4.r, goal->color4.g, goal->color4.b, mNumLeds, start_ledRegion4, end_ledRegion4);
        ros::Duration(goal->duration_on).sleep();
        // Set selected LEDs and turn them off
        mLed->setRangeRGBf(0, 0, 0, mNumLeds, start_ledRegion1, end_ledRegion1, true, true, false);
        mLed->setRangeRGBf(0, 0, 0, mNumLeds, start_ledRegion2, end_ledRegion2, true, true, false);
        mLed->setRangeRGBf(0, 0, 0, mNumLeds, start_ledRegion3, end_ledRegion3, true, true, false);
        mLed->setRangeRGBf(0, 0, 0, mNumLeds, start_ledRegion4, end_ledRegion4);
        ros::Duration(goal->duration_off).sleep();

        feedback.blinks_left = blinks_left;
        fourRegionsAS.publishFeedback(feedback);
    }

    feedback.blinks_left = blinks_left;

    mLed->setAllRGBf(0, 0, 0, mNumLeds);
    result.blinks_left = blinks_left;
    fourRegionsAS.setSucceeded(result);
}

void IIROB_LED_Base::chaserLightCallback(const iirob_led::ChaserLightGoal::ConstPtr& goal) {
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

    int old_tail;  // will use this to turn off the trail after the strip has moved
    for(int current_cycle = 0; current_cycle < cycles; current_cycle++) {
        // Prepare for the cycle
        head = (goal->start_led < 0) ? 0 : goal->start_led;
        tail = (head - offset);
        if(tail < 0) tail = tail + mNumLeds;
        old_tail = tail;
        // Movement within a single cycle
        for(int pos = 0; pos < mNumLeds; pos+=skip_per_step) {
            ROS_DEBUG("Head: %d | Tail: %d | ", head, tail);
            // Light up the strip
            mLed->setRangeRGBf(goal->color.r, goal->color.g, goal->color.b, mNumLeds, tail, head);
            // TODO Add HSV gradient (head is brightest, tail is dimmest). This will be possible only then when we can control a single LED
            // Wait a little bit
            //ros::Duration(single_step_duration).sleep();  // required but for now I can't figure out how to combine it with the rest speed-related stuff
            // Update the head and tail position of the strip
            head = (head + skip_per_step) % mNumLeds;
            tail = (head - offset);
            if(tail < 0) tail = tail + mNumLeds;
            ROS_DEBUG("New tail: %d | old tail: %d", tail, old_tail);
            // Turn off all LEDs between the old end position and the new end position of the strip
            mLed->setRangeRGBf(0, 0, 0, mNumLeds, old_tail, tail);

            old_tail = tail;

            feedback.current_start_pos = head;
            chaserLightAS.publishFeedback(feedback);
        }
    }

    mLed->setAllRGBf(0, 0, 0, mNumLeds);
    feedback.current_start_pos = head;
    chaserLightAS.publishFeedback(feedback);
    result.current_start_pos = head;
    chaserLightAS.setSucceeded(result);
}

void IIROB_LED_Base::forceCallback(const geometry_msgs::WrenchStamped::ConstPtr& ledForceMsg) {

   boost::shared_ptr<iirob_led::ForceWithColor> p_forceWithColor;
   p_forceWithColor.reset(new iirob_led::ForceWithColor());
   
   p_forceWithColor->force.header = ledForceMsg->header;
   p_forceWithColor->force.wrench = ledForceMsg->wrench;
   p_forceWithColor->color.r = 0.7;
   p_forceWithColor->color.g = 1;
   p_forceWithColor->color.b = 0.3;
   
   this->forceWithColorCallback(p_forceWithColor);
}

