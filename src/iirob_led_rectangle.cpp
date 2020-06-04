//#include <cmath>
#include <math.h>
#include "iirob_led/iirob_led_rectangle.h"
#include "iirob_led/iirob_led_base.h"

IIROB_LED_Rectangle::IIROB_LED_Rectangle(ros::NodeHandle nodeHandle, std::string const& _port, int const& _mNumLeds, double _maxForce, std::string link)
    : IIROB_LED_Base::IIROB_LED_Base(nodeHandle, _port, _mNumLeds, _maxForce, MAX_NUM_LEDS_RECTANGLE, link)
{
    // Initialize the tf2-related components
    buf = new tf2_ros::Buffer();
    tfl = new tf2_ros::TransformListener(*buf, nodeHandle);

    ros::NodeHandle nh("~");

    int rect_left_len, rect_back_len, rect_right_len, rect_front_len;

    if (!nh.hasParam("rect_left_len")) {
        ROS_WARN("Rectange parameters 'rect_left_len' not found. Using default values");
    }

    nh.param("rect_left_len", rect_left_len, -1);
    nh.param("rect_back_len", rect_back_len, -1);
    nh.param("rect_right_len", rect_right_len, -1);
    nh.param("rect_front_len", rect_front_len, -1);

    if (rect_left_len == -1 or rect_back_len == -1 or rect_right_len == -1 or rect_front_len == -1)
    {
        ROS_WARN("Rectange parameters 'rect_*_len' set to -1. Using default values");
        rect_front_const_ = RECT_FRONT;
        rect_back_const_ = RECT_BACK;
        rect_left_const_ = RECT_LEFT;
        rect_right_const_ = RECT_RIGHT;
    }
    else
    {
        rect_front_const_ = rect_left_len + rect_back_len + rect_right_len + rect_front_len/2;
        rect_back_const_ = rect_left_len + rect_back_len/2;
        rect_left_const_ = rect_left_len/2;
        rect_right_const_ = (rect_left_len - 1) + rect_back_len + rect_right_len/2;
    }

}

IIROB_LED_Rectangle::~IIROB_LED_Rectangle() {
    delete buf;
    delete tfl;
}

// Callbacks for all action servers and subscribers
void IIROB_LED_Rectangle::forceWithColorCallback(const iirob_led::ForceWithColor::ConstPtr& ledForceWithColorMsg) {

    // Process the transformation information
    geometry_msgs::Vector3Stamped forceIn, forceOut;
    forceIn.vector = ledForceWithColorMsg->force.wrench.force;
    geometry_msgs::TransformStamped tfStamped;
    ROS_DEBUG("Transforming from \"%s\" to \"%s\" (local frame for cross)", ledForceWithColorMsg->force.header.frame_id.c_str(), localFrame.c_str());
    try
    {
        tfStamped = buf->lookupTransform(localFrame, ledForceWithColorMsg->force.header.frame_id, ros::Time(0));
        tf2::doTransform(forceIn, forceOut, tfStamped);
    }
    catch(tf2::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }

    // Calculate the force
    double x = forceOut.vector.x;
    double y = forceOut.vector.y;

    if(x == 0 && y == 0)
    {
        ROS_DEBUG("Both x and y coordinates equal zero which does not allow visualization in 2D space.");
        return;
    }

    double xScaled = convert(_scalingFactor, std::fabs(x));
    double yScaled = convert(_scalingFactor, std::fabs(y));

    ROS_DEBUG("x: %f, y: %f | scaled x: %f, y: %f", x, y, xScaled, yScaled);

    double force = getVector2dLenght(xScaled, yScaled);
    if(force > maxForce)
    {
        ROS_ERROR("Force %.3f (using rounded xScaled and yScaled) exceeds specified maximum force %.3f", force, maxForce);
        return;
    }
    // Scale the received force to be in the interval between 0 and maxForce (maxFroce rounded up and converted to an integer - it will represent the number of LEDs to be lit)
    // [0] ----- [force] -- [maxForce]
    int forceRounded = (int)round(force);
    // Round: (down) X.00...01 = X.49...99 = X | (up) X.50...00 = X.99...99 = X+1
    //double forceRemainder = force - forceRounded; // Store the remainder (if any) which will be used as the V (in HSV) value of the last LED in the range of LEDs that display forceRounded
    double forceRemainder = std::fmod(force, 1);
    if((force - forceRounded) > 0.) forceRounded++;


    /*if(forceRemainder > 0)                      // Example: force = 2.3, forceRounded = 2 => forceRemainder = force - forceRounded = 2.3 - 2 = 0.3 => we have rounded DOWN the force
        forceRounded++;                         // forceRounded++ = 2+1 = 3 => 3 LEDs with last LED using V (in HSV) = 0.3
    else if(forceRemainder < 0)                 // Example: force = 2.7, forceRounded = 3 => forceRemainder = force - forceRounded = 2.7 - 3 = -0.3 => we have rounded UP the force
        forceRemainder = 1 + forceRemainder;    // forceRemainder = 1 + (-0.3) = 0.7 => 3 LEDs with the last LED using V (in HSV) = 0.7*/

    ROS_DEBUG("Max number of LEDs: %d", MAX_NUM_LEDS_RECTANGLE);
    ROS_DEBUG("Force: %.3f | Rounded and int: %d", force, forceRounded);
    if(forceRounded > maxForce)
    {
        ROS_ERROR("Received force is of greater magnitude than the set upper bound or exceed the numer of LEDs that can be displayed on each side of the platform | forceRounded(%d), forceMax(%d)", forceRounded, MAX_NUM_LEDS_RECTANGLE);
        return;
    }

    int direction = 0;
    double angle = 0.;
    double ledPerDeg = 0.;
    double translationAlongStrip;
    bool singleAxes = ((!x  && y) || (x && !y));    // If true, then either x or y equal 0
    if(!singleAxes)
    {
        angle = atan2(y,x);  // Angle in radians
        ledPerDeg = mNumLeds/360.;
        translationAlongStrip = (angle*180./M_PI) * ledPerDeg;
        direction = (int)round(rect_front_const_ + translationAlongStrip) % mNumLeds;
    }
    else
    {
        if(!x  && y < 0) direction = rect_right_const_;
        else if(!x && y > 0) direction = rect_left_const_;
        else if(!y && x > 0) direction = rect_front_const_;
        else if(!y && x < 0) direction = rect_back_const_;
    }

    ROS_DEBUG("XY coordinates: [%.3f , %.3f]\t|\tForce (upper limit of %.3f): %.3f", x, y, maxForce, force);
    ROS_DEBUG("Led/Angle: %f | Angle: %frad (=%fdeg) | Translation (num of LEDs): %f | location (LED index): %d", ledPerDeg, angle, angle*180./M_PI, translationAlongStrip, direction);

    mLed->setAllRGBf(0, 0, 0, mNumLeds); //

    float r = ledForceWithColorMsg->color.r, g = ledForceWithColorMsg->color.g, b = ledForceWithColorMsg->color.b;

    if(forceRounded == 1)
    {

        if(forceRemainder != 0)
        {
            float h, s, v;
            RGBConverter::rgbToHsv(r, g, b, &h, &s, &v);
            v = forceRemainder; // Use the remainder as V
            RGBConverter::hsvToRgb(h, s, v, &r, &g, &b);
        }
        mLed->setRangeRGBf(r, g, b, mNumLeds, direction, direction);
        return;
    }

    forceRounded--;

    mLed->setRangeRGBf(r, g, b, mNumLeds,
                        ((direction - forceRounded) > 0) ? (direction - forceRounded) : mNumLeds - (forceRounded - direction),
                        (direction + forceRounded),
                        true, true, false);
    if(forceRemainder != 0)
    {
        float h, s, v;
        RGBConverter::rgbToHsv(r, g, b, &h, &s, &v);
        v = forceRemainder; // Use the remainder as V
        RGBConverter::hsvToRgb(h, s, v, &r, &g, &b);
        // First and last LED in the range of LEDs use the fourceRemainder as their V (in HSV)
        mLed->setRangeRGBf(r, g, b, mNumLeds,
                            ((direction - forceRounded) > 0) ? (direction - forceRounded) : mNumLeds - (forceRounded - direction),
                            ((direction - forceRounded) > 0) ? (direction - forceRounded) : mNumLeds - (forceRounded - direction));
        mLed->setRangeRGBf(r, g, b, mNumLeds,
                            (direction + forceRounded),
                            (direction + forceRounded));
    }

    mLed->setRangeRGBf(0, 0, 1, mNumLeds, direction, direction);
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
            mLed->setRangeRGBf(goal->color_outer.r, goal->color_outer.g, goal->color_outer.b, mNumLeds, start_led_left_outer, end_led_left_outer, true, true, false);
            mLed->setRangeRGBf(goal->color_outer.r, goal->color_outer.g, goal->color_outer.b, mNumLeds, start_led_right_outer, end_led_right_outer);
            ros::Duration(0.5).sleep();

            mLed->setRangeRGBf(0, 0, 0, mNumLeds, start_led_left_outer, end_led_left_outer, true, true, false);
            mLed->setRangeRGBf(0, 0, 0, mNumLeds, start_led_right_outer, end_led_right_outer);
            ros::Duration(0.05).sleep();

            // Blink the inner subsections
            mLed->setRangeRGBf(goal->color_inner.r, goal->color_inner.g, goal->color_inner.b, mNumLeds, start_led_left_inner, end_led_left_inner, true, true, false);
            mLed->setRangeRGBf(goal->color_inner.r, goal->color_inner.g, goal->color_inner.b, mNumLeds, start_led_right_inner, end_led_right_inner);
            //ros::Duration(goal->fast_duration_on).sleep();
            ros::Duration(0.5).sleep();

            mLed->setRangeRGBf(0, 0, 0, mNumLeds, start_led_left_inner, end_led_left_inner, true, true, false);
            mLed->setRangeRGBf(0, 0, 0, mNumLeds, start_led_right_inner, end_led_right_inner);
            //ros::Duration(goal->fast_duration_off).sleep();
            ros::Duration(0.05).sleep();

            // Send feedback
            feedback.fast_blinks_left = fast_blinks_left;
            feedback.blinks_left = blinks_left;
            policeAS.publishFeedback(feedback);
        }

        // Make a pause before the next set of fast blinks
        //ros::Duration(goal->pause_between_long_blinks).sleep();
        ros::Duration(2.0).sleep();
        fast_blinks_left = goal->fast_blinks;
    }

    //m_led->setAllRGBf(0, 0, 0, m_numLeds);
    result.blinks_left = blinks_left;
    policeAS.setSucceeded(result);
}
