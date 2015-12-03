#include "iirob_led_cross.h"

// TODO Check if everywhere the duration parameter of a message is set properly (value > 0). Otherwise duration takes the default value from the message (=0), which equals infinity and node can only be stopped by escalating to SIGTERM

// CROSS TEST - Useful in the future as introduction into the topic and a tool for easily displaying the parts of the cross
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

IIROB_LED_Cross::IIROB_LED_Cross(ros::NodeHandle nodeHandle, std::string const& _port, int const& _mNumLeds, double _maxForce, std::string link)
    : IIROB_LED_Base::IIROB_LED_Base(nodeHandle, _port, _mNumLeds, _maxForce, MAX_NUM_LEDS_CROSS, link)
{
    // Initialize the tf2-related components
    buf = new tf2_ros::Buffer();
    tfl = new tf2_ros::TransformListener(*buf, nodeHandle);
}

IIROB_LED_Cross::~IIROB_LED_Cross() {
    delete buf;
    delete tfl;
}

// Callbacks for all action servers and subscribers
void IIROB_LED_Cross::forceCallback(const iirob_led::DirectionWithForce::ConstPtr& ledForceMsg) {

    ROS_INFO("[Before transformation] | Force: %.3f | x(%f), y(%f)",
             getVector2dLenght(ledForceMsg->force.wrench.force.x, ledForceMsg->force.wrench.force.y),
             ledForceMsg->force.wrench.force.x,
             ledForceMsg->force.wrench.force.y);

    // Process the transformation information
    geometry_msgs::Vector3Stamped forceIn, forceOut;
    forceIn.vector = ledForceMsg->force.wrench.force;
    geometry_msgs::TransformStamped tfStamped;
    ROS_INFO("Transfroming from \"%s\" to \"%s\" (local frame for cross)", ledForceMsg->force.header.frame_id.c_str(), localFrame.c_str());
    try
    {
        tfStamped = buf->lookupTransform(localFrame, ledForceMsg->force.header.frame_id, ros::Time(0));
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

    if(!x && !y)
    {
        ROS_WARN("Both x and y coordinates equal zero which does not allow visualization in 2D space.");
        return;
    }

    double force = getVector2dLenght(x, y);
    if(force > maxForce)
    {
        ROS_ERROR("Force %.3f (using rounded x and y) exceeds specified maximum force %.3f", force, maxForce);
        return;
    }

    ROS_INFO("[After transformation] Force: %.3f | x(%f), y(%f)", force, x, y);

    // Scale up/down the rounded x and y so that these two fit in the range [0..MAX_NUM_LEDS_CROSS].
    // [0] ----- [force] -- [max_force] => [0] ----- [force_scaled] -- [MAX_NUM_LEDS_CROSS]
    double xScaled = convert(_scalingFactor, std::fabs(x));
    double yScaled = convert(_scalingFactor, std::fabs(y));

    // We get the remainders using the scaled x and y values. The remainder is use for the V (in HSV) value of the last LED on each axis (x and y)
    double xScaledRemainder = fmod(xScaled, 1);
    double yScaledRemainder = fmod(yScaled, 1);

    ROS_INFO("Scaled: x(%f), y(%f) | Remainders: x(%f), y(%f)", xScaled, yScaled, xScaledRemainder, yScaledRemainder);

    float r = ledForceMsg->color.r, rX, rY;
    float g = ledForceMsg->color.g, gX, gY;
    float b = ledForceMsg->color.b, bX, bY;
    float h, s, v, vX, vY;

    // Convert to HSV
    RGBConverter::rgbToHsv(r, g, b, &h, &s, &v);
    // Use the remainders for the V (in HSV) of their corresponding axes
    vX = xScaledRemainder;
    vY = yScaledRemainder;
    // Convert back to RGB
    RGBConverter::hsvToRgb(h, s, vX, &rX, &gX, &bX);
    RGBConverter::hsvToRgb(h, s, vY, &rY, &gY, &bY);

    ROS_DEBUG("X: rbg[%f, %f, %f] -> hsv[%f, %f, %f] -> set V -> hsv[%f, %f, %f] -> rgb[%f, %f, %f]",
             r, g, b,
             h, s, v,
             h, s, vX,
             rX, gX, bX);
    ROS_DEBUG("Y: rbg[%f, %f, %f] -> hsv[%f, %f, %f] -> set V -> hsv[%f, %f, %f] -> rgb[%f, %f, %f]",
             r, g, b,
             h, s, v,
             h, s, vY,
             rY, gY, bY);

    mLed->setAllRGBf(0, 0, 0, mNumLeds);

    int xScaledRoundedInt = std::fabs(round(xScaled));
    int yScaledRoundedInt = std::fabs(round(yScaled));

    ROS_INFO("Scale rounded int: x=%d, y=%d", xScaledRoundedInt, yScaledRoundedInt);

    // If we have rounded down we need to add +1 so that we can display values such as 0.6, 0.3 etc.
    if((xScaled - xScaledRoundedInt) > 0.) xScaledRoundedInt++;
    if((yScaled - yScaledRoundedInt) > 0.) yScaledRoundedInt++;

    ROS_INFO("Scale rounded int: x=%d, y=%d", xScaledRoundedInt, yScaledRoundedInt);

    /*if(!xScaledRoundedInt && !yScaledRoundedInt)
    {
        ROS_WARN("Scaling resulted in both x and y coordinates equal zero which does not allow visualization in 2D space.");
        return;
    }*/

    //if(xScaledRemainder > 0. && (std::fabs(xScaledRoundedInt))) xScaledRoundedInt++;
    //if(yScaledRemainder > 0. && !yScaledRoundedInt) yScaledRoundedInt++;

    /*if(xScaledRoundedInt < 1 && yScaledRoundedInt < 1)
    {
        mLed->setRangeRGBf(r, g, b, mNumLeds, CROSS_CENTER, CROSS_CENTER);
        return;
    }*/

    ROS_INFO("Scaled and abs: x=%.3f, y=%.3f | Rounded and abs: x=%d, y=%d", xScaled, yScaled, xScaledRoundedInt, yScaledRoundedInt);

    // Check if the rounded value is greater than the original scaled one. This basically determines whether we have rounded up or down. If we have rounded down, we need to add +1 (2.3 -> (rounded down: 2 | remainder: 0.3) -> (++: 3) | remainder: 0.3)
    //if(((double)xScaledRoundedInt - std::fabs(xScaled)) < 0) xScaledRoundedInt++;
    //if(((double)yScaledRoundedInt - std::fabs(yScaled)) < 0) yScaledRoundedInt++;

    //ROS_INFO("AFTER round check | Rounded and abs: x=%d, y=%d", xScaled, yScaled, xScaledRoundedInt, yScaledRoundedInt);

    //ROS_INFO("X: (%d - %f) = %f", xScaledRoundedInt, std::fabs(xScaled), (double)xScaledRoundedInt - std::fabs(xScaled));
    //ROS_INFO("Y: (%d - %f) = %f", yScaledRoundedInt, std::fabs(yScaled), (double)yScaledRoundedInt - std::fabs(yScaled));

    if(x > 0 && y > 0 && xScaledRoundedInt != 0 && yScaledRoundedInt != 0)
    {
        ROS_INFO("x > 0 && y > 0");
        mLed->setRangeRGBf(r, g, b, mNumLeds, H_LEFT_XPLUS_START, H_LEFT_XPLUS_START - 1 + xScaledRoundedInt, true, true, false);                                           // +x
        mLed->setRangeRGBf(rX, gX, bX, mNumLeds, H_LEFT_XPLUS_START + xScaledRoundedInt - 1, H_LEFT_XPLUS_START + xScaledRoundedInt - 1, true, true, false);                          // +x | LAST
        mLed->setRangeRGBf(r, g, b, mNumLeds, V_UPPER_YPLUS_START + (V_SIDE - yScaledRoundedInt), V_UPPER_YPLUS_END, true, true, false);                                    // +y
        mLed->setRangeRGBf(rY, gY, bY, mNumLeds, V_UPPER_YPLUS_START + (V_SIDE - yScaledRoundedInt), V_UPPER_YPLUS_START + (V_SIDE - yScaledRoundedInt), true, true, false);          // +y | LAST
    }
    else if(x > 0 && y < 0 && xScaledRoundedInt != 0 && yScaledRoundedInt != 0)
    {
        ROS_INFO("x > 0 && y < 0");
        mLed->setRangeRGBf(r, g, b, mNumLeds, H_LEFT_XPLUS_START, H_LEFT_XPLUS_START - 1 + xScaledRoundedInt, true, true, false);                                           // +x
        mLed->setRangeRGBf(rX, gX, bX, mNumLeds, H_LEFT_XPLUS_START - 1 + xScaledRoundedInt, H_LEFT_XPLUS_START - 1 + xScaledRoundedInt, true, true, false);                          // +x | LAST
        mLed->setRangeRGBf(r, g, b, mNumLeds, V_BOTTOM_YMINUS_START, V_BOTTOM_YMINUS_START - 1 + yScaledRoundedInt, true, true, false);                                     // -y
        mLed->setRangeRGBf(rY, gY, bY, mNumLeds, V_BOTTOM_YMINUS_START - 1 + yScaledRoundedInt, V_BOTTOM_YMINUS_START - 1 + yScaledRoundedInt, true, true, false);                    // -y | LAST
    }
    else if(x < 0 && y > 0 && xScaledRoundedInt != 0 && yScaledRoundedInt != 0)
    {
        ROS_INFO("x < 0 && y > 0");
        mLed->setRangeRGBf(r, g, b, mNumLeds, H_RIGHT_XMINUS_START + (H_SIDE - xScaledRoundedInt), H_RIGHT_XMINUS_END, true, true, false);                                  // -x
        mLed->setRangeRGBf(rX, gX, bX, mNumLeds, H_RIGHT_XMINUS_START + (H_SIDE - xScaledRoundedInt), H_RIGHT_XMINUS_START + (H_SIDE - xScaledRoundedInt), true, true, false);        // -x | LAST
        mLed->setRangeRGBf(r, g, b, mNumLeds, V_UPPER_YPLUS_START + (V_SIDE - yScaledRoundedInt), V_UPPER_YPLUS_END, true, true, false);                                    // +y
        mLed->setRangeRGBf(rY, gY, bY, mNumLeds, V_UPPER_YPLUS_START + (V_SIDE - yScaledRoundedInt), V_UPPER_YPLUS_START + (V_SIDE - yScaledRoundedInt), true, true, false);          // +y | LAST
    }
    else if(x < 0 && y < 0 && xScaledRoundedInt != 0 && yScaledRoundedInt != 0)
    {
        ROS_INFO("x < 0 && y < 0");
        mLed->setRangeRGBf(r, g, b, mNumLeds, H_RIGHT_XMINUS_START + (H_SIDE - xScaledRoundedInt), H_RIGHT_XMINUS_END, true, true, false);                                  // -x
        mLed->setRangeRGBf(rX, gX, bX, mNumLeds, H_RIGHT_XMINUS_START + (H_SIDE - xScaledRoundedInt), H_RIGHT_XMINUS_START + (H_SIDE - xScaledRoundedInt), true, true, false);        // -x | LAST
        mLed->setRangeRGBf(r, g, b, mNumLeds, V_BOTTOM_YMINUS_START, V_BOTTOM_YMINUS_START - 1 + yScaledRoundedInt, true, true, false);                                     // -y
        mLed->setRangeRGBf(rY, gY, bY, mNumLeds, V_BOTTOM_YMINUS_START - 1 + yScaledRoundedInt, V_BOTTOM_YMINUS_START - 1 + yScaledRoundedInt, true, true, false);                    // -y | LAST
    }
    else
    {
        ROS_INFO("Single axis");
        // Single axis
        // x == 0
        if(!x && y < 0. && yScaledRoundedInt)
        {
            ROS_INFO("x == 0 && y < 0");
            mLed->setRangeRGBf(r, g, b, mNumLeds, V_BOTTOM_YMINUS_START, V_BOTTOM_YMINUS_START - 1 + yScaledRoundedInt, true, true, false);
            mLed->setRangeRGBf(rY, gY, bY, mNumLeds, V_BOTTOM_YMINUS_START - 1 + yScaledRoundedInt, V_BOTTOM_YMINUS_START - 1 + yScaledRoundedInt, true, true, false);
        }
        else if(!x && y > 0. && yScaledRoundedInt)
        {
            ROS_INFO("x == 0 && y > 0");
            mLed->setRangeRGBf(r, g, b, mNumLeds, V_UPPER_YPLUS_START + (V_SIDE - yScaledRoundedInt), V_UPPER_YPLUS_END, true, true, false);
            mLed->setRangeRGBf(rY, gY, bY, mNumLeds, V_UPPER_YPLUS_START + (V_SIDE - yScaledRoundedInt), V_UPPER_YPLUS_START + (V_SIDE - yScaledRoundedInt), true, true, false);
        }
        // y == 0
        else if(!y && x > 0. && xScaledRoundedInt)
        {
            ROS_INFO("y == 0 && x > 0");
            mLed->setRangeRGBf(r, g, b, mNumLeds, H_LEFT_XPLUS_START, H_LEFT_XPLUS_START - 1 + xScaledRoundedInt, true, true, false);
            mLed->setRangeRGBf(rX, gX, bX, mNumLeds, H_LEFT_XPLUS_START - 1 + xScaledRoundedInt, H_LEFT_XPLUS_START - 1 + xScaledRoundedInt, true, true, false);
        }
        else if(!y && x < 0. && xScaledRoundedInt)
        {
            ROS_INFO("y == 0 && x < 0");
            mLed->setRangeRGBf(r, g, b, mNumLeds, H_RIGHT_XMINUS_START + (H_SIDE - xScaledRoundedInt), H_RIGHT_XMINUS_END, true, true, false);
            mLed->setRangeRGBf(rX, gX, bX, mNumLeds, H_RIGHT_XMINUS_START + (H_SIDE - xScaledRoundedInt), H_RIGHT_XMINUS_START + (H_SIDE - xScaledRoundedInt), true, true, false);
        }
    }


    // Light up the center
    mLed->setRangeRGBf(0, 0, 1, mNumLeds, CROSS_CENTER, CROSS_CENTER);
}

void IIROB_LED_Cross::policeCallback(const iirob_led::PoliceGoal::ConstPtr& goal) {
    iirob_led::PoliceFeedback feedback;
    iirob_led::PoliceResult result;
    int blinks_left = goal->blinks;
    int fast_blinks_left = goal->fast_blinks;
    //double fast_duration = goal;
    // NOTE: We ignore the start, end and number of LEDs and hardcode the behaviour for this action

    int j;
    for(int i = 0; i < goal->blinks; ++i, --blinks_left)
    {
        ROS_INFO(" -- Long blinks left: %d", blinks_left);
        for(j = 0; j < goal->fast_blinks; ++j, --fast_blinks_left)
        {
            ROS_INFO(" -- Short blinks left: %d", fast_blinks_left);
            // Blink the outer subsections
            mLed->setRangeRGBf(goal->color_outer.r, goal->color_outer.g, goal->color_outer.b, mNumLeds, V_UPPER_YPLUS_START, V_UPPER_YPLUS_END, true, true, false);
            mLed->setRangeRGBf(goal->color_outer.r, goal->color_outer.g, goal->color_outer.b, mNumLeds, V_BOTTOM_YMINUS_START, V_BOTTOM_YMINUS_END);
            ros::Duration(0.25).sleep();

            mLed->setRangeRGBf(0, 0, 0, mNumLeds, V_UPPER_YPLUS_START, V_UPPER_YPLUS_END, true, true, false);
            mLed->setRangeRGBf(0, 0, 0, mNumLeds, V_BOTTOM_YMINUS_START, V_BOTTOM_YMINUS_END);
            ros::Duration(0.05).sleep();

            // Blink the inner subsections
            mLed->setRangeRGBf(goal->color_inner.r, goal->color_inner.g, goal->color_inner.b, mNumLeds, H_RIGHT_XMINUS_START, H_RIGHT_XMINUS_END, true, true, false);
            mLed->setRangeRGBf(goal->color_inner.r, goal->color_inner.g, goal->color_inner.b, mNumLeds, H_LEFT_XPLUS_START, H_LEFT_XPLUS_END);
            ros::Duration(0.25).sleep();

            mLed->setRangeRGBf(0, 0, 0, mNumLeds, H_RIGHT_XMINUS_START, H_RIGHT_XMINUS_END, true, true, false);
            mLed->setRangeRGBf(0, 0, 0, mNumLeds, H_LEFT_XPLUS_START, H_LEFT_XPLUS_END);
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
