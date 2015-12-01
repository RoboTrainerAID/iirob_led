//#include <cmath>
#include "iirob_led_cross.h"
#include "iirob_led_math.h"
#include "RGBConverter.h"

// TODO Check if everywhere the duration parameter of a message is set properly (value > 0). Otherwise duration takes the default value from the message (=0), which equals infinity and node can only be stopped by escalating to SIGTERM

// CROSS TEST
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

IIROB_LED_Cross::IIROB_LED_Cross(ros::NodeHandle nodeHandle, std::string const& _port, int const& _m_numLeds, std::string link, double _max_force)
    : policeAS(nodeHandle, "police", boost::bind(&IIROB_LED_Cross::policeCallback, this, _1), false),
      localFrame(link),
      maxForce(_max_force),
      IIROB_LED_Base::IIROB_LED_Base(nodeHandle, _port, _m_numLeds)
{
    policeAS.start();
    ROS_INFO("police action server started");
    ROS_INFO("led_force_cross subscriber started");
    subForce = nodeHandle.subscribe("led_force_cross", 10, &IIROB_LED_Cross::forceCallback, this);
    pubForceTransformed = nodeHandle.advertise<iirob_led::DirectionWithForce>("led_force_cross_local", 1);
    ROS_INFO("led_force_cross_local publisher started");

    // Initialize the tf2-related components
    buf = new tf2_ros::Buffer();
    tfl = new tf2_ros::TransformListener(*buf, nodeHandle);
}

IIROB_LED_Cross::~IIROB_LED_Cross() {
    ROS_INFO("Shutting down police action server and led_force_cross subscriber");
    policeAS.shutdown();
    subForce.shutdown();

    delete buf;
    delete tfl;
}

// Callbacks for all action servers and subscribers
void IIROB_LED_Cross::forceCallback(const iirob_led::DirectionWithForce::ConstPtr& led_force_msg) {

    // Calculate the force
    double x = led_force_msg->force.x;
    double y = led_force_msg->force.y;

    if(x == 0 && y == 0)
    {
        ROS_WARN("Both x and y coordinates equal zero which does not allow visualization in 2D space.");
        return;
    }

    /*// First we calculate the force
    double force = sqrt(pow(led_force_msg->force.x, 2) + pow(led_force_msg->force.y, 2));// + pow(led_force_msg->force.z, 2));
    // Next we scale up/down the given force by converting it from the range [0..max_force] to the range [0..MAX_NUM_LEDS_CROSS]
    // [0] ----- [force] -- [max_force] => [0] ----- [force_scaled] -- [MAX_NUM_LEDS_CROSS]
    double forceScaled = convert(&scalingFactor, 0, max_force, 0, MAX_NUM_LEDS_CROSS, force);
    ROS_INFO("Scaling force %.3f from [0..%.3f] to [0..%d] : %.3f ", force, max_force, MAX_NUM_LEDS_CROSS, forceScaled);

    // Third we calculate the remainder using the scaled force; note that forceRounded represent the number of LEDs that will be lit
    int forceRounded = round(forceScaled);
    // Round: (down) X.00...01 = X.49...99 = X | (up) X.50...00 = X.99...99 = X+1
    ROS_INFO("Rounding scaled force %.3f and converting to int: %d", forceScaled, forceRounded);

    // Store the remainder (if any) which will be used as the V (in HSV) value of the last LED in the range of LEDs that display forceRounded
    double forceRemainder = forceScaled - forceRounded;

    // We need to check whether the remainder is > or < 0; the case when it's equal to 0 is not interesting
    if(forceRemainder > 0)  // Example: force = 2.3, forceRounded = 2 => forceRemainder = forceScaled - forceRounded = 2.3 - 2 = 0.3 => we have rounded DOWN forceScaled
        forceRounded++;     // forceRounded++ = 2+1 = 3 => 3 LEDs with last LED using V (in HSV) = 0.3
    else if(forceRemainder < 0) // Example: force = 2.7, forceRounded = 3 => forceRemainder = forceScaled - forceRounded = 2.7 - 3 = -0.3 => we have rounded UP forceScaled
        forceRemainder = 1 + forceRemainder; // forceRemainder = 1 + (-0.3) = 0.7 => 3 LEDs with the last LED using V (in HSV) = 0.7

    if(forceRounded > MAX_NUM_LEDS_CROSS)
    {
        ROS_ERROR("Received force is of greater magnitude than the set upper bound or exceed the numer of LEDs that can be displayed on each side of the platform | forceRounded(%d), forceMax(%d)", forceRounded, MAX_NUM_LEDS_CROSS);
        return;
    }

    m_led->setAllRGBf(0, 0, 0, m_numLeds);

    bool singleAxes = ((!x  && y) || (x && !y));    // If true, then either x or y equal 0
    if(forceRounded == 1)
    {
        m_led->setRangeRGBf(0, 0, 1, m_numLeds, CROSS_CENTER, CROSS_CENTER);
        return;
    }

    forceRounded--;

    if(!singleAxes)
    {
        if(x > 0 && y > 0)
        {
            m_led->setRangeRGBf(1, 0, 0, m_numLeds, H_LEFT_XPLUS_START, H_LEFT_XPLUS_START - 1 + forceRounded, true, true, false);  // +x
            m_led->setRangeRGBf(1, 0, 0, m_numLeds, V_UPPER_YPLUS_START + (V_SIDE - forceRounded), V_UPPER_YPLUS_END);              // +y
        }
        else if(x > 0 && y < 0)
        {
            m_led->setRangeRGBf(1, 0, 0, m_numLeds, H_LEFT_XPLUS_START, H_LEFT_XPLUS_START - 1 + forceRounded, true, true, false);  // +x
            m_led->setRangeRGBf(1, 0, 0, m_numLeds, V_BOTTOM_YMINUS_START, V_BOTTOM_YMINUS_START - 1 + forceRounded);               // -y
        }
        else if(x < 0 && y > 0)
        {
            m_led->setRangeRGBf(1, 0, 0, m_numLeds, H_RIGHT_XMINUS_START + (H_SIDE - forceRounded), H_RIGHT_XMINUS_END, true, true, false); // -x
            m_led->setRangeRGBf(1, 0, 0, m_numLeds, V_UPPER_YPLUS_START + (V_SIDE - forceRounded), V_UPPER_YPLUS_END);                      // +y
        }
        else if(x < 0 && y < 0)
        {
            m_led->setRangeRGBf(1, 0, 0, m_numLeds, H_RIGHT_XMINUS_START + (H_SIDE - forceRounded), H_RIGHT_XMINUS_END, true, true, false); // -x
            m_led->setRangeRGBf(1, 0, 0, m_numLeds, V_BOTTOM_YMINUS_START, V_BOTTOM_YMINUS_START - 1 + forceRounded);                       // -y
        }
    }
    else
    {
        // TODO Change this to show the X,Y position + add scaling (0..max_force -> 0..#LEDs)
        if(x == 0 && y < 0) m_led->setRangeRGBf(1, 0, 0, m_numLeds, V_BOTTOM_YMINUS_START, V_BOTTOM_YMINUS_START - 1 + forceRounded);
        else if(x == 0 && y > 0) m_led->setRangeRGBf(1, 0, 0, m_numLeds, V_UPPER_YPLUS_START + (V_SIDE - forceRounded), V_UPPER_YPLUS_END);
        else if(y == 0 && x > 0) m_led->setRangeRGBf(1, 0, 0, m_numLeds, H_LEFT_XPLUS_START, H_LEFT_XPLUS_START - 1 + forceRounded);
        else if(y == 0 && x < 0) m_led->setRangeRGBf(1, 0, 0, m_numLeds, H_RIGHT_XMINUS_START + (H_SIDE - forceRounded), H_RIGHT_XMINUS_END);
    }

    m_led->setRangeRGBf(0, 0, 1, m_numLeds, CROSS_CENTER, CROSS_CENTER);*/

/*    // Round the original x and y
    // Round: (down) X.00...01 = X.49...99 = X | (up) X.50...00 = X.99...99 = X+1
    double xRounded = round(x);
    double yRounded = round(y);

    // If remainder > 0 that means that we have rounded DOWN so we need to increase by 1 (we can't have for example 1/3 LED ;))
    if((x - xRounded) > 0) xRounded++;
    if((y - yRounded) > 0) yRounded++;*/

    double force = getVector2dLenght(x, y);
    if(force > maxForce)
    {
        ROS_ERROR("Force %.3f (using rounded x and y) exceeds specified maximum force %.3f", force, maxForce);
        return;
    }

    ROS_INFO("Force: %.3f | x(%f), y(%f)", force, x, y);

    // Scale up/down the rounded x and y so that these two fit in the range [0..MAX_NUM_LEDS_CROSS].
    // [0] ----- [force] -- [max_force] => [0] ----- [force_scaled] -- [MAX_NUM_LEDS_CROSS]
    double xScaled = convert(&scalingFactor, 0, maxForce, 0, MAX_NUM_LEDS_CROSS, abs(x));
    double yScaled = convert(&scalingFactor, 0, maxForce, 0, MAX_NUM_LEDS_CROSS, abs(y));

    // We get the remainders using the scaled x and y values. The remainder is use for the V (in HSV) value of the last LED on each axis (x and y)
    double xScaledRemainder = fmod(xScaled, 1);//getRemainder(xScaled);
    double yScaledRemainder = fmod(yScaled, 1);//getRemainder(yScaled);
    // TODO See why getRemainder() returns 0

    ROS_INFO(" ### Scaled: x(%f), y(%f) | Remainders: x(%f), y(%f)", xScaled, yScaled, xScaledRemainder, yScaledRemainder);

    if((xScaled - xScaledRemainder) > 0) xScaled++;
    if((yScaled - yScaledRemainder) > 0) yScaled++;

    float r = 1.0, rX, rY;
    float g = 0.0, gX, gY;
    float b = 0.0, bX, bY;
    float h, s, v, vX, vY;

    // Convert to HSV
    RGBConverter::rgbToHsv(r, g, b, &h, &s, &v);
    // Use the remainders for the V (in HSV) of their corresponding axes
    vX = xScaledRemainder;
    vY = yScaledRemainder;
    // Convert back to RGB
    RGBConverter::hsvToRgb(h, s, vX, &rX, &gX, &bX);
    RGBConverter::hsvToRgb(h, s, vY, &rY, &gY, &bY);

    ROS_INFO(" @@@ X: rbg[%f, %f, %f] -> hsv[%f, %f, %f] -> set V -> hsv[%f, %f, %f] -> rgb[%f, %f, %f]",
             r, g, b,
             h, s, v,
             h, s, vX,
             rX, gX, bX);
    ROS_INFO(" @@@ Y: rbg[%f, %f, %f] -> hsv[%f, %f, %f] -> set V -> hsv[%f, %f, %f] -> rgb[%f, %f, %f]",
             r, g, b,
             h, s, v,
             h, s, vY,
             rY, gY, bY);

    m_led->setAllRGBf(0, 0, 0, m_numLeds);

    if(abs(x) <= 1 && abs(y) <= 1)
    {
        m_led->setRangeRGBf(r, g, b, m_numLeds, CROSS_CENTER, CROSS_CENTER);
        return;
    }

    int xScaledInt = abs(round(xScaled));
    int yScaledInt = abs(round(yScaled));

    ROS_INFO("Scaled: x=%.3f, y=%.3f | Rounded: x=%d, y=%d", xScaled, yScaled, xScaledInt, yScaledInt);

    // We exclude 1 LED on each axis since we also light up the central LED
    //xScaledInt--;
    //yScaledInt--;

    if(x > 0 && y > 0)
    {
        m_led->setRangeRGBf(r, g, b, m_numLeds, H_LEFT_XPLUS_START, H_LEFT_XPLUS_START - 1 + xScaledInt, true, true, false);                                           // +x
        m_led->setRangeRGBf(rX, gX, bX, m_numLeds, H_LEFT_XPLUS_START - 1 + xScaledInt, H_LEFT_XPLUS_START - 1 + xScaledInt, true, true, false);                          // +x | LAST
        m_led->setRangeRGBf(r, g, b, m_numLeds, V_UPPER_YPLUS_START + (V_SIDE - yScaledInt), V_UPPER_YPLUS_END, true, true, false);                                    // +y
        m_led->setRangeRGBf(rY, gY, bY, m_numLeds, V_UPPER_YPLUS_START + (V_SIDE - yScaledInt), V_UPPER_YPLUS_START + (V_SIDE - yScaledInt), true, true, false);          // +y | LAST
    }
    else if(x > 0 && y < 0)
    {
        m_led->setRangeRGBf(r, g, b, m_numLeds, H_LEFT_XPLUS_START, H_LEFT_XPLUS_START - 1 + xScaledInt, true, true, false);                                           // +x
        m_led->setRangeRGBf(rX, gX, bX, m_numLeds, H_LEFT_XPLUS_START - 1 + xScaledInt, H_LEFT_XPLUS_START - 1 + xScaledInt, true, true, false);                          // +x | LAST
        m_led->setRangeRGBf(r, g, b, m_numLeds, V_BOTTOM_YMINUS_START, V_BOTTOM_YMINUS_START - 1 + yScaledInt, true, true, false);                                     // -y
        m_led->setRangeRGBf(rY, gY, bY, m_numLeds, V_BOTTOM_YMINUS_START - 1 + yScaledInt, V_BOTTOM_YMINUS_START - 1 + yScaledInt, true, true, false);                    // -y | LAST
    }
    else if(x < 0 && y > 0)
    {
        m_led->setRangeRGBf(r, g, b, m_numLeds, H_RIGHT_XMINUS_START + (H_SIDE - xScaledInt), H_RIGHT_XMINUS_END, true, true, false);                                  // -x
        m_led->setRangeRGBf(rX, gX, bX, m_numLeds, H_RIGHT_XMINUS_START + (H_SIDE - xScaledInt), H_RIGHT_XMINUS_START + (H_SIDE - xScaledInt), true, true, false);        // -x | LAST
        m_led->setRangeRGBf(r, g, b, m_numLeds, V_UPPER_YPLUS_START + (V_SIDE - yScaledInt), V_UPPER_YPLUS_END, true, true, false);                                    // +y
        m_led->setRangeRGBf(rY, gY, bY, m_numLeds, V_UPPER_YPLUS_START + (V_SIDE - yScaledInt), V_UPPER_YPLUS_START + (V_SIDE - yScaledInt), true, true, false);          // +y | LAST
    }
    else if(x < 0 && y < 0)
    {
        m_led->setRangeRGBf(r, g, b, m_numLeds, H_RIGHT_XMINUS_START + (H_SIDE - xScaledInt), H_RIGHT_XMINUS_END, true, true, false);                                  // -x
        m_led->setRangeRGBf(rX, gX, bX, m_numLeds, H_RIGHT_XMINUS_START + (H_SIDE - xScaledInt), H_RIGHT_XMINUS_START + (H_SIDE - xScaledInt), true, true, false);        // -x | LAST
        m_led->setRangeRGBf(r, g, b, m_numLeds, V_BOTTOM_YMINUS_START, V_BOTTOM_YMINUS_START - 1 + yScaledInt, true, true, false);                                     // -y
        m_led->setRangeRGBf(rY, gY, bY, m_numLeds, V_BOTTOM_YMINUS_START - 1 + yScaledInt, V_BOTTOM_YMINUS_START - 1 + yScaledInt, true, true, false);                    // -y | LAST
    }
    else
    {
        // Single axis
        if(x == 0 && y < 0)
        {
            m_led->setRangeRGBf(r, g, b, m_numLeds, V_BOTTOM_YMINUS_START, V_BOTTOM_YMINUS_START - 1 + yScaledInt, true, true, false);
            m_led->setRangeRGBf(rY, gY, bY, m_numLeds, V_BOTTOM_YMINUS_START - 1 + yScaledInt, V_BOTTOM_YMINUS_START - 1 + yScaledInt, true, true, false);
        }
        else if(x == 0 && y > 0)
        {
            m_led->setRangeRGBf(r, g, b, m_numLeds, V_UPPER_YPLUS_START + (V_SIDE - yScaledInt), V_UPPER_YPLUS_END, true, true, false);
            m_led->setRangeRGBf(rY, gY, bY, m_numLeds, V_UPPER_YPLUS_START + (V_SIDE - yScaledInt), V_UPPER_YPLUS_START + (V_SIDE - yScaledInt), true, true, false);
        }
        else if(y == 0 && x > 0)
        {
            m_led->setRangeRGBf(r, g, b, m_numLeds, H_LEFT_XPLUS_START, H_LEFT_XPLUS_START - 1 + xScaledInt, true, true, false);
            m_led->setRangeRGBf(rX, gX, bX, m_numLeds, H_LEFT_XPLUS_START - 1 + xScaledInt, H_LEFT_XPLUS_START - 1 + xScaledInt, true, true, false);
        }
        else if(y == 0 && x < 0)
        {
            m_led->setRangeRGBf(r, g, b, m_numLeds, H_RIGHT_XMINUS_START + (H_SIDE - xScaledInt), H_RIGHT_XMINUS_END, true, true, false);
            m_led->setRangeRGBf(rX, gX, bX, m_numLeds, H_RIGHT_XMINUS_START + (H_SIDE - xScaledInt), H_RIGHT_XMINUS_START + (H_SIDE - xScaledInt), true, true, false);
        }
    }


    // Light up t

    // Light up the center
    m_led->setRangeRGBf(0., 0., 1., m_numLeds, CROSS_CENTER, CROSS_CENTER);

    // Process the transformation information
    geometry_msgs::Vector3Stamped forceIn, forceOut;
    forceIn.vector = led_force_msg->force;
    geometry_msgs::TransformStamped tf_stamped;
    iirob_led::DirectionWithForce led_force_msg_transformed;
    try
    {
        tf_stamped = buf->lookupTransform(localFrame, led_force_msg_transformed.tf_stamped.header.frame_id, ros::Time(0));
        tf2::doTransform(forceIn, forceOut, tf_stamped);
        ROS_INFO("Transforming vec(x: %f, y: %f, z: %f) with trans(x: %f, y: %f, z: %f), rot(x: %f, y: %f, z: %f)",
                 forceIn.vector.x, forceIn.vector.y, forceIn.vector.z,
                 tf_stamped.transform.translation.x, tf_stamped.transform.translation.y, tf_stamped.transform.translation.z,
                 tf_stamped.transform.rotation.x, tf_stamped.transform.rotation.y, tf_stamped.transform.rotation.z);
    }
    catch(tf2::TransformException ex)
    {
        ROS_WARN("%s", ex.what());
    }
    led_force_msg_transformed.tf_stamped.header = led_force_msg->tf_stamped.header;
    led_force_msg_transformed.tf_stamped.header.frame_id = localFrame;
    led_force_msg_transformed.tf_stamped.transform = tf_stamped.transform;
    led_force_msg_transformed.force = forceOut.vector;
    pubForceTransformed.publish(led_force_msg_transformed);
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
            m_led->setRangeRGBf(goal->color_outer.r, goal->color_outer.g, goal->color_outer.b, m_numLeds, V_UPPER_YPLUS_START, V_UPPER_YPLUS_END, true, true, false);
            m_led->setRangeRGBf(goal->color_outer.r, goal->color_outer.g, goal->color_outer.b, m_numLeds, V_BOTTOM_YMINUS_START, V_BOTTOM_YMINUS_END);
            ros::Duration(0.25).sleep();

            m_led->setRangeRGBf(0, 0, 0, m_numLeds, V_UPPER_YPLUS_START, V_UPPER_YPLUS_END, true, true, false);
            m_led->setRangeRGBf(0, 0, 0, m_numLeds, V_BOTTOM_YMINUS_START, V_BOTTOM_YMINUS_END);
            ros::Duration(0.05).sleep();

            // Blink the inner subsections
            m_led->setRangeRGBf(goal->color_inner.r, goal->color_inner.g, goal->color_inner.b, m_numLeds, H_RIGHT_XMINUS_START, H_RIGHT_XMINUS_END, true, true, false);
            m_led->setRangeRGBf(goal->color_inner.r, goal->color_inner.g, goal->color_inner.b, m_numLeds, H_LEFT_XPLUS_START, H_LEFT_XPLUS_END);
            ros::Duration(0.25).sleep();

            m_led->setRangeRGBf(0, 0, 0, m_numLeds, H_RIGHT_XMINUS_START, H_RIGHT_XMINUS_END, true, true, false);
            m_led->setRangeRGBf(0, 0, 0, m_numLeds, H_LEFT_XPLUS_START, H_LEFT_XPLUS_END);
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
