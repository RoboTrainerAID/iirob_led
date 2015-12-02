//#include <cmath>
#include <math.h>
#include "iirob_led_rectangle.h"
#include "iirob_led_base.h"
#include "iirob_led_math.h"
#include "RGBConverter.h"

IIROB_LED_Rectangle::IIROB_LED_Rectangle(ros::NodeHandle nodeHandle, std::string const& _port, int const& _m_numLeds, std::string link, double _max_force)
    : policeAS(nodeHandle, "police", boost::bind(&IIROB_LED_Rectangle::policeCallback, this, _1), false),
      local_frame(link),
      max_force(_max_force),
      IIROB_LED_Base::IIROB_LED_Base(nodeHandle, _port, _m_numLeds)
{
    policeAS.start();
    ROS_INFO("police action server started");
    subForce = nodeHandle.subscribe("led_force_rectangle", 10, &IIROB_LED_Rectangle::forceCallback, this);
    ROS_INFO("led_force_rectangle subscriber started");
    pubForceTransformed = nodeHandle.advertise<iirob_led::DirectionWithForce>("led_force_rectangle_local", 1);
    ROS_INFO("led_force_rectangle_local publisher started");

    // Initialize the tf2-related components
    buf = new tf2_ros::Buffer();
    tfl = new tf2_ros::TransformListener(*buf, nodeHandle);
}

IIROB_LED_Rectangle::~IIROB_LED_Rectangle() {
    ROS_INFO("Shutting down police action server and led_force_rectangle subscriber");
    policeAS.shutdown();
    subForce.shutdown();
    pubForceTransformed.shutdown();

    delete buf;
    delete tfl;
}

// Callbacks for all action servers and subscribers
void IIROB_LED_Rectangle::forceCallback(const iirob_led::DirectionWithForce::ConstPtr& led_force_msg) {
    // Calculate the force
    double x = led_force_msg->force.x;
    double y = led_force_msg->force.y;

    if(x == 0 && y == 0)
    {
        ROS_WARN("Both x and y coordinates equal zero which does not allow visualization in 2D space.");
        return;
    }
    double force = sqrt(pow(led_force_msg->force.x, 2) + pow(led_force_msg->force.y, 2)); // + pow(led_force_msg->force.z, 2));
    // Scale the received force to be in the interval between 0 and maxForce (maxFroce rounded up and converted to an integer - it will represent the number of LEDs to be lit)
    // [0] ----- [force] -- [maxForce]
    int forceRounded = (int)round(force);
    // Round: (down) X.00...01 = X.49...99 = X | (up) X.50...00 = X.99...99 = X+1
    double forceRemainder = force - forceRounded; // Store the remainder (if any) which will be used as the V (in HSV) value of the last LED in the range of LEDs that display forceRounded

    if(forceRemainder > 0)  // Example: force = 2.3, forceRounded = 2 => forceRemainder = force - forceRounded = 2.3 - 2 = 0.3 => we have rounded DOWN the force
        forceRounded++;     // forceRounded++ = 2+1 = 3 => 3 LEDs with last LED using V (in HSV) = 0.3
    else if(forceRemainder < 0) // Example: force = 2.7, forceRounded = 3 => forceRemainder = force - forceRounded = 2.7 - 3 = -0.3 => we have rounded UP the force
        forceRemainder = 1 + forceRemainder; // forceRemainder = 1 + (-0.3) = 0.7 => 3 LEDs with the last LED using V (in HSV) = 0.7

    //int forceRounded = (int)round(force);       // 1.5 becomes 2 but 1.3 becomes 1
    ROS_INFO("Max number of LEDs: %d", MAX_NUM_LEDS_RECTANGLE);
    ROS_INFO("Force: %.3f | Rounded and int: %d", force, forceRounded);
    if(forceRounded > max_force)
    {
        ROS_ERROR("Received force is of greater magnitude than the set upper bound or exceed the numer of LEDs that can be displayed on each side of the platform | forceRounded(%d), forceMax(%d)", forceRounded, MAX_NUM_LEDS_RECTANGLE);
        return;
    }

    /*
     * We combine the LED indexing (front right corner: LED[0] ... back right corner: LED[0+108] ... ... ... front right corner: LED[383])
     * and the degrees we get from the atan2 function
     * We observe two major cases here:
     * 1)one of the axes is equal to 0 - in this case we don't need to calculate atan2() and we can directly go to the LED indexing part (this can also be done for other scenarios too such as when we hit the corners of the rectangle; see the macros RECT_CORNER_XXX)
     * 2)both axes are unequal to 0 - in this case we have to calculate atan2()
     *
     * The way the platform currently moves is as follows:
     * -y : move to the right (the side where the kinect is mounted)
     * +y : move to the left
     * +x : move forwards (the side where the on/off key is)
     * -x : move backwards
     *
     * We define a coordinate system with its center being located in the center of the platform and
     * its first quadrant (+x,+y) being located at the fron
     *
     *               FRONT (0 DEG)
     *   [0],[383]    +x        ______SR2
     *         \  _____|_____  / __________unit circle
     *          \/     |     \/ /
     *          /|-----|-----|\/
     *         / |     |     | \
     *         | |     |     | |
     *LEFT +y__|_|_____0_____|_|__-y RIGHT
     *(+90 DEG,| |     |     | |     (+270 DEG, 90 DEG)
     *-270 DEG)| |     |     | |
     *         \ |     |     | /
     *          \|-----|-----|/
     *           \_____|_____/
     *                 |
     *                -x
     *               BACK
     *             (+180 DEG, 180 DEG)
     *
     */

    int direction;
    double angle;
    double ledPerDeg;
    double translationAlongStrip;
    bool singleAxes = ((!x  && y) || (x && !y));    // If true, then either x or y equal 0
    if(!singleAxes)
    {
        angle = atan2(y,x);  // Angle in radians
        ledPerDeg = m_numLeds/360.;
        translationAlongStrip = (angle*180./M_PI) * ledPerDeg;
        direction = (int)round(RECT_FRONT + translationAlongStrip) % m_numLeds;
    }
    else
    {
        if(x == 0  && y < 0) direction = RECT_RIGHT;
        else if(x == 0 && y > 0) direction = RECT_LEFT;
        else if(y == 0 && x > 0) direction = RECT_FRONT;
        else if(y == 0 && x < 0) direction = RECT_BACK;
    }

    ROS_INFO("XY coordinates: [%.3f , %.3f]\t|\tForce (upper limit of %d): %.3f", x, y, max_force, force);
    ROS_INFO("Led/Angle: %f | Angle: %frad (=%fdeg) | Translation (num of LEDs): %f | location (LED index): %d", ledPerDeg, angle, angle*180./M_PI, translationAlongStrip, direction);

    m_led->setAllRGBf(0, 0, 0, m_numLeds);

    if(forceRounded == 1)
    {
        float r = 0, g = 0, b = 1;  //TODO add colour control via the action itself

        if(forceRemainder != 0)
        {
            float h, s, v;
            RGBConverter::rgbToHsv(r, g, b, &h, &s, &v);
            v = forceRemainder; // Use the remainder as V
            RGBConverter::hsvToRgb(h, s, v, &r, &g, &b);
        }
        m_led->setRangeRGBf(r, g, b, m_numLeds, direction, direction);
        return;
    }

    forceRounded--;

    //TODO add colour control via the action itself
    float r = 1, g = 0, b = 0;
    m_led->setRangeRGBf(r, g, b, m_numLeds,
                        ((direction - forceRounded) > 0) ? (direction - forceRounded) : m_numLeds - (forceRounded - direction),
                        (direction + forceRounded),
                        true, true, false);
    if(forceRemainder != 0)
    {
        float h, s, v;
        RGBConverter::rgbToHsv(r, g, b, &h, &s, &v);
        v = forceRemainder; // Use the remainder as V
        RGBConverter::hsvToRgb(h, s, v, &r, &g, &b);
        // First and last LED in the range of LEDs use the fourceRemainder as their V (in HSV)
        m_led->setRangeRGBf(r, g, b, m_numLeds,
                            ((direction - forceRounded) > 0) ? (direction - forceRounded) : m_numLeds - (forceRounded - direction),
                            ((direction - forceRounded) > 0) ? (direction - forceRounded) : m_numLeds - (forceRounded - direction));
        m_led->setRangeRGBf(r, g, b, m_numLeds,
                            (direction + forceRounded),
                            (direction + forceRounded));
    }

    m_led->setRangeRGBf(0, 0, 1, m_numLeds, direction, direction);

    // Process the transformation information
    geometry_msgs::Vector3Stamped forceIn, forceOut;
    forceIn.vector = led_force_msg->force;
    geometry_msgs::TransformStamped tf_stamped;
    iirob_led::DirectionWithForce led_force_msg_transformed;
    try
    {
        tf_stamped = buf->lookupTransform(local_frame, led_force_msg_transformed.tf_stamped.header.frame_id, ros::Time(0));
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
    led_force_msg_transformed.tf_stamped.header.frame_id = local_frame;
    led_force_msg_transformed.tf_stamped.transform = tf_stamped.transform;
    led_force_msg_transformed.force = forceOut.vector;
    pubForceTransformed.publish(led_force_msg_transformed);
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
            m_led->setRangeRGBf(goal->color_outer.r, goal->color_outer.g, goal->color_outer.b, m_numLeds, start_led_left_outer, end_led_left_outer, true, true, false);
            m_led->setRangeRGBf(goal->color_outer.r, goal->color_outer.g, goal->color_outer.b, m_numLeds, start_led_right_outer, end_led_right_outer);
            ros::Duration(0.5).sleep();

            m_led->setRangeRGBf(0, 0, 0, m_numLeds, start_led_left_outer, end_led_left_outer, true, true, false);
            m_led->setRangeRGBf(0, 0, 0, m_numLeds, start_led_right_outer, end_led_right_outer);
            ros::Duration(0.05).sleep();

            // Blink the inner subsections
            m_led->setRangeRGBf(goal->color_inner.r, goal->color_inner.g, goal->color_inner.b, m_numLeds, start_led_left_inner, end_led_left_inner, true, true, false);
            m_led->setRangeRGBf(goal->color_inner.r, goal->color_inner.g, goal->color_inner.b, m_numLeds, start_led_right_inner, end_led_right_inner);
            //ros::Duration(goal->fast_duration_on).sleep();
            ros::Duration(0.5).sleep();

            m_led->setRangeRGBf(0, 0, 0, m_numLeds, start_led_left_inner, end_led_left_inner, true, true, false);
            m_led->setRangeRGBf(0, 0, 0, m_numLeds, start_led_right_inner, end_led_right_inner);
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
