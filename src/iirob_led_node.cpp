#include <ros/ros.h>
#include <ros/console.h>
#include "actionlib/server/simple_action_server.h"

//#include "iirob_led/PlayLedAction.h"
#include "iirob_led/BlinkyAction.h"
#include "iirob_led/PoliceAction.h"
#include "iirob_led/FourMusketeersAction.h"
#include "iirob_led/RunningBunnyAction.h"
#include "iirob_led/ChangelingAction.h"
//#include "iirob_led/SpinnerAction.h"
#include "iirob_led/SetLedDirectory.h"

#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>

#include <cmath>

#include <RGBConverter.h>

// TODO override - when ready set to false
// TODO directoryCallback, set_led_directory - replace this with a vector (btw it's DIRECTION not directory -_-)
// TODO Check return value for all iirob::hardware functions (setAllRGBf(), setRangeRGBf() etc.)
// TODO Add a duration parameter (number of steps per cycle should be determined from it) to Changeling (just add two modes)
// TODO All the checks if start_led or end_led are negative or not can be avoided by simply using unsigned chars (as far as I can tell internally the LED library is actually using this type)
// TODO Reimplement setRangeRGBX, setRangeRGB and setRangeRGBf. The way those three are working now is - to put it mildly - in a very poor manner.
// NOTE Except for color values (ColorRGBA requires float) use double (float64 in ROS) for more accuracy
// FIXME Currently (and how it was implemented by Ruben) if start_led > end_led things screw up. Easiest workaround is to swap the values if such scenario occurs. More complex solution (but better one)
//       is to split the given strap in 2 parts (see RunningBunny case where there is an issue when we transit from the last led (423 I think it was) to the first one (0))

#include "LEDStrip.h"

class LEDNode
{
    int m_msec;
    int m_numLeds;
    iirob_hardware::LEDStrip* m_led;
    int m_showDir;
    bool status;        // Contains the return result from init()
    std::string port;
    int num;

    ros::Subscriber subLedCommand;
    ros::Subscriber subChosenDir;
    ros::Subscriber subLedColor;

    ros::ServiceServer set_led_directory_srv;

    actionlib::SimpleActionServer<iirob_led::BlinkyAction> blinkyAS;
    actionlib::SimpleActionServer<iirob_led::PoliceAction> policeAS;
    actionlib::SimpleActionServer<iirob_led::FourMusketeersAction> fourMusketeersAS;
    actionlib::SimpleActionServer<iirob_led::RunningBunnyAction> runningBunnyAS;
    actionlib::SimpleActionServer<iirob_led::ChangelingAction> changelingAS;
    //actionlib::SimpleActionServer<iirob_led::SpinnerAction> spinnerAS;

    bool init(std::string const & port, int const & num) {
        m_led = new iirob_hardware::LEDStrip(port);
        if (m_led->ready()) {
            m_numLeds = num;
            m_led->setAllRGBf(0, 0, 0, m_numLeds);
            m_led->setUniRGB(0, 0, 0);
            //m_showDir = 4;

            ROS_INFO("%s: IIROB-LED action server started", ros::this_node::getName().c_str());
            ROS_INFO("Number of LEDs: %d", m_numLeds);

            blinkyAS.start();
            ROS_INFO("blinky");
            policeAS.start();
            ROS_INFO("police");
            fourMusketeersAS.start();
            ROS_INFO("fourMusketeers");
            runningBunnyAS.start();
            ROS_INFO("runningBunny");
            changelingAS.start();
            ROS_INFO("changeling");

            return true;
        }
        return false;
    }
public:
	//! Constructor.
    LEDNode(ros::NodeHandle nodeHandle, std::string const& _port, int const& _num)
        : m_led(0), m_numLeds(423), m_msec(1),
          port(_port), num(_num),
          blinkyAS(nodeHandle, "blinky", boost::bind(&LEDNode::blinkyCallback, this, _1), false),
          policeAS(nodeHandle, "police", boost::bind(&LEDNode::policeCallback, this, _1), false),
          fourMusketeersAS(nodeHandle, "four_musketeers", boost::bind(&LEDNode::fourMusketeersCallback, this, _1), false),
          runningBunnyAS(nodeHandle, "running_bunny", boost::bind(&LEDNode::runningBunnyCallback, this, _1), false),
          changelingAS(nodeHandle, "changeling", boost::bind(&LEDNode::changelingCallback, this, _1), false)
          //spinnerAS(nodeHandle, "spinner", boost::bind(&LEDNode::spinnerCallback, this, _1), false)
    {
        status = init(port, num);

        if(status)
        {
            subLedCommand = nodeHandle.subscribe("led_command", 10, &LEDNode::commandCallback, this);
            subChosenDir = nodeHandle.subscribe("chosen_directory", 10, &LEDNode::ledDirectionCallback, this);
            subLedColor = nodeHandle.subscribe("led_color", 10, &LEDNode::colorCallback, this);

            set_led_directory_srv = nodeHandle.advertiseService("set_led_directory", &LEDNode::setLedDirection, this);
            ROS_INFO("%s: IIROB-LED directory set to %s", ros::this_node::getName().c_str(), set_led_directory_srv.getService().c_str());
        }
    }

	//! Destructor.
	~LEDNode(){
        ROS_INFO("Turning all LEDs off");
		//turn everything off again
        if (m_led) {
			m_led->setAllRGBf(0, 0, 0, m_numLeds);
			m_led->setUniRGB(0, 0, 0);
			delete m_led;
		}

        ROS_INFO("Shutting down all action servers");
        blinkyAS.shutdown();
        policeAS.shutdown();
        fourMusketeersAS.shutdown();
        runningBunnyAS.shutdown();
        changelingAS.shutdown();
        //spinnerAS.shutdown();

        subLedCommand.shutdown();
        subChosenDir.shutdown();
        subLedColor.shutdown();

        set_led_directory_srv.shutdown();
    }

    bool getStatus() { return status; }

	//! Callback function for subscriber.
	void commandCallback(const std_msgs::String::ConstPtr& msg) {
		ROS_INFO("I heard: [%s]", msg->data.c_str());
		if (msg->data == "off") {
			m_led->setAllRGBf(0, 0, 0, m_numLeds);
		}
	}
	void colorCallback(const std_msgs::ColorRGBA::ConstPtr& msg) {
		ROS_INFO("I heard: [%f %f %f %f %d]", msg->r, msg->g, msg->b, msg->a, m_numLeds );
		ROS_INFO("[%d]", m_showDir);
		//Entscheidung welche LED-Richtung gezeigt wird
		switch(m_showDir) {
			case 0:
				m_led->setRangeRGBf(msg->r, msg->g, msg->b, m_numLeds, 0, 7);
				ROS_INFO("A");
				break;
			case 1:
				m_led->setRangeRGBf(msg->r, msg->g, msg->b, m_numLeds, 7, 14);
				ROS_INFO("B");
				break;
			case 2:
				m_led->setRangeRGBf(msg->r, msg->g, msg->b, m_numLeds, 14, 22);
				ROS_INFO("C");
				break;
			case 3:
				m_led->setRangeRGBf(msg->r, msg->g, msg->b, m_numLeds, 23, 31);
				ROS_INFO("D");
				break;
			case 4:
				m_led->setAllRGBf(msg->r, msg->g, msg->b, m_numLeds);
				ROS_INFO("F");
				break;
			default:
				m_led->setAllRGBf(msg->r, msg->g, msg->b, m_numLeds);
				ROS_INFO("Default");
				break;
		}
		//Auskommentiert, da Richtungsversion den Fall abdeckt
		//m_led->setAllRGBf(msg->r, msg->g, msg->b, m_numLeds);
	}

    void ledDirectionCallback(const std_msgs::String::ConstPtr& msg) {
        std::string tempMsg = msg->data.c_str();

        if (tempMsg == "r") {       // right
            m_showDir = 0;
        }
        else if (tempMsg == "l") {  // left
            m_showDir = 1;
        }
        else if (tempMsg == "b") {  // back
            m_showDir = 2;
        }
        else if (tempMsg == "f") {  // front
            m_showDir = 3;
        }
        else {
            //Default
            //m_showDir = 4;
        }

        /*
         * NOTE See what exactly "chosen_directory" is. From what I see above it seems that it only
         * contains single character strings and frankly it doesn't make much sense to use complete strings
         * if that's the case
         *
      void directoryCallback(const std_msgs::Char::ConstPtr& msg) {
        char tempMsg = msg->data;

        switch(tempMsg)
        {
        case 'r':
            m_showDir = 0;
            break;
        case 'l':
            m_showDir = 1;
            break;
        case 'b':
            m_showDir = 2;
            break;
        case 'f':
            m_showDir = 3;
            break;
        default:
            //m_showDir = 4;
            break;
        }
    }
        */
	}

    bool setLedDirection(iirob_led::SetLedDirectory::Request &req,
                           iirob_led::SetLedDirectory::Response &res)
    {
        ROS_INFO("I heard: [%f %f %f %f %d %d]",
                 req.color.r, req.color.g, req.color.b, req.color.a, req.start_led, req.end_led);
        m_led->setRangeRGBf(req.color.r, req.color.g, req.color.b, m_numLeds, req.start_led, req.end_led);
        return true;
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

        if(start_led < 0) start_led = 0;
        if(end_led >= m_numLeds) end_led = m_numLeds-1;

        ROS_DEBUG("%s starting Blinky action with color [%f %f %f] [RGB]. Duration(ON): %f sec, Duration(OFF): %f, start led: %d, end led: %d",
                 ros::this_node::getName().c_str(),
                 goal->color.r, goal->color.g, goal->color.b, goal->duration_on, goal->duration_off, start_led, end_led);

        for(int i = 0; i < goal->blinks; ++i, --blinks_left)
        {
            // Set selected LEDs and turn them on
            m_led->setRangeRGBf(goal->color.r, goal->color.g, goal->color.b, m_numLeds, start_led, end_led);
            ros::Duration(goal->duration_on).sleep();
            // Set selected LEDs and turn them off
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
    void fourMusketeersCallback(const iirob_led::FourMusketeersGoal::ConstPtr& goal) {
        iirob_led::FourMusketeersFeedback feedback;
        iirob_led::FourMusketeersResult result;
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
        int start_ledAthos = start_led;
        int end_ledAthos = start_ledAthos + half/2;

        int start_ledPorthos = end_ledAthos;
        int end_ledPorthos = start_ledPorthos + half/2;

        int start_ledAramis = end_ledPorthos;
        int end_ledAramis = start_ledAramis + half/2;

        int start_ledDArtagnan = end_ledAramis;
        int end_ledDArtagnan = end_led;

        ROS_INFO("%s starting Four Musketeers action with duration(ON): %.2f sec and duration(OFF): %.2f\n\n\t Athos\t\tstart led: %d, end led: %d\tcolor: %.2f %.2f %.2f \n\t Porthos\tstart led: %d, end led: %d\tcolor: %.2f %.2f %.2f \n\t Aramis\t\tstart led: %d, end led: %d\tcolor: %.2f %.2f %.2f \n\t DArtagnan\tstart led: %d, end led: %d\tcolor: %.2f %.2f %.2f ",
                 ros::this_node::getName().c_str(),
                 goal->duration_on, goal->duration_off,
                 start_ledAthos, end_ledAthos,
                 goal->color1.r, goal->color1.g, goal->color1.b,
                 start_ledPorthos, end_ledPorthos,
                 goal->color2.r, goal->color2.g, goal->color2.b,
                 start_ledAramis, end_ledAramis,
                 goal->color3.r, goal->color3.g, goal->color3.b,
                 start_ledDArtagnan, end_ledDArtagnan,
                 goal->color4.r, goal->color4.g, goal->color4.b);

        for(int i = 0; i < goal->blinks; ++i, --blinks_left)
        {
            // Set selected LEDs and turn them on
            m_led->setRangeRGBf(goal->color1.r, goal->color1.g, goal->color1.b, m_numLeds, start_ledAthos, end_ledAthos);
            m_led->setRangeRGBf(goal->color2.r, goal->color2.g, goal->color2.b, m_numLeds, start_ledPorthos, end_ledPorthos);
            m_led->setRangeRGBf(goal->color3.r, goal->color3.g, goal->color3.b, m_numLeds, start_ledAramis, end_ledAramis);
            m_led->setRangeRGBf(goal->color4.r, goal->color4.g, goal->color4.b, m_numLeds, start_ledDArtagnan, end_ledDArtagnan);
            ros::Duration(goal->duration_on).sleep();
            // Set selected LEDs and turn them off
            m_led->setRangeRGBf(0, 0, 0, m_numLeds, start_ledAthos, end_ledAthos);
            m_led->setRangeRGBf(0, 0, 0, m_numLeds, start_ledPorthos, end_ledPorthos);
            m_led->setRangeRGBf(0, 0, 0, m_numLeds, start_ledAramis, end_ledAramis);
            m_led->setRangeRGBf(0, 0, 0, m_numLeds, start_ledDArtagnan, end_ledDArtagnan);
            ros::Duration(goal->duration_off).sleep();

            feedback.blinks_left = blinks_left;
            ROS_INFO("%s: Four Musketeers will blink %d more times", ros::this_node::getName().c_str(), feedback.blinks_left);
            fourMusketeersAS.publishFeedback(feedback);
        }

        feedback.blinks_left = blinks_left;
        ROS_INFO("%s: Four Musketeers will blink %d more times", ros::this_node::getName().c_str(), feedback.blinks_left);

        m_led->setAllRGBf(0, 0, 0, m_numLeds);
        result.blinks_left = blinks_left;
        fourMusketeersAS.setSucceeded(result);
    }

    void runningBunnyCallback(const iirob_led::RunningBunnyGoal::ConstPtr& goal) {
        iirob_led::RunningBunnyFeedback feedback;
        iirob_led::RunningBunnyResult result;

        int head = goal->head;
        int body = goal->body;
        float r = goal->color.r;
        float g = goal->color.g;
        float b = goal->color.b;
        int counter = 0;
        int circles = goal->num_circles;
        bool split = false;
        int _splitPos = 0;
        int _after_split = m_numLeds-1;
        int _head, _tail = head - body;
        int _tail_non_negative = (_tail < 0) ? _tail + m_numLeds : _tail;   //423-1 = 422, 423-2 = 421, 423-3 = 420 ...
        int _tail_non_negative_old = _tail_non_negative-1;


        printf("Starting position: (head[%d] - tail[%d])\n-----------------------------------------\n", head, (head - body) % m_numLeds);

        for(int circle = 0; circle < circles; circle++) {
            _head = head;
            _tail = head - body;
            //_tail_non_negative_old = _tail_non_negative;

            for(counter = 0; counter < m_numLeds; counter+=goal->skip_leds_per_step) {
                //_tail_non_negative_old = _tail_non_negative-1;

                if(_tail < 0) {
                    split = true;
                    _tail_non_negative = _tail + m_numLeds;
                } else {
                    split = false;
                    _tail_non_negative = _tail;
                }

                ROS_INFO("[%d] : (head[%d] - tail[%d])\tsplit %s", counter, _head, _tail_non_negative, split ? "+" : "-");

                if(split) {
                    ROS_INFO("\t\t\\_________(head[%d] - split[%d] | after_split[%d] - tail[%d])\n", _head, _splitPos, _after_split, _tail_non_negative);
                    //m_led->setRangeRGBf(r, g, b, m_numLeds, _tail_non_negative, _after_split);   // We finish the current LED stripe (x->422)
                    //m_led->setRangeRGBf(r, g, b, m_numLeds, _splitPos, _head);                   // Then we start the next (0->y)
                    ROS_INFO("Turning off LEDs %d to %d", _head+1, _tail_non_negative-1);
                    //m_led->setRangeRGBf(0, 0, 0, m_numLeds, _head+1, _tail_non_negative-1);      // And finally we turn "off" all (y+1->x-1) (I've noticed that when we overlap LEDs it screws up things)
                } else {
                    m_led->setRangeRGBf(r, g, b, m_numLeds, _tail_non_negative, _head);
                    ROS_INFO("Turning off LEDs %d to %d", _tail_non_negative_old, _tail_non_negative);
                    m_led->setRangeRGBf(0, 0, 0, m_numLeds, _tail_non_negative_old, _tail_non_negative);
                }

                feedback.current_head_pos = _head;
                runningBunnyAS.publishFeedback(feedback);

                _head = (_head + goal->skip_leds_per_step) % m_numLeds;
                _tail = _head - body;
                _tail_non_negative_old = _tail_non_negative-1;
            }

            ROS_INFO("circle %d completed", circle);
        }

        // Test movement for a single LED ("good" range)
        /*for(int i = 10; i < 200; i++) {
            m_led->setRangeRGBf(r, g, b, m_numLeds, i-1, i);
            m_led->setAllRGBf(0, 0, 0, m_numLeds);
        }*/

        m_led->setAllRGBf(0, 0, 0, m_numLeds);
        feedback.current_head_pos = head;
        runningBunnyAS.publishFeedback(feedback);
        result.current_head_pos = head;
        runningBunnyAS.setSucceeded(result);
    }

    /**
     * @brief runningBunnyCallback processes RunningBunnyGoal messages - a given stripe of LEDs (bunny) "jumps" in circle (a jump is defined as the number of LEDs that are skipped by the bunny); previous name: lightActionCallback_6 with argument std_msgs::Float32MultiArray::ConstPtr&
     * @param goal
     */
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
//    void runningBunnyCallback(const iirob_led::RunningBunnyGoal::ConstPtr& goal) {
//        // TODO Use modulo to create a neat transition between previous and next circle!
//        // TODO Change RGB values using the routine from Changeling
//        ROS_INFO("RUNNING BUNNY CALLBACK");
//        iirob_led::RunningBunnyFeedback feedback;
//        iirob_led::RunningBunnyResult result;
//        int circle_counter = goal->num_circles;
//        int head = goal->head;
//        int body = goal->body;
//        int jumpOver = goal->skip_leds_per_step;
//        float r = goal->color.r;
//        float g = goal->color.g;
//        float b = goal->color.b;

//        // The bunny has a minimal size of 2 LEDs
//        if(!head) head = 1;
//        if(!body) body = 1; // Bunny consists of a single LED
//        if(jumpOver <= 0) jumpOver = (int)body/2;

//        int tail = head - body;     // NOTE See bidirectional movement todo. This here will determine which way the bunny has to run

//        ROS_INFO("Head: %d\nBody: %d\nTail: %d", head, body, tail);
//        // We cut the bunny into two parts (as cruel as it may sound...). The is used when we do the 422->0 transition
//        // Split in two intervals - (headSplit_start : headSplit_end) splitPos (tailSplit_start : tailSplit_end) with split = LED at position 0

//        // Formular for determining the LED position where we do the split:
//        // if we reach 422->0 transtition activate split
//        // for each head++ incr

//        //ROS_INFO("Bunny split: head[%d - %d] | tail[%d - %d]", headSplit_start, headSplit_end, tailSplit_start, tailSplit_end);

//        m_led->setAllRGBf(0, 0, 0, m_numLeds);

//        for(int i = 0; i < circle_counter; ++i) {
//            int _head = head;
//            int _tail = _head - body;
//            bool activateSplit = false;
//            int headSplit_end = 0;
//            int tailSplit_start = m_numLeds-1;
//            int _tail_old = _tail;

//            ROS_INFO("Circle %d", circle_counter);

//            for(int distance = 0; distance < m_numLeds; distance+=jumpOver, _head = (_head + jumpOver) % m_numLeds) {
//                ROS_INFO("Bunny has ran %d out of %d LEDs so far", distance, m_numLeds-1);
//                _tail = _head - body;
//                ROS_INFO("Tail: %d", _tail+(m_numLeds-1));

//                // Whenever _tail is negative (meaning: head has passed the 0 position but the body is still behind the 0 mark)
//                if(_tail < 0) {
//                    activateSplit = true;
//                }

//                if(activateSplit) {
//                    ROS_INFO("Split active");
//                    _tail = _tail + m_numLeds;

//                    m_led->setRangeRGBf(r, g, b, m_numLeds, headSplit_end, _head);
//                    m_led->setRangeRGBf(0, 0, 0, m_numLeds, _head, _tail);
//                    m_led->setRangeRGBf(r, g, b, m_numLeds, _tail, tailSplit_start);
//                /*
//                 * Example:
//                 * Head on position 1, body length 8 (tail = 417)
//                 *
//                 *                      splitPos
//                 *                    (headSplit_end)    tailSplit_start
//                 *                          |                   |
//                 *  headSplit_start         |                   |   tailSplit_end             [x] = head/tail, (0) = splitPos = headSplit_end
//                 *      |                   |                   |         |
//                 *  <head[1]><body        [(0)]              [422] : <tail[417]>>            [1]    (0)     422     421     420     419     418     [417]
//                 *  <head[2]><body[1]      (0)               [422] : <tail[418]>>            [2]     1      (0)     422     421     420     419     [418]
//                 *  <head[3]><body[2] : [1](0)               [422] : <tail[419]>>            [3]     2       1      (0)     422     421     420     [419]
//                 *  <head[4]><body[3] : [1](0)               [422] : <tail[420]>>            [4]     3       2      1       (0)     422     421     [420]
//                 *  <head[5]><body[4] : [1](0)               [422] : <tail[421]>>            [5]     4       3      2        1      (0)     422     [421]
//                 *  <head[6]><body[5] : [1](0)               [422] : <tail[422]>>            [6]     5       4      3        2       1      (0)     [422]
//                 *  <head[6]><body[5] :                              <tail[0]>>              [6]     5       4      3        2       1      [0]            <----- splitting OFF
//                 */
//                    ROS_INFO("Split state : head[%d] - headSplit_end[%d] - tailSplit_start[%d] - tail[%d]", _head, headSplit_end, tailSplit_start, _tail);
//                } else {
//                    //m_led->setRangeRGBf(r, g, b, m_numLeds, _head, _tail);
//                    m_led->setRangeRGBf(r, g, b, m_numLeds, _tail, _head);
//                    m_led->setRangeRGBf(0, 0, 0, m_numLeds, _tail-_tail_old, _tail);
//                    ROS_INFO("Tail(old): %d", _tail_old);
//                    ROS_INFO("Tail(new): %d", _tail);
//                    _tail_old = _tail;
//                    ROS_INFO("Normal state: head[%d] - tail[%d]", _head, _tail);
//                }

//                // The tail goes from 422 to 0
//                //if(!(_tail % (m_numLeds-1))) activateSplit = false;
//                if(_tail == (m_numLeds-1)) {
//                    m_led->setAllRGBf(0,0,0, m_numLeds);
//                    activateSplit = false;
//                }

//                feedback.current_head_pos = _head;
//                runningBunnyAS.publishFeedback(feedback);
//            }
//        }

//        m_led->setAllRGBf(0, 0, 0, m_numLeds);
//        result.current_head_pos = head;
//        runningBunnyAS.setSucceeded(result);

//    }

    // TODO Maybe expand this to be similar to blinky - support N number of cycles
    /**
     * @brief changelingCallback processes ChangelingGoal messages - similar to Blinky but currently only for a single "blink" and supports fluent transition from black to specifed color and reverse; previous name: lightActionCallback_7 with argument std_msgs::Float32MultiArray::ConstPtr&
     * @param goal
     */
    void changelingCallback(const iirob_led::ChangelingGoal::ConstPtr& goal) {
        iirob_led::ChangelingFeedback feedback;
        iirob_led::ChangelingResult result;

        int start_led = goal->start_led;
        int end_led = goal->end_led;

        if(start_led < 0) start_led = 0;

        double step = goal->step;
        if(step < 0.001) step = 0.1;
        if(step > 0.5) step = 0.1;                  // De facto this is a Blinky action
        int stepsPerHalfCycle = 1/step;             // we use integer since there is no such thing as (for example) 1/3 loop cycle ;)
        int stepsPerCycle = 2*stepsPerHalfCycle;    // full cycle contains two half-cycles
        int currentStep;
        int steps_temp = 0; // Use for counting how many steps we have done so far

        float r = goal->color.r;
        float g = goal->color.g;
        float b = goal->color.b;
        float h, s, v;

        // Initial conversion
        RGBConverter::rgbToHsv(r, g, b, &h, &s, &v);

        ROS_INFO("Target color (RGB): %.2f %.2f %.2f", r, g, b);
        ROS_INFO("Target color (HSV): %.2f %.2f %.2f", h, s, v);
        v = 0.0;    // Reset value (HSV) since this is the component that we will be working with
        ROS_INFO("Number of steps per cycle (single step = %.2f): %d", step, stepsPerCycle);

        // Increase value from 0.0 to 1.0 by step
        for(currentStep = 0; currentStep < stepsPerHalfCycle; v += step, ++currentStep, ++steps_temp) {
            RGBConverter::hsvToRgb(h, s, v, &r, &g, &b);
            m_led->setRangeRGBf(r, g, b, m_numLeds, start_led, end_led);
            ros::Duration(goal->time_between_steps).sleep();
            feedback.steps_in_cycle_left = stepsPerCycle - steps_temp;
            changelingAS.publishFeedback(feedback);
            ROS_DEBUG("%d out of %d steps left until end of cycle", feedback.steps_in_cycle_left, stepsPerCycle);
            ROS_DEBUG("HSV: %.2f %.2f %.2f", h, s, v);
        }

        // Decrease value from 0.0 to 1.0 by step
        for(currentStep = stepsPerHalfCycle; currentStep >= 0; v -= step, --currentStep, ++steps_temp) {
            RGBConverter::hsvToRgb(h, s, v, &r, &g, &b);
            m_led->setRangeRGBf(r, g, b, m_numLeds, start_led, end_led);
            ros::Duration(goal->time_between_steps).sleep();
            feedback.steps_in_cycle_left = stepsPerCycle - steps_temp;
            changelingAS.publishFeedback(feedback);
            ROS_DEBUG("%d out of %d steps left until end of cycle", feedback.steps_in_cycle_left, stepsPerCycle);
            ROS_DEBUG("HSV: %.2f %.2f %.2f", h, s, v);
        }

        m_led->setAllRGBf(0, 0, 0, m_numLeds);
        result.steps_in_cycle_left = currentStep;
        changelingAS.setSucceeded(result);
    }

    /*void spinnerCallback(const iirob_led::SpinnerGoal::ConstPtr& goal) {

    }*/

    void spin() { ros::spin(); }
};

int main(int argc, char **argv)
{

    std::string port;
	int num;
	bool override;

	ros::init(argc, argv, "iirob_led_node");
    ros::NodeHandle nh("~");

    nh.param<std::string>("port", port, "/dev/ttyUSB0");
    nh.param<int>("led_num", num, 1);
    nh.param<bool>("override", override, false);

    ROS_INFO("IIROB-LED node %s: Setting override to %s", ros::this_node::getName().c_str(), (override ? (char *)"on" : (char *)"off"));

    // If more then 8 LEDs are connected to the mC it will burn out
    if (!override && (num>8)) num = 8;
    ROS_INFO("IIROB-LED node %s: Setting number of LEDs to %d", ros::this_node::getName().c_str(), num);

    ROS_INFO("IIROB-LED node %s: Initializing ledNode on port %s with %d LEDs.", ros::this_node::getName().c_str(), port.c_str(), num);
//    LEDNode *ledNode = new LEDNode(nh);
    LEDNode *ledNode = new LEDNode(nh, port, num);

    // move all that is below to LEDNode
    //if (ledNode->init(port, num)) {
    /*if (ledNode->getStatus()) {

        // MOVE THOSE TO CONSTRUCTOR OF LEDNODE
        ros::Subscriber sub = nh.subscribe("led_command", 10, &LEDNode::commandCallback, ledNode);
        ros::Subscriber sub3 = nh.subscribe("chosen_directory", 10, &LEDNode::directoryCallback, ledNode);
        ros::Subscriber sub2 = nh.subscribe("led_color", 10, &LEDNode::colorCallback, ledNode);


        ros::ServiceServer set_led_directory_srv = nh.advertiseService("set_led_directory", &LEDNode::set_led_directory, ledNode);
        //ROS_INFO("%s: Service for set motor rotational speed [rpm] started on: %s", ros::this_node::getName().c_str(), set_led_directory_srv.getService().c_str()); // ?????????
        ROS_INFO("%s: IIROB-LED directory set to %s", ros::this_node::getName().c_str(), set_led_directory_srv.getService().c_str());

        ros::spin();

        delete ledNode;
    }
    else {
        ROS_ERROR("IIROB-LED node %s: Initializing ledNode on port %s with %d LEDs failed!", ros::this_node::getName().c_str(), port.c_str(), num);
    }*/

    if(ledNode->getStatus()) ledNode->spin();
    return 0;
}
