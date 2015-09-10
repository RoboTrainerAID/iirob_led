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
// NOTE All the checks if start_led or end_led are negative or not can be avoided by simply using unsigned chars (as far as I can tell internally the LED library is actually using this type)
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
    bool restartFlag;   // Used for displaying conditional information when calling init() (see init() for further details)

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
            subChosenDir = nodeHandle.subscribe("chosen_directory", 10, &LEDNode::directoryCallback, this);
            subLedColor = nodeHandle.subscribe("led_color", 10, &LEDNode::colorCallback, this);

            set_led_directory_srv = nodeHandle.advertiseService("set_led_directory", &LEDNode::set_led_directory, this);
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

    bool init(std::string const & port, int const & num) {
        m_led = new iirob_hardware::LEDStrip(port);
        if (m_led->ready()) {
            m_numLeds = num;
            m_led->setAllRGBf(0, 0, 0, m_numLeds);
            m_led->setUniRGB(0, 0, 0);
            //m_showDir = 4;

            //blinkyAS.start();
            ROS_INFO("%s: IIROB-LED action server started", ros::this_node::getName().c_str());
            ROS_INFO("Number of LEDs: %d", m_numLeds);

            // NOTE It seems that the init might also be there in order to reconfigure the LEDNode so
            // it might be a good idea to check here if the action servers are active and if so
            // shut those down and start again. Unless init() is just a mishap and actually has to
            // be part of the constructor

            if(!restartFlag) ROS_INFO("Starting action servers:");
            else ROS_INFO("Restarting action servers:");
            if(blinkyAS.isActive()) {
                ROS_WARN_COND((blinkyAS.isPreemptRequested() || blinkyAS.isNewGoalAvailable()), "Action server has preempt request or a panding new goal!");
                blinkyAS.shutdown();
            }
            blinkyAS.start();
            ROS_INFO("blinky");
            if(policeAS.isActive()) {
                ROS_WARN_COND((policeAS.isPreemptRequested() || policeAS.isNewGoalAvailable()), "Action server has preempt request or a panding new goal!");
                policeAS.shutdown();
            }
            policeAS.start();
            ROS_INFO("police");
            if(fourMusketeersAS.isActive()) {
                ROS_WARN_COND((fourMusketeersAS.isPreemptRequested() || fourMusketeersAS.isNewGoalAvailable()), "Action server has preempt request or a panding new goal!");
                fourMusketeersAS.shutdown();
            }
            fourMusketeersAS.start();
            ROS_INFO("fourMusketeers");
            if(runningBunnyAS.isActive()) {
                ROS_WARN_COND((runningBunnyAS.isPreemptRequested() || runningBunnyAS.isNewGoalAvailable()), "Action server has preempt request or a panding new goal!");
                runningBunnyAS.shutdown();
            }
            runningBunnyAS.start();
            ROS_INFO("runningBunny");
            if(changelingAS.isActive()) {
                ROS_WARN_COND((changelingAS.isPreemptRequested() || changelingAS.isNewGoalAvailable()), "Action server has preempt request or a panding new goal!");
                changelingAS.shutdown();
            }
            changelingAS.start();
            ROS_INFO("changeling");

            return true;
        }
        return false;
    }

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

	//Empfangen der Richtung
    void directoryCallback(const std_msgs::String::ConstPtr& msg) {
        std::string tempMsg = msg->data.c_str();

        if (tempMsg == "r") {
            m_showDir = 0;
        }
        else if (tempMsg == "l") {
            m_showDir = 1;
        }
        else if (tempMsg == "b") {
            m_showDir = 2;
        }
        else if (tempMsg == "f") {
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

	bool set_led_directory(iirob_led::SetLedDirectory::Request &req,
                           iirob_led::SetLedDirectory::Response &res)
    {
        ROS_INFO("I heard: [%f %f %f %f %d %d]",
                 req.color.r, req.color.g, req.color.b, req.color.a, req.start_led, req.end_led);
        m_led->setRangeRGBf(req.color.r, req.color.g, req.color.b, m_numLeds, req.start_led, req.end_led);
        return true;
    }

    /*void playLed(const iirob_led::PlayLedGoalConstPtr& goal)
    {
        iirob_led::PlayLedFeedback feedback;
        iirob_led::PlayLedResult result;
        double begin = ros::Time::now().toSec();
        double time_diff;
        ros::Rate frequency(goal->frequency);
        int i = 0;
        int num_leds_middle, num_leds_begin_end;
        int start_led = goal->start_led;
        int end_led = goal->end_led;
        int directory_size = goal->directory_size;
        float hue = 0.0;
        float factor;

        if (end_led >= m_numLeds) {
            end_led = m_numLeds-1;
        }

//         while (! m_cancelFlag ) {
//         hue += 0.01;
//         if (hue >= 1.0)
//             hue -= 1.0;
//         m_led->setAllHue(hue, m_numLeds);
//         //OpenThreads::Thread::microSleep(m_msec*1000);
//     }

        ROS_DEBUG("%s starting PlayLed action with duration %f s, frequency %f Hz, color [%f %f %f] [RGB]. Duration: %f sec, frequency: %f Hz, start led: %d, end led: %d, directory: %d",
                 ros::this_node::getName().c_str(), goal->duration, goal->frequency,
                 goal->color.r, goal->color.g, goal->color.b, goal->duration, goal->frequency, goal->start_led, goal->end_led, goal->directory_size);

        while(true) {

            if (time_diff >= goal->duration) {
                break;
            }

            if (start_led == 0 && end_led == 0 && directory_size == 0) {
                hue += 0.01;
                if (hue >= 1.0) {
                    hue -= 1.0;
                }
                m_led->setAllHue(hue, m_numLeds);
            }
            else if (goal->directory_size == 0) {
                m_led->setRangeRGBf(goal->color.r, goal->color.g, goal->color.b, m_numLeds, start_led, end_led);
            }
            else {
                if (goal->directory_size < 3) {
                    m_led->setRangeRGBf(goal->color.r, goal->color.g, goal->color.b, m_numLeds, i, i+directory_size);
                }
                else {
                    m_led->setAllRGB(0, 0, 0, m_numLeds);
                    num_leds_begin_end = directory_size/3;
                    num_leds_middle = 2*num_leds_begin_end;
                    for (int j = 1; j <= num_leds_begin_end; j++) {
                        factor = (1.0/num_leds_begin_end)*j;
                        ROS_INFO("%f", factor);
                        m_led->setRangeRGBf((goal->color.r)*factor, (goal->color.g)*factor, (goal->color.b)*factor, m_numLeds,
                                            i+j-1, i+j-1);
                    }
                    m_led->setRangeRGBf(goal->color.r, goal->color.g, goal->color.b, m_numLeds,
                                            i+num_leds_begin_end, i+num_leds_begin_end+num_leds_middle);
                    for (int j = 1; j <= num_leds_begin_end; j++) {
                        factor = (1.0/num_leds_begin_end)*(num_leds_begin_end-j+1);
                        m_led->setRangeRGBf((goal->color.r)*factor, (goal->color.g)*factor, (goal->color.b)*factor, m_numLeds,
                                            i+num_leds_begin_end+num_leds_middle+j, i+num_leds_begin_end+num_leds_middle+j);
                    }
                }
                i++;
                i = (i+directory_size)%end_led;
                if (i < start_led) {
                    i = start_led;
                }
            }
            feedback.until_end = time_diff;
            ROS_DEBUG("%s: PlayLedAction until end %f s", ros::this_node::getName().c_str(), feedback.until_end);
            as_.publishFeedback(feedback);

            frequency.sleep();
            time_diff = ros::Time::now().toSec() - begin;
        }

        m_led->setAllRGB(0, 0, 0, m_numLeds);
        result.real_duration = time_diff;
        as_.setSucceeded(result);
    }*/

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

    /**
     * @brief runningBunnyCallback processes RunningBunnyGoal messages - a given stripe of LEDs (bunny) "jumps" in circle (a jump is defined as the number of LEDs that are skipped by the bunny); previous name: lightActionCallback_6 with argument std_msgs::Float32MultiArray::ConstPtr&
     * @param goal
     */
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
        if(!body) body = 1;
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
    }

    // TODO Maybe expand this to be similar to blinky - support N number of cycles
    void changelingCallback(const iirob_led::ChangelingGoal::ConstPtr& goal) {
        ROS_INFO("CHANGELING CALLBACK");
        iirob_led::ChangelingFeedback feedback;
        iirob_led::ChangelingResult result;

        int start_led = goal->start_led;
        int end_led = goal->end_led;

        if(start_led < 0) start_led = 0;

        double step = goal->step;
        if(step < 0.01) step = 0.1;
        if(step > 0.5) step = 0.1;
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
            ROS_INFO("%d out of %d steps left until end of cycle", feedback.steps_in_cycle_left, stepsPerCycle);
            ROS_INFO("HSV: %.2f %.2f %.2f", h, s, v);
        }

        // Decrease value from 0.0 to 1.0 by step
        for(currentStep = stepsPerHalfCycle; currentStep >= 0; v -= step, --currentStep, ++steps_temp) {
            RGBConverter::hsvToRgb(h, s, v, &r, &g, &b);
            m_led->setRangeRGBf(r, g, b, m_numLeds, start_led, end_led);
            ros::Duration(goal->time_between_steps).sleep();
            feedback.steps_in_cycle_left = stepsPerCycle - steps_temp;
            changelingAS.publishFeedback(feedback);
            ROS_INFO("%d out of %d steps left until end of cycle", feedback.steps_in_cycle_left, stepsPerCycle);
            ROS_INFO("HSV: %.2f %.2f %.2f", h, s, v);
        }

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
