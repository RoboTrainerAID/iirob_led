#include <ros/ros.h>
#include "actionlib/server/simple_action_server.h"

//#include "iirob_led/PlayLedAction.h"
#include "iirob_led/BlinkyAction.h"
#include "iirob_led/PoliceAction.h"
#include "iirob_led/FourMusketeersAction.h"
#include "iirob_led/RunningBunnyAction.h"
#include "iirob_led/ChangelingAction.h"
#include "iirob_led/SpinnerAction.h"
#include "iirob_led/SetLedDirectory.h"

#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>

// TODO:
/* - OVERRIDE - when ready set to false
 * - Put subscribers inside LEDNode
 * - Merge init with constructor and store result inside status (class member)
 */

#include "LEDStrip.h"

class LEDNode
{
    int m_msec;
    int m_numLeds;
    iirob_hardware::LEDStrip* m_led;
    int m_showDir;
    bool light_flash_dir;
    bool status;        // contains the return result from init()
    std::string port;
    int num;

    ros::Subscriber subLedCommand;
    ros::Subscriber subChosenDir;
    ros::Subscriber subLedColor;

    actionlib::SimpleActionServer<iirob_led::BlinkyAction> blinkyAS;
    actionlib::SimpleActionServer<iirob_led::PoliceAction> policeAS;
    actionlib::SimpleActionServer<iirob_led::FourMusketeersAction> fourMusketeersAS;
    actionlib::SimpleActionServer<iirob_led::RunningBunnyAction> runningBunnyAS;
    actionlib::SimpleActionServer<iirob_led::ChangelingAction> changelingAS;
    actionlib::SimpleActionServer<iirob_led::SpinnerAction> spinnerAS;
public:
	//! Constructor.
    LEDNode(ros::NodeHandle nodeHandle, std::string const& _port, int const& _num)
        : m_led(0), m_numLeds(423), m_msec(1),
          port(_port), num(_num),
          blinkyAS(nodeHandle, "blinky", boost::bind(&LEDNode::blinkyCallback, this, _1), false),
          policeAS(nodeHandle, "police", boost::bind(&LEDNode::policeCallback, this, _1), false),
          fourMusketeersAS(nodeHandle, "four_musketeers", boost::bind(&LEDNode::fourMusketeersCallback, this, _1), false),
          runningBunnyAS(nodeHandle, "running_bunny", boost::bind(&LEDNode::runningBunnyCallback, this, _1), false),
          changelingAS(nodeHandle, "changeling", boost::bind(&LEDNode::changelingCallback, this, _1), false),
          spinnerAS(nodeHandle, "spinner", boost::bind(&LEDNode::spinnerCallback, this, _1), false)
    {
        light_flash_dir = true;
        status = init(port, num);

        if(status)
        {
            //subLedCommand = nh.subscribe("led_command", 10, &LEDNode::commandCallback, this);
            //subChosenDir = nh.subscribe("chosen_directory", 10, &LEDNode::directoryCallback, this);
            //subLedColor = nh.subscribe("led_color", 10, &LEDNode::colorCallback, this);
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
        spinnerAS.shutdown();
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

            // NOTE:
            // It seems that the init might also be there in order to reconfigure the LEDNode so
            // it might be a good idea to check here if the action servers are active and if so
            // shut those down and start again. Unless init() is just a mishap and actually has to
            // be part of the constructor

            ROS_INFO("Starting actions servers: ");
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
            spinnerAS.start();
            ROS_INFO("spinner");

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
         * NOTE: See what exactly "chosen_directory" is. From what I see above it seems that it only
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

    // Old name: lightActionCallback_2 with argument std_msgs::Float32MultiArray::ConstPtr&
    // New name: Blinky
    // Description: turns on and off selected LEDs
    void blinkyCallback(const iirob_led::BlinkyGoal::ConstPtr& goal) {
        ROS_INFO("BLINKY CALLBACK HERE!");
        iirob_led::BlinkyFeedback feedback;
        iirob_led::BlinkyResult result;
        int blinks_left = goal->blinks;
        int start_led = goal->start_led;
        int end_led = goal->end_led;

        if(start_led < 0)
            start_led = 0;

        if(end_led >= m_numLeds)
            end_led = m_numLeds-1;

        ROS_DEBUG("%s starting Blinky action with color [%f %f %f] [RGB]. Duration(ON): %f sec, Duration(OFF): %f, start led: %d, end led: %d",
                 ros::this_node::getName().c_str(),
                 goal->color.r, goal->color.g, goal->color.b, goal->duration_on, goal->duration_off, start_led, end_led);

        // POST VS PREINCREMENT

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

        // Old
        /*for(int j=0; j<(msg->data[0]); j++) {
                m_led->setRangeRGBf(msg->data[5], msg->data[6], msg->data[7], m_numLeds, msg->data[3], msg->data[4]);
                //~ for (int i=0; i<(msg->data[1]*100000000); i++);
                ros::Duration(msg->data[1]).sleep();
                m_led->setRangeRGBf(0, 0, 0, m_numLeds, msg->data[3], msg->data[4]);
                //~ for (int i=0; i<(msg->data[2]*100000000); i++);
                ros::Duration(msg->data[2]).sleep();
        }*/
    }

    void policeCallback(const iirob_led::PoliceGoal::ConstPtr& goal) {
        iirob_led::PoliceFeedback feedback;
        iirob_led::PoliceResult result;
        int blinks_left = goal->blinks;

        // LEFT_OUTER | LEFT_INNER | RIGHT_INNER | RIGHT_OUTER
        int start_led_left_outer = goal->start_led_left_outer;
        int end_led_left_outer = goal->end_led_left_outer;
        int start_led_left_inner = goal->start_led_left_inner;
        int end_led_left_inner = goal->end_led_left_inner;

        int start_led_right_inner = goal->start_led_right_inner;
        int end_led_right_inner = goal->end_led_right_inner;
        int start_led_right_outer = goal->start_led_right_outer;
        int end_led_right_outer = goal->end_led_right_outer;

        // TODO: See a proper handling of out of range LEDs (more difficult then with Blinky since we have 4 sub-sections each with its own start and end
        /*if(start_led < 0)
            start_led = 0;

        if(end_led >= m_numLeds)
            end_led = m_numLeds-1;*/

        ROS_INFO("Here comes the police!");

        int i = 0;
        for(; i < goal->blinks; ++i, --blinks_left)
        {
            m_led->setRangeRGBf(goal->color_left_outer.r, goal->color_left_outer.g, goal->color_left_outer.b, m_numLeds, start_led_left_outer, end_led_left_outer);
            m_led->setRangeRGBf(goal->color_left_inner.r, goal->color_left_inner.g, goal->color_left_inner.b, m_numLeds, start_led_left_inner, end_led_left_inner);
            ros::Duration(goal->duration_on).sleep();
            m_led->setRangeRGBf(0, 0, 0, m_numLeds, start_led_left_outer, end_led_left_outer);
            m_led->setRangeRGBf(0, 0, 0, m_numLeds, start_led_left_inner, end_led_left_inner);
            ros::Duration(goal->duration_off).sleep();
            m_led->setRangeRGBf(goal->color_right_inner.r, goal->color_right_inner.g, goal->color_right_inner.b, m_numLeds, start_led_right_inner, end_led_right_inner);
            m_led->setRangeRGBf(goal->color_right_outer.r, goal->color_right_outer.g, goal->color_right_outer.b, m_numLeds, start_led_right_outer, end_led_right_outer);
            ros::Duration(goal->duration_on).sleep();
            m_led->setRangeRGBf(0, 0, 0, m_numLeds, start_led_right_inner, end_led_right_inner);
            m_led->setRangeRGBf(0, 0, 0, m_numLeds, start_led_right_outer, end_led_right_outer);
            ros::Duration(goal->duration_off).sleep();

            feedback.blinks_left = blinks_left;
            ROS_INFO("%s: Police will blink %d more times", ros::this_node::getName().c_str(), feedback.blinks_left);
            policeAS.publishFeedback(feedback);
        }

        /*if(light_flash_dir)
        {
            for(; i < goal->blinks; ++i)
            {
                m_led->setRangeRGBf(goal->color_left_outer.r, goal->color_left_outer.g, goal->color_left_outer.b, m_numLeds, start_led_left_outer, end_led_left_outer);
                m_led->setRangeRGBf(goal->color_left_inner.r, goal->color_left_inner.g, goal->color_left_inner.b, m_numLeds, start_led_left_inner, end_led_left_inner);
                ros::Duration(goal->duration_on).sleep();
                m_led->setRangeRGBf(0, 0, 0, m_numLeds, start_led_left_outer, end_led_left_outer);
                m_led->setRangeRGBf(0, 0, 0, m_numLeds, start_led_left_inner, end_led_left_inner);
                ros::Duration(goal->duration_off).sleep();

                feedback.blinks_left = blinks_left;
                ROS_INFO("%s: Police will blink %d more times", ros::this_node::getName().c_str(), feedback.blinks_left);
                policeAS.publishFeedback(feedback);
            }
        }
        else
        {
            for(; i < goal->blinks; ++i)
            {
                m_led->setRangeRGBf(goal->color_right_inner.r, goal->color_right_inner.g, goal->color_right_inner.b, m_numLeds, start_led_right_inner, end_led_right_inner);
                m_led->setRangeRGBf(goal->color_right_outer.r, goal->color_right_outer.g, goal->color_right_outer.b, m_numLeds, start_led_right_outer, end_led_right_outer);
                ros::Duration(goal->duration_on).sleep();
                m_led->setRangeRGBf(0, 0, 0, m_numLeds, start_led_right_inner, end_led_right_inner);
                m_led->setRangeRGBf(0, 0, 0, m_numLeds, start_led_right_outer, end_led_right_outer);
                ros::Duration(goal->duration_off).sleep();
            }
        }*/

        feedback.blinks_left = blinks_left;
        ROS_INFO("%s: Police will blink %d more times", ros::this_node::getName().c_str(), feedback.blinks_left);

        m_led->setAllRGBf(0, 0, 0, m_numLeds);
        result.blinks_left = blinks_left;
        policeAS.setSucceeded(result);

        // Toggle blinking stripes
        light_flash_dir = !light_flash_dir;
    }

    void fourMusketeersCallback(const iirob_led::FourMusketeersGoal::ConstPtr& goal) {

    }

    void runningBunnyCallback(const iirob_led::RunningBunnyGoal::ConstPtr& goal) {

    }

    void changelingCallback(const iirob_led::ChangelingGoal::ConstPtr& goal) {

    }

    void spinnerCallback(const iirob_led::SpinnerGoal::ConstPtr& goal) {

    }
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
    if (ledNode->getStatus()) {

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
    }
    return 0;
}
