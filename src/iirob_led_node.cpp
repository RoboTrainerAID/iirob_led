#include <ros/ros.h>
#include "actionlib/server/simple_action_server.h"

#include "iirob_led/PlayLedAction.h"
#include "iirob_led/BlinkyAction.h"
#include "iirob_led/SetLedDirectory.h"

#include "std_msgs/String.h"
#include "std_msgs/ColorRGBA.h"

#include "LEDStrip.h"

class LEDNode
{
public:
	//! Constructor.
    LEDNode(ros::NodeHandle nodeHandle) : m_numLeds(0), m_led(0), m_msec(1), as_(nodeHandle, "blinky", boost::bind(&LEDNode::blinkyCallback, this, _1), false)
    {
    }

	//! Destructor.
	~LEDNode(){
		//turn everything off again
		if (m_led) {
			m_led->setAllRGBf(0, 0, 0, m_numLeds);
			m_led->setUniRGB(0, 0, 0);
			delete m_led;
		}
    }

	bool init(std::string const & port, int const & num);

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
			m_showDir = 4;
		}
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
        iirob_led::BlinkyFeedback feedback;
        iirob_led::BlinkyResult result;
        int blinks_left;
        int start_led = goal->start_led;
        int end_led = goal->end_led;

        if(start_led < 0)
            start_led = 0;

        if(end_led >= m_numLeds)
            end_led = m_numLeds-1;

        ROS_DEBUG("%s starting Blinky action with color [%f %f %f] [RGB]. Duration(ON): %f sec, Duration(OFF): %f, start led: %d, end led: %d",
                 ros::this_node::getName().c_str(),
                 goal->color.r, goal->color.g, goal->color.b, goal->duration_on, goal->duration_off, start_led, end_led);


        for(int i = 0; i < goal->blinks; ++i)
        {
            // Set selected LEDs and turn them on
            m_led->setRangeRGB(goal->color.r, goal->color.g, goal->color.b, m_numLeds, start_led, end_led);
            ros::Duration(goal->duration_on).sleep();
            // Set selected LEDs and turn them off
            m_led->setRangeRGB(0, 0, 0, m_numLeds, goal->start_led, goal->end_led);
            ros::Duration(goal->duration_off).sleep();

            blinks_left--;
            feedback.blinks_left = blinks_left;
            ROS_DEBUG("%s: Blinky will blink %d more times", ros::this_node::getName().c_str(), feedback.blinks_left);
            as_.publishFeedback(feedback);
        }

        m_led->setAllRGB(0, 0, 0, m_numLeds);
        result.blinks_left = blinks_left;
        as_.setSucceeded(result);

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


private:
    int m_msec;
    int m_numLeds;
    iirob_hardware::LEDStrip* m_led;
    int m_showDir;

    actionlib::SimpleActionServer<iirob_led::BlinkyAction> as_;

};

bool LEDNode::init(std::string const & port, int const & num) {

    m_led = new iirob_hardware::LEDStrip(port);
	if (m_led->ready()) {
		m_numLeds = num;
		m_showDir = 4;

        as_.start();
        ROS_INFO("%s: IIROB-LED action server started", ros::this_node::getName().c_str());

		return true;
	}
	return false;
}


int main(int argc, char **argv)
{

    std::string port;
	int num;
	bool override;

	ros::init(argc, argv, "iirob_led_node");
	ros::NodeHandle nodeHandle("~");

    nodeHandle.param<std::string>("port", port, "/dev/ttyUSB0");
    nodeHandle.param<int>("led_num", num, 1);
    nodeHandle.param<bool>("override", override, false);

    if ((!override) && (num>8))
        num = 8;

    ROS_INFO("IIROB-LED node %s: Initializing ledNode on port %s with %d LEDs.", ros::this_node::getName().c_str(), port.c_str(), num);
    LEDNode *ledNode = new LEDNode(nodeHandle);
    if (ledNode->init(port, num)) {

        // Create subscribers for all the
        ros::Subscriber sub = nodeHandle.subscribe("led_command", 1000, &LEDNode::commandCallback, ledNode);
        ros::Subscriber sub3 = nodeHandle.subscribe("chosen_directory", 1000, &LEDNode::directoryCallback, ledNode);
        ros::Subscriber sub2 = nodeHandle.subscribe("led_color", 1000, &LEDNode::colorCallback, ledNode);

        //ROS_INFO("Starting subscription for Blinky action");
        //ros::Subscriber blinkySub = nodeHandle.subscribe("blinky", 1, &LEDNode::blinkyCallback, ledNode);

        ros::ServiceServer set_led_directory_srv = nodeHandle.advertiseService("set_led_directory", &LEDNode::set_led_directory, ledNode);
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
