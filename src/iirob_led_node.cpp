#include <ros/ros.h>
#include "actionlib/server/simple_action_server.h"

#include "iirob_led/PlayLedAction.h"

#include "std_msgs/String.h"
#include "std_msgs/ColorRGBA.h"

#include "LEDStrip.h"

class LEDNode
{
public:
	//! Constructor.
	LEDNode(ros::NodeHandle nodeHandle) : m_numLeds(0), m_led(0), m_msec(1), as_(nodeHandle, "play_led", boost::bind(&LEDNode::playLed, this, _1), false)
    {
    };

	//! Destructor.
	~LEDNode(){
		//turn everything off again
		if (m_led) {
			m_led->setAllRGBf(0, 0, 0, m_numLeds);
			m_led->setUniRGB(0, 0, 0);
			delete m_led;
		}
	};

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

	void playLed(const iirob_led::PlayLedGoalConstPtr& goal)
    {
        iirob_led::PlayLedFeedback feedback;
        iirob_led::PlayLedResult result;
        double begin = ros::Time::now().toSec();
        double time_diff;
        ros::Rate frequency(goal->frequency);
        int i = 0;

        ROS_INFO("%s starting PlayLed action with duration %f s, frequency %f Hz and color [%f %f %f] [RGB].", ros::this_node::getName().c_str(), goal->duration, goal->frequency, goal->color.r, goal->color.g, goal->color.b);
        
        //const std_msgs::ColorRGBA::ConstPtr& msg;




        while(true) {
            m_led->setRangeRGBf(goal->color.r, goal->color.g, goal->color.b, m_numLeds, i, i+1);            
            i++; i %= m_numLeds;            

            time_diff = ros::Time::now().toSec() - begin;
            if (time_diff >= goal->duration) {
                break;
            }

            feedback.until_end = time_diff;
            ROS_INFO("%s: PlayLedAction until end %f s", ros::this_node::getName().c_str(), feedback.until_end);
            as_.publishFeedback(feedback);
            frequency.sleep();
        }

        m_led->setAllRGB(0, 0, 0, m_numLeds);
        result.real_duration = time_diff;
        as_.setSucceeded(result);
    }

protected:
        

private:
	int m_msec;	
	int m_numLeds;
    irob_hardware::LEDStrip* m_led;
    int m_showDir;
    
    actionlib::SimpleActionServer<iirob_led::PlayLedAction> as_;

};

bool LEDNode::init(std::string const & port, int const & num) {

	m_led = new irob_hardware::LEDStrip(port);
	if (m_led->ready()) {
		m_numLeds = num;
		m_showDir = 4;

        as_.start();
        ROS_INFO("%s: PlayLed action server started", ros::this_node::getName().c_str());

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

        ros::Subscriber sub = nodeHandle.subscribe("led_command", 1000, &LEDNode::commandCallback, ledNode);
        ros::Subscriber sub3 = nodeHandle.subscribe("chosen_directory", 1000, &LEDNode::directoryCallback, ledNode);
        ros::Subscriber sub2 = nodeHandle.subscribe("led_color", 1000, &LEDNode::colorCallback, ledNode);

        ros::spin();

        delete ledNode;
    }
    else {
        ROS_ERROR("IIROB-LED node %s: Initializing ledNode on port %s with %d LEDs failed!", ros::this_node::getName().c_str(), port.c_str(), num);
    }
    return 0;
}
