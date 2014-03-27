#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/ColorRGBA.h"

#include "LEDStrip.h"

class LEDNode
{
public:
	//! Constructor.
	LEDNode() : m_cancelFlag(false), m_numLeds(0), m_led(0), m_msec(1) {};

	//! Destructor.
	~LEDNode(){
		//turn everything off again
		if (m_led) {
			m_led->setAllRGBf(0, 0, 0, m_numLeds);
			m_led->setUniRGB(0, 0, 0);
			delete m_led;
		}
	};

	virtual void run();

	bool init(std::string const & port, int const & num);

	void stop() { m_cancelFlag = true;}

	//! Callback function for subscriber.
	void commandCallback(const std_msgs::String::ConstPtr& msg) {
		ROS_INFO("I heard: [%s]", msg->data.c_str());
		if (msg->data == "off") {
			m_led->setAllRGBf(0, 0, 0, m_numLeds);
		}
	}
	void colorCallback(const std_msgs::ColorRGBA::ConstPtr& msg) {
		ROS_INFO("I heard: [%f %f %f %f %d]", msg->r, msg->g, msg->b, msg->a, m_numLeds );
		ROS_INFO("[%f]", showDir);
		//Entscheidung welche LED-Richtung gezeigt wird
		switch(showDir) {
			case 0:
				m_led->setMyAllRGBf(msg->r, msg->g, msg->b, m_numLeds, 0, 7);
				ROS_INFO("A");
				break;
			case 1:
				m_led->setMyAllRGBf(msg->r, msg->g, msg->b, m_numLeds, 7, 14);
				ROS_INFO("B");
				break;
			case 2:
				m_led->setMyAllRGBf(msg->r, msg->g, msg->b, m_numLeds, 14, 22);
				ROS_INFO("C");
				break;
			case 3:
				m_led->setMyAllRGBf(msg->r, msg->g, msg->b, m_numLeds, 23, 31);
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
	int showDir;
	void directoryCallback(const std_msgs::String::ConstPtr& msg) {
		std::string tempMsg = msg->data.c_str();
		if (tempMsg == "r") {
			showDir = 0;
		}
		else if (tempMsg == "l") {
			showDir = 1;
		}
		else if (tempMsg == "b") {
			showDir = 2;
		}
		else if (tempMsg == "f") {
			showDir = 3;
		}
		else {
			//Default
			showDir = 4;
		}
	}

	//! The actual message.
	std::string message;


private:
	bool m_cancelFlag;
	//OpenThreads::Mutex m_ledMutex;
	std::string m_port;
	int m_numLeds;
	irob_hardware::LEDStrip* m_led;
	int m_msec;

};

void LEDNode::run() {

	float hue = 0.0;

	while (! m_cancelFlag ) {
		hue += 0.01;
		if (hue >= 1.0)
			hue -= 1.0;
		m_led->setAllHue(hue, m_numLeds);
		//OpenThreads::Thread::microSleep(m_msec*1000);
	}

	m_led->setAllRGB(0, 0, 0, m_numLeds);
	m_led->setUniRGB(0, 0, 0);

}

bool LEDNode::init(std::string const & port, int const & num) {

	m_led = new irob_hardware::LEDStrip(port);
	if (m_led->ready()) {
		m_numLeds = num;
		return true;
	}


	return false;

}


int main(int argc, char **argv)
{

	int mode = 0;
	int num = 1;
	float speed = 1.0;
	bool printDebug = false;
	bool override = true;

	std::string port;
#ifdef IROB_OS_WIN32
	port = "COM5";
#else
	port = "/dev/ttyUSB0";
#endif

	std::cout << std::endl << "Supported command line options:"
			<< std::endl << "  -p <port>  default: COM5, /dev/ttyUSB0"
			<< std::endl << "  -n <num>   number of leds on string (RGB triples)"
			<< std::endl << "  -X         override current limiter (max. 8 LEDs w/o ext. supply)"
			<< std::endl << "             WARNING: improper usage may fry USB port!"
			<< std::endl;

	for (int i=1; i<argc; ++i) {
		if ((i < argc-1) && !strcmp(argv[i], "-n"))
			num = atoi(argv[i+1]);
		else if (!strcmp(argv[i], "-X"))
			override = true;
		else if ((i < argc-1) && !strcmp(argv[i], "-p"))
			port = argv[i+1];
	}

	num = 31;

	if ((!override) && (num>8))
		num = 8;

	LEDNode *ledNode = new LEDNode ();
	if (! ledNode->init(port, num)) {
		std::cerr << "Failed to init LEDNode" << std::endl;
		return (-1);
	}

	//ledNode->start();


	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line. For programmatic
	 * remappings you can use a different version of init() which takes remappings
	 * directly, but for most command-line programs, passing argc and argv is the easiest
	 * way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "LEDROSNode");

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle n;

	/**
	 * The subscribe() call is how you tell ROS that you want to receive messages
	 * on a given topic.  This invokes a call to the ROS
	 * master node, which keeps a registry of who is publishing and who
	 * is subscribing.  Messages are passed to a callback function, here
	 * called chatterCallback.  subscribe() returns a Subscriber object that you
	 * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
	 * object go out of scope, this callback will automatically be unsubscribed from
	 * this topic.
	 *
	 * The second parameter to the subscribe() function is the size of the message
	 * queue.  If messages are arriving faster than they are being processed, this
	 * is the number of messages that will be buffered up before beginning to throw
	 * away the oldest ones.
	 */

	ros::Subscriber sub = n.subscribe("ledCommand", 1000, &LEDNode::commandCallback, ledNode);
	ros::Subscriber sub3 = n.subscribe("chosen_Directory", 1000, &LEDNode::directoryCallback, ledNode); //hinzugefügt
	ros::Subscriber sub2 = n.subscribe("ledColor", 1000, &LEDNode::colorCallback, ledNode);


	/**
	 * ros::spin() will enter a loop, pumping callbacks.  With this version, all
	 * callbacks will be called from within this thread (the main one).  ros::spin()
	 * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	 */
	ros::spin();

	std::cout << "Canceled" << std::endl;

	//ledNode->stop();
	//ledNode->join();
	delete ledNode;

	return 0;
}
