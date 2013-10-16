#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/projectrosgui/qnode.hpp"

namespace projectrosgui {

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
    init_argv(argv){
    this->max_data_length = 200;
}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"projectrosgui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);

    command = n.advertise<std_msgs::String>("Command", 1000);
    ultrasound = n.subscribe("Ultrasound", 1000, &QNode::ultraCallback, this);
    irsensor = n.subscribe("IRSensor", 1000, &QNode::irCallback, this);

	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,"projectrosgui");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;
	while ( ros::ok() ) {

//		std_msgs::String msg;
//		std::stringstream ss;
//		ss << "hello world " << count;
//		msg.data = ss.str();
//		chatter_publisher.publish(msg);
//        log(Info,std::string("I sent: ")+msg.data);
        ros::spinOnce();
//      loop_rate.sleep();
//		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::addData(QVector<double> & x, QVector<double> & idx, double data){
    x.push_back(data);
    idx.push_back(idx.back() + 1);
    if (idx.back() - idx.front() > this->max_data_length)
    {
        for (int i = 0; i < this->max_data_length; i++)
        {
            x[i] = x[i+1];
            idx[i] = idx[i+1];
        }
//        x.pop_front();
//        idx.pop_front();
    }
}

void QNode::irCallback(const sensor_msgs::RangeConstPtr &msg){
    int id = msg->radiation_type;
    switch (id) {
    case 1: {

        this->ir1.push_back(msg->range);
        this->ir1_t.push_back(this->ir1_t.back() + 1);
        if (this->ir1.size() > this->max_data_length)
        {
            this->ir1.pop_front();
            this->ir1_t.pop_front();
        }

        break;
    }
    case 2: {
        this->ir2.push_back(msg->range);
        this->ir2_t.push_back(this->ir2_t.back() + 1);
        if (this->ir2.size() > this->max_data_length)
        {
            this->ir2.pop_front();
            this->ir2_t.pop_front();
        }
        break;
    }
    case 3: {
        this->ir3.push_back(msg->range);
        this->ir3_t.push_back(this->ir3_t.back() + 1);
        if (this->ir3.size() > this->max_data_length)
        {
            this->ir3.pop_front();
            this->ir3_t.pop_front();
        }
        break;
    }
    }
    std::stringstream ss;
    ss << "IR" << id << "Updated" << msg->range;
//    log(Info,ss.str());
    emit recieveIRData();
}

void QNode::ultraCallback(const sensor_msgs::RangeConstPtr &msg){
    int id = msg->radiation_type;
    switch (id) {
    case 1: {

        this->ul1.push_back(msg->range);
        this->ul1_t.push_back(this->ul1_t.back() + 1);
        if (this->ul1.size() > this->max_data_length)
        {
            this->ul1.pop_front();
            this->ul1_t.pop_front();
        }

        break;
    }
    case 2: {
        this->ul2.push_back(msg->range);
        this->ul2_t.push_back(this->ul2_t.back() + 1);
        if (this->ul2.size() > this->max_data_length)
        {
            this->ul2.pop_front();
            this->ul2_t.pop_front();
        }
        break;
    }
    case 3: {
        this->ul3.push_back(msg->range);
        this->ul3_t.push_back(this->ul3_t.back() + 1);
        if (this->ul3.size() > this->max_data_length)
        {
            this->ul3.pop_front();
            this->ul3_t.pop_front();
        }
        break;
    }
    }
    std::stringstream ss;
    ss << "Ultra" << id << "Updated" << msg->range;
//    log(Info,ss.str());
    emit recieveUltraData();
}

void QNode::log( const LogLevel &level, const std::string &msg) {
    logging_model.insertRows(logging_model.rowCount(),1);
    std::stringstream logging_model_msg;
    switch ( level ) {
        case(Debug) : {
                ROS_DEBUG_STREAM(msg);
                logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
                break;
        }
        case(Info) : {
                ROS_INFO_STREAM(msg);
                logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
                break;
        }
        case(Warn) : {
                ROS_WARN_STREAM(msg);
                logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
                break;
        }
        case(Error) : {
                ROS_ERROR_STREAM(msg);
                logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
                break;
        }
        case(Fatal) : {
                ROS_FATAL_STREAM(msg);
                logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
                break;
        }
    }
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    emit loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace projectrosgui
