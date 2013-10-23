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
    this->max_data_length = 80;
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

    command = n.advertise<std_msgs::String>("SensorStatus", 1000);
    ultrasound = n.subscribe("Ultrasound", 1000, &QNode::ultraCallback, this);
    irsensor = n.subscribe("IRSensor", 1000, &QNode::irCallback, this);
    status = n.subscribe("SensorStatus", 1000, &QNode::statusCallback, this);

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
    if (x.size() > this->max_data_length)
    {
        x.pop_front();
        idx.pop_front();
    }
}

void QNode::irCallback(const sensor_msgs::RangeConstPtr &msg){
    int id = msg->radiation_type;
    switch (id) {
    case 1: {
        this->addData(this->ir1, this->ir1_t, msg->range);
        break;
    }
    case 2: {
        this->addData(this->ir2, this->ir2_t, msg->range);
        this->addData(this->right_dist, this->right_dist_t, double(msg->range) * (-1.0));
        break;
    }
    case 3: {
        this->addData(this->ir3, this->ir3_t, msg->range);
        break;
    }
    }
    std::stringstream ss;
    ss << "IR" << id << "Updated" << msg->range * (-1.0);
//    log(Info,ss.str());
    emit recieveIRData();
}

void QNode::statusCallback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("Recieved: [%s]", msg->data.c_str());
    std::stringstream ss;
    ss << "Recieved: " << msg->data.c_str();
    log(Info, ss.str());
}

void QNode::ultraCallback(const sensor_msgs::RangeConstPtr &msg){
    int id = msg->radiation_type;
    switch (id) {
    case 1: {
        this->addData(this->ul1, this->ul1_t, msg->range);
        break;
    }
    case 2: {
        this->addData(this->ul2, this->ul2_t, msg->range);
        break;
    }
    case 3: {
        this->addData(this->ul3, this->ul3_t, msg->range);
        break;
    }
    }
    std::stringstream ss;
    ss << "Ultra" << id << "Updated" << msg->range;
//    log(Info,ss.str());
    emit recieveUltraData();
}

void QNode::log( const LogLevel &level, const std::string &msg) {
    if (logging_model.rowCount() > 50)
        logging_model.removeRow(1);
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
