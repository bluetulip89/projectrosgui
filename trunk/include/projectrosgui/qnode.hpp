#ifndef projectrosgui_QNODE_HPP_
#define projectrosgui_QNODE_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>
#include <string>
#include <QThread>
#include <QStringListModel>

namespace projectrosgui {

struct pieceData{
    double x, t;
};

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

    // Data for the sensors
    QVector<double> ir1, ir2, ir3, ul1, ul2, ul3;
    QVector<double> ir1_t, ir2_t, ir3_t, ul1_t, ul2_t, ul3_t;
    double max_data_length;

    // Callback for Recieving Data
    void irCallback(const sensor_msgs::RangeConstPtr &msg);
    void ultraCallback(const sensor_msgs::RangeConstPtr &msg);
    void statusCallback(const std_msgs::String::ConstPtr &msg);

    QStringListModel logging_model;

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void recieveIRData();
    void recieveUltraData();
    void recieveStatus();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;

    // Message from
    ros::Subscriber irsensor;
    ros::Subscriber ultrasound;
    ros::Subscriber status;

    // Message for
    ros::Publisher command;

    void addData(QVector<double> & x, QVector<double> & idx, double data);
};

}  // namespace projectrosgui

#endif /* projectrosgui_QNODE_HPP_ */
