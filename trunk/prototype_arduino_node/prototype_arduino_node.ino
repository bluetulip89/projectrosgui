#include <ros.h>
#include <ros/console.h>
#include <ros/Time.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle nh;

sensor_msgs::Range irmsg;
sensor_msgs::Range ultramsg;

ros::Publisher ir_pub("IRSensor", &irmsg);
ros::Publisher ultra_pub("Ultrasound", &ultramsg);

void setup()
{
  nh.initNode();
  nh.advertise(ir_pub);
  nh.advertise(ultra_pub);
}

double tick = 0;

void loop()
{
  tick = tick + 0.1;
  for (int i = 1; i < 4; i++)
  {
    irmsg.radiation_type = i;
    irmsg.range = i * 0.5 * cos(tick / i * 4 + i) - 0.8;
   // irmsg.header.stamp = ros::Time::now();
    ir_pub.publish(&irmsg);
    ultramsg.radiation_type = i;
    ultramsg.range = i * 0.5 * sin(tick * 4 / i + i) + 0.8;
   // ultramsg.header.stamp = ros::Time::now();
    ultra_pub.publish(&ultramsg);
  }
  nh.spinOnce();
  delay(100);
}
