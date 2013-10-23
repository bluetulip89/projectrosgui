// Code to follow corridor
  #include <ros.h>
  #include <ros/console.h>
  #include <ros/Time.h>
  #include <sensor_msgs/Range.h>
  #include <std_msgs/String.h>

#define ir_Left A0
#define ir_Right A4
#define max_allowable_diff 30
#define update_frequency 20

ros::NodeHandle nh;
sensor_msgs::Range irmsg;
  
std_msgs::String sensorstatus;
  
ros::Publisher ir_pub("IRSensor", &irmsg);
ros::Publisher status_pub("SensorStatus", &sensorstatus);

boolean G_debug = true;

unsigned long time_diff = 1/update_frequency;
unsigned long last_time;
boolean Done = false;

void setup()
{
  pinMode (ir_Left, INPUT);  
  pinMode (ir_Right, INPUT); 
  nh.initNode();
    nh.advertise(ir_pub);
    nh.advertise(status_pub);
//  Serial.begin(9600);
}



void sensor_read()
{   
    int lin_depthLeft[19] = {};
    int lin_depthRight[19] = {};
  
    for (int i = 0; i < 19; i++)
    {
         lin_depthLeft[i] = (analogRead(ir_Left) * (-1*0.35)) + 153;
         lin_depthRight[i] = (analogRead(ir_Right) * (-1*0.35)) + 153;      
    }
    
    
    int ir_Left = find_median (lin_depthLeft,19);
    int ir_Right = find_median (lin_depthRight,19);
//    if (G_debug==true)
//    {
//        Serial.print(ir_Left);
//        Serial.print(",");
//        Serial.println(ir_Right);
//    }
      irmsg.radiation_type = 1;
      irmsg.range = ir_Left;
      ir_pub.publish(&irmsg);
      irmsg.radiation_type = 2;
      irmsg.range = ir_Right;
      ir_pub.publish(&irmsg);
      
    int difference = ir_Left - ir_Right;
    if( abs(difference) > max_allowable_diff)
    {
        change_dir(difference);
    }
    
    
 }
 
 
int find_median(int array[],int siz)
{
    SortDec(array,siz);
    return array[(siz-1)/2];

}

// make into a library
void change_dir(int difference)
{
    if (difference > 0)
    {
      sensorstatus.data = "Move Left";
        status_pub.publish(&sensorstatus);
//        Serial.print("Move Left");
    }
    
    if (difference < 0)
    {
      sensorstatus.data = "Move Right";
        status_pub.publish(&sensorstatus);
        //Serial.print("Move Right");
    }
}


// make into a library
int FindMax(int array[], int start, int END)
{
  int maximum, location;
  
  maximum = array[start];
  location = start;
  for(int i = start + 1; i < END; i++)
  {
    if(array[i] > maximum)
    {
      maximum = array[i];
      location = i;
    }
  }
  
  return location;
}


// make into a library
void Swap(int array[], int a, int b)
{
  int temp, location;
  
  temp = array[a];
  array[a] = array[b];
  array[b] = temp;
}


// make into a library
void SortDec(int array[], int siz)
{
  int location;
  
  for(int i = 0; i < siz - 1; i++)
  {
    location = FindMax(array, i, siz);
    
    //Serial.print(location);
    Swap(array, i, location);
  }
}

void loop()
{
     if ((millis() - last_time) > time_diff)
     {
         last_time= millis();
         sensor_read();
     }
 
}



