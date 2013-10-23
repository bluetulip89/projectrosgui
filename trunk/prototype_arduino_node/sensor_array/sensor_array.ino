// Code to follow corridor

#define ir_Left A0
#define ir_Center A2
#define ir_Right A4
#define us_Center A5

#define stop Threshold 30
#define update_frequency 20

unsigned long time_diff = 1/update_frequency;
unsigned long last_time;
boolean G_debug = false;

void setup()
{
  pinMode (ir_Left, INPUT);  
  pinMode (ir_Center, INPUT); 
  pinMode (ir_Right, INPUT);  
  pinMode (us_Center, INPUT); 
  Serial.begin(9600);
}


void sensor_read()
{   
    int lin_depthLeft[19] = {};
    int lin_depthCenter[19] = {};
    int lin_depthRight[19] = {};
    int us_depthCenter[19] = {};
    
    for (int i = 0; i < 19; i++)
    {
         lin_depthLeft[i]   = (analogRead(ir_Left));
         lin_depthCenter[i] = (analogRead(ir_Center)); 
         lin_depthRight[i]  = (analogRead(ir_Right));           
         us_depthCenter[i]  = (analogRead(us_Center ));
    }
    
    
    int ir_Left = (find_median (lin_depthLeft,19) * (-1*0.35)) + 153;
    int ir_Center = (find_median (lin_depthCenter,19) * (-1*0.35)) + 153;
    int ir_Right = (find_median (lin_depthRight,19)* (-1*0.35)) + 153;
    int us_Center = (find_median (us_depthCenter,19)) *2.543;
    
    if (G_debug==true)
    {
        Serial.print(ir_Left);
        Serial.print(",");
        Serial.print(ir_Center);
        Serial.print(",");
        Serial.println(ir_Right);
        Serial.print(",");
        Serial.println(us_Center);
    }
    
    boolean right_blocked = ir_Right < 100;
    boolean left_blocked = ir_Left < 100;
    boolean center_blocked = ir_Center < 100 or us_Center < 200;
    
    
    if (ir_Right<50||ir_Left<50||ir_Center<50||us_Center<50)
    {
       Serial.println("Stop");
    }
    
    else if (center_blocked)
    {
        if (left_blocked && right_blocked)
        {
            Serial.println("Stop");
        }
        
        else if (left_blocked )
        {
            Serial.println("Swerve Right");
        }
        else if (right_blocked )
        {
            Serial.println("Swerve left");
        }
    }
    else if (left_blocked && right_blocked )
    {
           Serial.println("Stop"); 
    }
    
    else if (right_blocked )
    {
            Serial.println("Swerve left");
    }
    
    else if (left_blocked )
    {
            Serial.println("Swerve Right");
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
        Serial.print("Move Left");
    }
    
    if (difference < 0)
    {
        Serial.print("Move Right");
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


