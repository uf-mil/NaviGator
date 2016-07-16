#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

#include <Arduino.h>

#include <Servo.h>

class Feeder
{
  private:

  public:
    static const int pin = 6;
    static Servo controller;
    static int goal;
    static int cur;

    static void init()
    {
      controller.attach(pin);
      set(1500);
      goal = 1500;
    }
    static void set(int x)
    {
      controller.writeMicroseconds(x);
      cur = x;
    }
    static void on()
    {
      goal = 2000;
    }
    static void off()
    {
      goal = 1500;
    }
    static void reverse()
    {
      goal = 1000;
    }
    static void run()
    {
      if (cur != goal)
      {
        if (goal == 1500) set(1500);
        else if (goal < 1500)
        {
          set(cur - 100);
        }
        else if (goal > 1500)
        {
          set(cur + 100);
        }
      }
      Serial.println(cur);
    }
};
int Feeder::goal = 0;
Servo Feeder::controller = Servo();
int Feeder::cur = 0;

class Shooter
{
  private:

  public:
    static const int pin = 5;
    static Servo controller;
    static int goal;
    static int cur;

    static void init()
    {
      controller.attach(pin);
      set(1500);
      goal = 1500;
    }
    static void set(int x)
    {
      controller.writeMicroseconds(x);
      cur = x;
    }
    static void on()
    {
      goal = 2000;
    }
    static void off()
    {
      goal = 1500;
    }
    static void run()
    {
      if (cur != goal)
      {
        if (cur < goal)
          set(cur+100);
        else set(goal);
      }
    }
};
int Shooter::goal = 0;
Servo Shooter::controller = Servo();
int Shooter::cur = 0;

class Comms
{
  private:
    //ROS
    ros::NodeHandle nh;
    std_msgs::String str_msg;
    //ros::Publisher chatter;
    ros::Subscriber<std_msgs::String> sub;

    static const int ON_CHAR = 105; //i
    static const int OFF_CHAR = 111; //o
    static const int FEED_ON_CHAR = 102; //f
    static const int FEED_OFF_CHAR = 115; //s
    static const int FEED_REVERSE_CHAR = 114; //r

    static void messageCallback(const std_msgs::String& str_msg)
    {
      String s = str_msg.data;
      if (s == "i") digitalWrite(13,HIGH);
      else if (s == "o") digitalWrite(13,LOW);
    }
  public:
    Comms() :
      str_msg(),
      sub("shooter_control",&messageCallback)
      //chatter("chatter", &str_msg)
    {

    }
    void init()
    {
      nh.initNode();
      nh.subscribe(sub);
	Serial.begin(9600);
      //nh.advertise(chatter);   
    }
    void run()
    {
      nh.spinOnce();
    }
};

Comms com;
void setup()
{
  pinMode(13,OUTPUT);
  com.init();
  Shooter::init();
  Feeder::init();
}

void loop()
{
  com.run();
  Shooter::run();
  Feeder::run();
  delay(100);
}
