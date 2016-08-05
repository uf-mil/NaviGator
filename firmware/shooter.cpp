#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

#include <Arduino.h>

#include <Servo.h>

const int SHOOTER_PIN = 3;
const int FEEDER_MOTOR_PIN = 5;

class Victor
{
  private:
    Servo controller;
    int goal;
    int cur;
    int pin;
    bool reversed;
    //Internal set command to write to controller PWM
    void _set(int s)
    {
      controller.writeMicroseconds(s);
      cur = s;
    }
  public:
    Victor(int p)
    {
      pin = p;
      controller = Servo();
      goal = 1500;
      reversed = false;
    }
    void init()
    {
      controller.attach(pin);
      _set(goal);
    }
    void set(int speed)
    {
      if (reversed) goal = map(speed,-100,100,2000,1000);
      else goal = map(speed,-100,100, 1000,2000);
    }
    int get()
    {
      if (reversed) return map(cur,1000,2000,100,-100);
      else return map(cur,1000,2000,-100,100);
    }
    void off()
    {
      set(0);
    }
    void on()
    {
      set(100);
    }
    void reverse()
    {
      set(-100);
    }
    void setReversed(bool rev)
    {
      reversed = rev;
    }
    //Should be called in each loop so PWM slowly ramps up, doesn't work otherwise
    void run()
    {
      if (cur != goal)
      {
        if (goal == 1500) _set(1500);
        else if (goal < 1500)
        {
          _set(cur - 100);
        }
        else if (goal > 1500)
        {
          _set(cur + 100);
        }
      }
    }
};
Victor shooter(SHOOTER_PIN);
//shooter.setReversed(true);


class Pololu
{
    private:
      int inA_pin;
      int inB_pin;
      int pwm_pin;
      int speed;
      Servo controller;
   public:
      Pololu(int a, int b, int pwm)
      {
        inA_pin = a;
        inB_pin = b;
        pwm_pin = pwm;
        controller = Servo();
      }
     void init()
     {
       pinMode(inA_pin,OUTPUT);
       pinMode(inB_pin,OUTPUT);
       controller.attach(pwm_pin);
     }
     void set(int s)
    {
      if (s == 0)
      {
        digitalWrite(inA_pin,LOW);
        digitalWrite(inB_pin,LOW);
        controller.writeMicroseconds(1500);
      }
      if(s < 0)
      {
        digitalWrite(inA_pin,HIGH);
        digitalWrite(inB_pin,LOW);
        //do direction stuff
        controller.writeMicroseconds(map(s,-100,0,1000,2000));
      } else if (s > 0)
      {
        digitalWrite(inA_pin,LOW);
        digitalWrite(inB_pin,HIGH);
        controller.writeMicroseconds(map(s,0,100,1000,2000));
        
      }
    }
void off()
{
set(0);
}
void on()
{
set(100);
}
void reverse()
{
set(-100);
}
};


class Feeder
{
  private:

  public:
    Pololu motor;
    Feeder (int a,int b, int pwm) :
      motor(a,b,pwm)
    {
  
    }
    void init()
    {
      motor.init();
    }
    void run()
    {
      //motor.run();
    }
};
Feeder feeder(8,9,5);

class AutoController
{
  private:
    static const unsigned long SPIN_UP_TIME = 1000; //Constant for time to spin up flywheels before feeding balls in
    static const unsigned long SHOOT_TIME = 12000; //Time to shoot all 4 balls once after they start being fed in
    static const unsigned long TOTAL_TIME = SPIN_UP_TIME + SHOOT_TIME;
    static const int FEED_SPEED = 50; //speed (out of 100) to set feeder motor to when feeding balls

    unsigned long start_shoot_time;
    bool auto_shoot;
  public:
		AutoController()
		{
			start_shoot_time = 0;
			auto_shoot = false;
		}
		void shoot()
		{
			auto_shoot = true;
			start_shoot_time = millis();
		}
		void cancel()
		{
			feeder.motor.off();
			shooter.off();
			auto_shoot = false;
		}
		void run()
		{
			if (auto_shoot)
			{
				unsigned long time_since_start = millis() - start_shoot_time;
				if (time_since_start < SPIN_UP_TIME) shooter.on();
				else if (time_since_start > SPIN_UP_TIME && time_since_start < TOTAL_TIME) feeder.motor.on(); //feeder.motor.set(FEED_SPEED);
				else if (time_since_start > TOTAL_TIME) cancel();
			}
		}	
};
AutoController autoController;

class Comms
{
  private:
    //ROS
    ros::NodeHandle nh;
    std_msgs::String str_msg;
    //ros::Publisher chatter;
    ros::Subscriber<std_msgs::String> sub;

    static void messageCallback(const std_msgs::String& str_msg)
    {
      String s = str_msg.data;
      if (s == "flyon")
        shooter.on();
      else if (s == "flyoff")
        shooter.off();
      else if (s == "feedon")
        feeder.motor.on();
      else if (s == "feedoff")
        feeder.motor.off();
      else if (s == "feedreverse")
        feeder.motor.reverse();
      else if (s == "shoot")
        autoController.shoot();
      else if (s == "cancel")
        autoController.cancel();
      else if (s == "ledon")
        digitalWrite(13,HIGH);
      else if (s == "ledoff")
        digitalWrite(13,LOW);
    }
  public:
    Comms() :
      str_msg(),
      sub("/shooter/control",&messageCallback)
      //chatter("chatter", &str_msg)
    {
      pinMode(13,OUTPUT);
    }
    void init()
    {
      nh.initNode();
      nh.subscribe(sub);
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
  shooter.setReversed(true);
  shooter.init();
  feeder.init();
  com.init();
}

void loop()
{
  com.run();
  autoController.run();
  shooter.run();
  feeder.run();
  delay(100);
}
