#ifndef JointSignalSubscriber_hh
#define JointSignalSubscriber_hh 1

#include <ros/subscriber.h>
#include <std_msgs/UInt16.h>

struct JointPinMapping {
  const char* joint;
  enum Type
    {
      TALON,
      SPIKE
    } type;
  unsigned int pin0, pin1;
};

/** Subscriber for arm-joint signals.    */
class JointSignalSubscriber : public ros::Subscriber_
{
public:
  std_msgs::UInt16 msg;

  JointSignalSubscriber(const JointPinMapping& mapping, int endpoint=rosserial_msgs::TopicInfo::ID_SUBSCRIBER)
    : mapping_(mapping),
      endpoint_(endpoint),
      servo_(NULL)
  {
    unsigned char alloc_size = strlen(mapping_.joint) + 9;
    topic_ = (const char*) malloc(alloc_size);
    snprintf((char*) topic_, alloc_size - 1, "%s/signal", mapping_.joint);
    if ( mapping.type == JointPinMapping::TALON )
      servo_ = new Servo;
  }

  ~JointSignalSubscriber()
  {
    free((char*) topic_);
    if ( servo_ )
        delete servo_;
    servo_ = NULL;
  }

  void attach()
  {
    if ( servo_ )
      {
        servo_->attach(mapping_.pin0);
        servo_->writeMicroseconds(1500);
      }
    else
      {
        pinMode(mapping_.pin0, OUTPUT);
        pinMode(mapping_.pin1, OUTPUT);
      }
  }
  
  virtual void callback(unsigned char* data){
    msg.deserialize(data);
    if ( servo_ )
      servo_->writeMicroseconds(msg.data);
    else if ( msg.data < 1500 )
      {
        digitalWrite(mapping_.pin0, HIGH);
        digitalWrite(mapping_.pin1, LOW);
      }
    else if ( msg.data > 1500 )
      {
        digitalWrite(mapping_.pin0, LOW);
        digitalWrite(mapping_.pin1, HIGH);
      }
    else
      {
        digitalWrite(mapping_.pin0, LOW);
        digitalWrite(mapping_.pin1, LOW);
      }
  }

  virtual const char * getMsgType(){ return this->msg.getType(); }
  virtual const char * getMsgMD5(){ return this->msg.getMD5(); }
  virtual int getEndpointType(){ return endpoint_; }

private:
  const JointPinMapping& mapping_;
  int endpoint_;
  Servo* servo_;
};

#endif  /* JointSignalSubscriber_hh */
