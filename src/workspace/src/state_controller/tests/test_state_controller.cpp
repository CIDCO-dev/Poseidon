#include <state_controller/state_controller.h>
#include <ros/ros.h>
#include <gtest/gtest.h>


#include <thread>
#include <chrono>

class AnyMessage
{
};
typedef boost::shared_ptr<AnyMessage> AnyMessagePtr;
typedef boost::shared_ptr<AnyMessage const> AnyMessageConstPtr;

namespace ros
{
namespace message_traits
{

template<>
struct MD5Sum<AnyMessage>
{
  static const char* value() { return "*"; }
  static const char* value(const AnyMessage&) { return "*"; }
};

template<>
struct DataType<AnyMessage>
{
  static const char* value() { return "*"; }
  static const char* value(const AnyMessage&) { return "*"; }
};

template<>
struct Definition<AnyMessage>
{
};

}

namespace serialization
{
template<>
struct Serializer<AnyMessage>
{
  template<typename Stream, typename T>
  static void allInOne(Stream s, T t)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
};
}
}

struct AnyHelper
{
  AnyHelper()
  : count(0)
  {
  }
  void cb(const AnyMessageConstPtr& msg)
  {
    ++count;
  }

  uint32_t count;
};

class MyTestSuite : public ::testing::Test {
  public:
    MyTestSuite() {
    }
    ~MyTestSuite() {}
};

void stateCallbackvalue3(const state_controller_msg::State& state)
{


}


void stateCallbackvalue4(const state_controller_msg::State& state)
{
   //EXPECT_EQ(state.position.header.seq, 22);
   //ASSERT_EQ(state.position.header.stamp, time);
   //EXPECT_EQ(state.position.status.service, 1);
   //EXPECT_EQ(state.position.header.stamp.nsec, 0);
   //EXPECT_EQ(state.position.longitude, 66);
   //EXPECT_EQ(state.position.latitude, 99);
   //EXPECT_EQ(state.position.altitude, 123);
  EXPECT_EQ(state.vitals.header, 22);
  EXPECT_EQ(state.vitals.cputemp, 11);
  EXPECT_EQ(state.vitals.cpuload, 33);
  EXPECT_EQ(state.vitals.freeram, 55);
  EXPECT_EQ(state.vitals.freehdd, 44);
  EXPECT_EQ(state.vitals.uptime, 123456);
  EXPECT_EQ(state.vitals.vbat, 12.2);
  EXPECT_EQ(state.vitals.rh, 25);
  EXPECT_EQ(state.vitals.temp, 12);
  EXPECT_EQ(state.vitals.psi, 64);


}


TEST_F(MyTestSuite, pub_value1) {
  StateController stateControl;
  ros::NodeHandle nh;
//  AnyHelper h;
  ros::Time time=ros::Time::now();
  
  ros::Publisher pub1 = nh.advertise<sensor_msgs::NavSatFix>("fix", 1);
  sensor_msgs::NavSatFix msg1;
  msg1.header.seq=22;
  msg1.header.stamp=time;
  msg1.status.service = 1;
  msg1.header.stamp.nsec=0;
  msg1.longitude=66.66;
  msg1.latitude=99.66;
  msg1.altitude=123.66;
//  pub1.publish(msg1);
  
  ros::Publisher pub2 = nh.advertise<sensor_msgs::Imu>("pose", 1);
  sensor_msgs::Imu msg2;
  msg2.header.seq=22;
  msg2.header.stamp=time;
  msg2.orientation.w = 22;
  msg2.orientation.x = 33;
  msg2.orientation.y = 44;
  msg2.orientation.z = 55;
//  pub2.publish(msg2);


  ros::Publisher pub3 = nh.advertise<geometry_msgs::PointStamped>("depth", 1);
  geometry_msgs::PointStamped msg;
  msg.header.seq=22;
  msg.header.stamp=time;
  msg.point.x = 3.2;
  msg.point.y = 3.2;
  msg.point.z = 3.2;
//  pub3.publish(msg);

  ros::Subscriber sub3 = nh.subscribe("state", 1000, stateCallbackvalue3);


  ros::Publisher pub4 = nh.advertise<raspberrypi_vitals_msg::sysinfo>("vitals", 1);
  raspberrypi_vitals_msg::sysinfo msg4;
  msg4.header=22;
  msg4.cputemp=11;
  msg4.cpuload=33;
  msg4.freeram=55;
  msg4.freehdd=44;
  msg4.uptime=123456;
  msg4.vbat=12.2;
  msg4.rh=25;
  msg4.temp=12;
  msg4.psi=64;
  pub4.publish(msg4);

  ros::Subscriber sub4 = nh.subscribe("state", 1000, stateCallbackvalue4);
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "TestNode");
    
    testing::InitGoogleTest(&argc, argv);
    
    std::thread t([]{while(ros::ok()) ros::spin();});
    
    auto res = RUN_ALL_TESTS();
    
    ros::shutdown();
    
    return res;
}
