#include <imu_dummy/imu_dummy.h>
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



void imuCallbackzm(const sensor_msgs::Imu& imuMsgs)
{
  EXPECT_GE(imuMsgs.orientation.z, -180);
}
void imuCallbackxm(const sensor_msgs::Imu& imuMsgs)
{
   EXPECT_GE(imuMsgs.orientation.x, -180);
}
void imuCallbackym(const sensor_msgs::Imu& imuMsgs)
{
  EXPECT_GE(imuMsgs.orientation.y, -180);
}
void imuCallbackzx(const sensor_msgs::Imu& imuMsgs)
{
   EXPECT_LE(imuMsgs.orientation.z, 180);
}
void imuCallbackxx(const sensor_msgs::Imu& imuMsgs)
{
  EXPECT_LE(imuMsgs.orientation.x, 180);
}
void imuCallbackyx(const sensor_msgs::Imu& imuMsgs)
{
   EXPECT_LE(imuMsgs.orientation.y, 180);
}


TEST_F(MyTestSuite, pub_z_min)
{
  ros::NodeHandle nh;
  AnyHelper h;
  ros::Subscriber sub2 = nh.subscribe("pose", 1000, imuCallbackzm);
  }
TEST_F(MyTestSuite, pub_x_min)
{
  ros::NodeHandle nh;
  AnyHelper h;
  ros::Subscriber sub2 = nh.subscribe("pose", 1000, imuCallbackxm);
  }

TEST_F(MyTestSuite, pub_y_min)
{
  ros::NodeHandle nh;
  AnyHelper h;
  ros::Subscriber sub2 = nh.subscribe("pose", 1000, imuCallbackym);
  }
TEST_F(MyTestSuite, pub_z_max)
{
  ros::NodeHandle nh;
  AnyHelper h;
  ros::Subscriber sub2 = nh.subscribe("pose", 1000, imuCallbackzx);
  }

TEST_F(MyTestSuite, pub_x_max)
{
  ros::NodeHandle nh;
  AnyHelper h;
  ros::Subscriber sub2 = nh.subscribe("pose", 1000, imuCallbackxx);
  }
TEST_F(MyTestSuite, pub_y_max)
{
  ros::NodeHandle nh;
  AnyHelper h;
  ros::Subscriber sub2 = nh.subscribe("pose", 1000, imuCallbackyx);
  }




int main(int argc, char** argv) {
    ros::init(argc, argv, "TestNode");
    
    testing::InitGoogleTest(&argc, argv);
    
    std::thread t([]{while(ros::ok()) ros::spin();});
    
    auto res = RUN_ALL_TESTS();
    
    ros::shutdown();
    
    return res;
}
