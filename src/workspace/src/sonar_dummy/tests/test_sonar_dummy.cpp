#include <sonar_dummy/sonar_dummy.h>
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



void sonarCallbackmin(const geometry_msgs::PointStamped& sonarMsgs)
{
  EXPECT_GE(sonarMsgs.point.z, 0);
}
void sonarCallbackmax(const geometry_msgs::PointStamped& sonarMsgs)
{
  EXPECT_LE(sonarMsgs.point.z, -180);
}


TEST_F(MyTestSuite, pub_min)
{
  ros::NodeHandle nh;
  AnyHelper h;
  ros::Subscriber sub3 = nh.subscribe("depth", 1000, sonarCallbackmin);
  }
TEST_F(MyTestSuite, pub_max)
{
  ros::NodeHandle nh;
  AnyHelper h;
  ros::Subscriber sub3 = nh.subscribe("depth", 1000, sonarCallbackmax);
  }




int main(int argc, char** argv) {
    ros::init(argc, argv, "TestNode");
    
    testing::InitGoogleTest(&argc, argv);
    
    std::thread t([]{while(ros::ok()) ros::spin();});
    
    auto res = RUN_ALL_TESTS();
    
    ros::shutdown();
    
    return res;
}
