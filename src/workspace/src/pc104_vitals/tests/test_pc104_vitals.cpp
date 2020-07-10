#include <pc104_vitals/pc104_vitals.h>
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

void cputemp(const raspberrypi_vitals_msg::sysinfo& msg)
{
  EXPECT_GE(msg.cputemp, 0);
  EXPECT_LE(msg.cputemp, 60);
}

TEST_F(MyTestSuite, CPU_temp)
{
  ros::NodeHandle nh;
  AnyHelper h;
  ros::Subscriber sub2 = nh.subscribe("vitals", 1000, cputemp);
  }

void cpuload(const raspberrypi_vitals_msg::sysinfo& msg)
{
  EXPECT_GE(msg.cpuload, 0);
  EXPECT_LE(msg.cpuload, 60);
}

TEST_F(MyTestSuite, CPU_load)
{
  ros::NodeHandle nh;
  AnyHelper h;
  ros::Subscriber sub2 = nh.subscribe("vitals", 1000, cpuload);
  }

void freeram(const raspberrypi_vitals_msg::sysinfo& msg)
{
  EXPECT_GE(msg.freeram, 0);
  EXPECT_LE(msg.freeram, 100);
}

TEST_F(MyTestSuite, FREE_ram)
{
  ros::NodeHandle nh;
  AnyHelper h;
  ros::Subscriber sub2 = nh.subscribe("vitals", 1000, freeram);
  }


void freehdd(const raspberrypi_vitals_msg::sysinfo& msg)
{
  EXPECT_GE(msg.freehdd, 0);
  EXPECT_LE(msg.freehdd, 100);
}

TEST_F(MyTestSuite, FREE_hdd)
{
  ros::NodeHandle nh;
  AnyHelper h;
  ros::Subscriber sub2 = nh.subscribe("vitals", 1000, freehdd);
  }

int main(int argc, char** argv) {
    ros::init(argc, argv, "TestNode");
    
    testing::InitGoogleTest(&argc, argv);
    
    std::thread t([]{while(ros::ok()) ros::spin();});
    
    auto res = RUN_ALL_TESTS();
    
    ros::shutdown();
    
    return res;
}
