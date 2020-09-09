/*
#include <gnss_dummy/gnss_dummy.h>
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

TEST_F(MyTestSuite, ellipsoidalHeight_low) {  
  GNSS gnss;
  int initial_value = 1;
  double value = gnss.ellipsoidalHeight(initial_value);
  ASSERT_EQ(value, sin(initial_value*42+100)*10) << "Value should be it's initial value plus 5";
}

TEST_F(MyTestSuite, ellipsoidalHeight_high) {
  GNSS gnss;
  int initial_value = 49;
  double value = gnss.ellipsoidalHeight(initial_value);
  ASSERT_EQ(value, sin(initial_value*42+100)*10) << "Value should be 0";
}




void gnssCallbacklatmin(const sensor_msgs::NavSatFix& gnss)
{
  EXPECT_GE(gnss.latitude, -90);
}
void gnssCallbacklongmin(const sensor_msgs::NavSatFix& gnss)
{
   EXPECT_GE(gnss.longitude, -180);
}
void gnssCallbacklatmax(const sensor_msgs::NavSatFix& gnss)
{
  EXPECT_LE(gnss.latitude, 90);
}
void gnssCallbacklongmax(const sensor_msgs::NavSatFix& gnss)
{
   EXPECT_LE(gnss.longitude, 180);
}


TEST_F(MyTestSuite, pub_lat_min)
{
  ros::NodeHandle nh;
  AnyHelper h;
  ros::Subscriber sub1 = nh.subscribe("fix", 1000, gnssCallbacklatmin);
  }
TEST_F(MyTestSuite, pub_long_min)
{
  ros::NodeHandle nh;
  AnyHelper h;
  ros::Subscriber sub1 = nh.subscribe("fix", 1000, gnssCallbacklongmin);
  }

TEST_F(MyTestSuite, pub_lat_max)
{
  ros::NodeHandle nh;
  AnyHelper h;
  ros::Subscriber sub1 = nh.subscribe("fix", 1000, gnssCallbacklatmax);
  }
TEST_F(MyTestSuite, pub_long_max)
{
  ros::NodeHandle nh;
  AnyHelper h;
  ros::Subscriber sub1 = nh.subscribe("fix", 1000, gnssCallbacklongmax);
  }

void gnssCallbackvalue1(const sensor_msgs::NavSatFix& gnss)
{
   ASSERT_EQ(gnss.header.seq, 12);
   ASSERT_EQ(gnss.longitude, 49.00);
   ASSERT_EQ(gnss.latitude, 60.00);

}


TEST_F(MyTestSuite, pub_value1) {
  GNSS gnss;
  ros::NodeHandle nh;
  AnyHelper h;
  ros::Subscriber sub1 = nh.subscribe("fix", 1000, gnssCallbackvalue1);
  int sequenceNumber= 12;
  double longitude = 49.00;
  double latitude = 60.00;
  gnss.message(sequenceNumber, longitude, latitude);
  
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "TestNode");
    
    testing::InitGoogleTest(&argc, argv);
    
    std::thread t([]{while(ros::ok()) ros::spin();});
    
    auto res = RUN_ALL_TESTS();
    
    ros::shutdown();
    
    return res;
}
*/
