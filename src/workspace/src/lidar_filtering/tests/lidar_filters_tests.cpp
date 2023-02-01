#include <gtest/gtest.h>
#include "lidarFilters.h"
#include <thread>

class LidarFiltersTestSuite : public ::testing::Test {
  public:

    LidarFiltersTestSuite() {}
    ~LidarFiltersTestSuite() {}
    
    protected:
    	ros::NodeHandle n;
};

TEST_F(LidarFiltersTestSuite, horizontalAngleFilter) {
	
	ASSERT_TRUE(Filters::horizontalAngleFilter(0.707, 0.707, 0.0, 90.0));
	ASSERT_FALSE(Filters::horizontalAngleFilter(0.707, 0.707, 90.0, 270.0));


}

TEST_F(LidarFiltersTestSuite, distanceFilter) {
	
	ASSERT_TRUE(Filters::distanceFilter(0.707, 0.707, 0.0, 1.1, 0.0));
	ASSERT_TRUE(Filters::distanceFilter(0.707, 0.707, 0.0, 0.0, 0.5));
	
	ASSERT_FALSE(Filters::distanceFilter(0.707, 0.707, 0.0, 0.5, 1.1));


}

int main(int argc, char **argv) {

    ros::init(argc, argv, "TestLidarFilters");

    testing::InitGoogleTest(&argc, argv);

    std::thread t([]{ros::spin();}); // let ros spin in its own thread

    auto res = RUN_ALL_TESTS();

    ros::shutdown(); // this will cause the ros::spin() to return
    t.join();

    return res;
}
