#include <ros/ros.h>
#include <gtest/gtest.h>

TEST(MathExpressions, operatorRecognition){
   EXPECT_EQ(5, (2+3)); }

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
