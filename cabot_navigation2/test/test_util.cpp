#include <rclcpp/rclcpp.hpp>

#include <cabot_navigation2/util.hpp>
#include <cabot_navigation2/cabot_planner_util.hpp>

#include <gtest/gtest.h>
#include <stdlib.h>

namespace minimal_integration_test {
class TaskPlanningFixture : public testing::Test {
 public:
  TaskPlanningFixture()
      : node_(std::make_shared<rclcpp::Node>("basic_test"))
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "DONE WITH CONSTRUCTOR!!");
  }

  void SetUp() override {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "DONE WITH SETUP!!");
    l1 = Safety::Line(
        Safety::Point(0, 0),
        Safety::Point(10, 10)
    );
    l2 = Safety::Line(
        Safety::Point(0, 10),
        Safety::Point(10, 0)
    );
    l3 = Safety::Line(
        Safety::Point(0, 10),
        Safety::Point(4, 6)
    );
    l4 = Safety::Line(
        Safety::Point(20, 20),
        Safety::Point(30, 30)
    );
    l5 = Safety::Line(
        Safety::Point(10, 0),
        Safety::Point(20, 10)
    );
  }

  void TearDown() override {
    std::cout << "DONE WITH TEARDOWN" << std::endl;
  }

 protected:
  rclcpp::Node::SharedPtr node_;
  Safety::Line l1, l2, l3, l4, l5;
};

TEST_F(TaskPlanningFixture, IntersectTest) {
  std::cout << "TEST BEGINNING!!" << std::endl;
  EXPECT_TRUE(l1.intersect_segment(l2));
}

TEST_F(TaskPlanningFixture, NotIntersectTest) {
  EXPECT_FALSE(l1.intersect_segment(l3));
}

TEST_F(TaskPlanningFixture, OnLineSegmentTest) {
  EXPECT_FALSE(l1.intersect_segment(l1));
}

TEST_F(TaskPlanningFixture, OnLineTest) {
  EXPECT_FALSE(l1.intersect_segment(l4));
}

TEST_F(TaskPlanningFixture, ParalelTest) {
  EXPECT_FALSE(l1.intersect_segment(l5));
}

TEST_F(TaskPlanningFixture, ClosestPointTest) {
  auto l = Safety::Line(Safety::Point(132, 158), Safety::Point(136, 158));
  auto p = l.closestPoint(Safety::Point(134.01, 80));
  printf("####%.2f %.2f\n", p.x, p.y);
  EXPECT_LT(p.x, 140);
  EXPECT_GT(p.y, 150);
}

TEST_F(TaskPlanningFixture, ObstacleDistanceTest) {
  cabot_navigation2::Obstacle o1(0, 0, 254, 0, 0, false);
  cabot_navigation2::Obstacle o2(10, 10, 253, 0, 0, false);
  o2.lethal = &o1;
  cabot_navigation2::Point p1(10, 0);
  cabot_navigation2::Point p2(0, 10);
  cabot_navigation2::Point p3(-10, 0);
  cabot_navigation2::Point p4(0, -10);

  EXPECT_EQ(o2.distance(p1), 0);
  EXPECT_EQ(o2.distance(p2), 0);
  EXPECT_LT(o2.distance(p3), 0);
  EXPECT_LT(o2.distance(p4), 0);

  printf("####%.2f\n", o2.distance(p1));
  printf("####%.2f\n", o2.distance(p2));
  printf("####%.2f\n", o2.distance(p3));
  printf("####%.2f\n", o2.distance(p4));
}



}  // namespace minimal_integration_test

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
  return result;
}