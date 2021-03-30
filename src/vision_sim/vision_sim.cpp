// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "ros2_knowledge_graph/GraphNode.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/point.hpp"

class VisionSim : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  VisionSim()
  : CascadeLifecycleNode("vision")
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(),
      get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    sugar_.transform.translation.x = -1.01;
    sugar_.transform.translation.y = -0.1;
    sugar_.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setEuler(0.0,0.0,0.422);
    sugar_.transform.rotation.x =  q.x();
    sugar_.transform.rotation.y =  q.y();
    sugar_.transform.rotation.z =  q.z();
    sugar_.transform.rotation.w =  q.w();

    lemon_.transform.translation.x = -1.71;
    lemon_.transform.translation.y = -1.61;
    lemon_.transform.translation.z = 0.05;
    q.setEuler(0.05,0.04,0.04);
    lemon_.transform.rotation.x =  q.x();
    lemon_.transform.rotation.y =  q.y();
    lemon_.transform.rotation.z =  q.z();
    lemon_.transform.rotation.w =  q.w();

  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state)
  {
    graph_ = std::make_shared<ros2_knowledge_graph::GraphNode>("vision");
    graph_->start();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    timer_ = create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&VisionSim::loop, this));
    graph_->add_node(ros2_knowledge_graph::Node{"sugar_box", "object"});
    graph_->add_edge(ros2_knowledge_graph::Edge{
            tf_to_string(sugar_),
            "tf_static",
            "world" ,
            "sugar_box"
          });

    graph_->add_node(ros2_knowledge_graph::Node{"lemon", "object"});
    graph_->add_edge(ros2_knowledge_graph::Edge{
            tf_to_string(lemon_),
            "tf_static",
            "world" ,
            "lemon"
          });
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    timer_ = nullptr;

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  std::string 
  tf_to_string(geometry_msgs::msg::TransformStamped tf)
  {
    std::string x,y,z, y_str, p_str, r_str;
    x = std::to_string(tf.transform.translation.x);
    y = std::to_string(tf.transform.translation.y);
    z = std::to_string(tf.transform.translation.z);
    tf2::Quaternion tfq;
    tf2::fromMsg(tf.transform.rotation, tfq);
    double Y,P,R;
    tf2::Matrix3x3(tfq).getEulerYPR(Y,P,R);
    y_str = std::to_string(Y);
    p_str = std::to_string(P);
    r_str = std::to_string(R);
    std::string s = x + ":" + y + ":" + z + ":" + y_str + ":" + p_str + ":" + r_str;
    return s;
  }

  void loop()
  {
    
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<ros2_knowledge_graph::GraphNode> graph_;
  geometry_msgs::msg::TransformStamped sugar_, lemon_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisionSim>();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
