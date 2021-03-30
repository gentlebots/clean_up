// NOLINT (legal/copyright) 

#ifndef CLEANUPHFSM_H_
#define CLEANUPHFSM_H_

#include <string>

#include "std_msgs/msg/string.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace cascade_hfsm
{
class CleanUpHFSM : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  CleanUpHFSM();
  virtual ~CleanUpHFSM();

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state);

  virtual void SearchObject_code_iterative() {}
  virtual void SearchObject_code_once() {}
  virtual void PickObject_code_iterative() {}
  virtual void PickObject_code_once() {}
  virtual void PlaceObject_code_iterative() {}
  virtual void PlaceObject_code_once() {}
  virtual void Init_code_iterative() {}
  virtual void Init_code_once() {}

  virtual bool Init_2_SearchObject() {return false;}
  virtual bool PlaceObject_2_SearchObject() {return false;}
  virtual bool PickObject_2_PlaceObject() {return false;}
  virtual bool SearchObject_2_PickObject() {return false;}


  void tick();

protected:
  rclcpp::Time state_ts_;

private:
  void deactivateAllDeps();
  void SearchObject_activateDeps();
  void PickObject_activateDeps();
  void PlaceObject_activateDeps();
  void Init_activateDeps();


  static const int SEARCHOBJECT = 0;
  static const int PICKOBJECT = 1;
  static const int PLACEOBJECT = 2;
  static const int INIT = 3;


  int state_;

  std::string myBaseId_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr loop_timer_;
};

}  // namespace cascade_hfsm

#endif  // CLEANUPHFSM_H_
