// NOLINT (legal/copyright)

#include "CleanUpHFSM.hpp"

namespace cascade_hfsm
{
CleanUpHFSM::CleanUpHFSM()
: CascadeLifecycleNode("CleanUpHFSM"), state_(INIT), myBaseId_("CleanUpHFSM")
{
  declare_parameter("frequency");

  state_ts_ = now();
  state_pub_ = create_publisher<std_msgs::msg::String>("/" + myBaseId_ + "/state", 1);
}

CleanUpHFSM::~CleanUpHFSM()
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CleanUpHFSM::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  deactivateAllDeps();

  state_ = INIT;
  state_ts_ = now();

  Init_activateDeps();
  Init_code_once();


  double frequency = 5.0;
  get_parameter_or<double>("frequency", frequency, 5.0);

  loop_timer_ = create_wall_timer(
    std::chrono::duration<double, std::ratio<1>>(1.0 / frequency),
    std::bind(&CleanUpHFSM::tick, this));

  state_pub_->on_activate();
  return CascadeLifecycleNode::on_activate(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CleanUpHFSM::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  loop_timer_ = nullptr;

  return CascadeLifecycleNode::on_deactivate(previous_state);
}

void CleanUpHFSM::tick()
{
  std_msgs::msg::String msg;

  switch (state_) {
    case SEARCHOBJECT:
      SearchObject_code_iterative();

      msg.data = "SearchObject";
      state_pub_->publish(msg);

      if (SearchObject_2_PickObject()) {
        deactivateAllDeps();

        state_ = PICKOBJECT;
        state_ts_ = now();

        PickObject_activateDeps();
        PickObject_code_once();
      }
      break;
    case PICKOBJECT:
      PickObject_code_iterative();

      msg.data = "PickObject";
      state_pub_->publish(msg);

      if (PickObject_2_PlaceObject()) {
        deactivateAllDeps();

        state_ = PLACEOBJECT;
        state_ts_ = now();

        PlaceObject_activateDeps();
        PlaceObject_code_once();
      }
      break;
    case PLACEOBJECT:
      PlaceObject_code_iterative();

      msg.data = "PlaceObject";
      state_pub_->publish(msg);

      if (PlaceObject_2_SearchObject()) {
        deactivateAllDeps();

        state_ = SEARCHOBJECT;
        state_ts_ = now();

        SearchObject_activateDeps();
        SearchObject_code_once();
      }
      break;
    case INIT:
      Init_code_iterative();

      msg.data = "Init";
      state_pub_->publish(msg);

      if (Init_2_SearchObject()) {
        deactivateAllDeps();

        state_ = SEARCHOBJECT;
        state_ts_ = now();

        SearchObject_activateDeps();
        SearchObject_code_once();
      }
      break;
  }
}

void
CleanUpHFSM::deactivateAllDeps()
{
  remove_activation("vision");
  remove_activation("attention");
}

void
CleanUpHFSM::SearchObject_activateDeps()
{
  add_activation("vision");
  add_activation("attention");
}
void
CleanUpHFSM::PickObject_activateDeps()
{
  add_activation("attention");
}
void
CleanUpHFSM::PlaceObject_activateDeps()
{
  add_activation("attention");
}
void
CleanUpHFSM::Init_activateDeps()
{
}


}  // namespace cascade_hfsm
