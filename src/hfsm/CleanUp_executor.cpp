#include "CleanUp_executor.hpp"

CleanUp_executor::CleanUp_executor() 
{
  plansys2_node_ = rclcpp::Node::make_shared("cleanup_plansys2_node");
}

void CleanUp_executor::init()
{
  graph_ = std::make_shared<ros2_knowledge_graph::GraphNode>("clean_up");
  graph_->start();
  problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>(plansys2_node_);
  executor_client_ = std::make_shared<plansys2::ExecutorClient>(plansys2_node_);
  initKnowledge();

  //if (!executor_client_->start_plan_execution()) 
  //{
  //  RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
  //}
}

void CleanUp_executor::initKnowledge() 
{
  problem_expert_->addInstance(plansys2::Instance{"r2d2", "robot"});

  // Should be in a knowledge_extractor
  problem_expert_->addInstance(plansys2::Instance{"sugar_box", "object"});
  problem_expert_->addInstance(plansys2::Instance{"lemon", "object"});

  problem_expert_->addInstance(plansys2::Instance{"outdoor", "zone"});
  problem_expert_->addInstance(plansys2::Instance{"living_room", "zone"});
  problem_expert_->addInstance(plansys2::Instance{"near_lemon", "zone"});

  zone_w_subzones_.push_back("foodtray");
  initSubZones();

  problem_expert_->addPredicate(plansys2::Predicate("(robot_at r2d2 outdoor)"));
  graph_->add_node(ros2_knowledge_graph::Node{"world", "place"});
  graph_->add_node(ros2_knowledge_graph::Node{"r2d2", "robot"});
}

void CleanUp_executor::initSubZones()
{
  for (auto zone : zone_w_subzones_)
  {
    problem_expert_->addInstance(plansys2::Instance{zone, "zone"});
    for (int i = 0; i < n_subzones_; i++)
    {
      std::string subzone_id = zone + "_sz_" + std::to_string(i);
      problem_expert_->addInstance(plansys2::Instance{subzone_id, "subzone"});
      problem_expert_->addPredicate(plansys2::Predicate(
        "(subzone_at " + subzone_id + " " + zone +")"));
      problem_expert_->addPredicate(plansys2::Predicate("(free " + subzone_id + ")"));
    }
  }
}

std::string 
CleanUp_executor::statusToString(int8_t status) {
    switch (status)
    {
    case plansys2_msgs::msg::ActionExecutionInfo::NOT_EXECUTED:
      return "NOT_EXECUTED";
      break;
    case plansys2_msgs::msg::ActionExecutionInfo::EXECUTING:
      return "EXECUTING";
      break;
    case plansys2_msgs::msg::ActionExecutionInfo::FAILED:
      return "FAILED";
      break;
    case plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED:
      return "SUCCEEDED";
      break;
    case plansys2_msgs::msg::ActionExecutionInfo::CANCELLED:
      return "CANCELLED";
      break;
    default:
      return "UNKNOWN";
      break;
    }
}

bool
CleanUp_executor::getResult()
{
  RCLCPP_INFO(get_logger(), "========================= PLAN FINISHED =========================");
  auto result = executor_client_->getResult();
  if (result.has_value()) {
    RCLCPP_INFO_STREAM(get_logger(), "Plan succesful: " << result.value().success);
    for (const auto & action_info : result.value().action_execution_status) 
    {
      std::string args;
      rclcpp::Time start_stamp = action_info.start_stamp;
      rclcpp::Time status_stamp = action_info.status_stamp;
      for (const auto & arg : action_info.arguments) 
      {
        args = args + " " + arg;
      } 
      RCLCPP_INFO_STREAM(get_logger(), "Action: " << action_info.action << args << " " << statusToString(action_info.status) << " " << (status_stamp - start_stamp).seconds() << " secs");
    }
    return true;
  } else {
    RCLCPP_WARN(get_logger(), "No result for this plan");
  }
  return false;
}

void CleanUp_executor::SearchObject_code_iterative()
{
  RCLCPP_INFO(get_logger(), "SearchObject state");
}

void CleanUp_executor::SearchObject_code_once()
{

}

void CleanUp_executor::PickObject_code_iterative()
{
  RCLCPP_INFO(get_logger(), "PickObject_code_iterative!");
  //auto feedback = executor_client_->

  if (!executor_client_->execute_and_check_plan()) 
  {
    succesful_plan_ = getResult();
  }
}

void CleanUp_executor::PickObject_code_once()
{
  succesful_plan_ = false;
  auto edges = graph_->get_edges_from_node_by_data("r2d2", "wanna_pick", "symbolic");
  RCLCPP_INFO(get_logger(), "PickObject_code_once!");
  for (auto edge : edges)
  {
    problem_expert_->clearGoal();
    problem_expert_->setGoal(plansys2::Goal("(and(object_picked r2d2 lemon))"));
  }

  if (executor_client_->start_plan_execution()) 
  {
    RCLCPP_INFO(get_logger(), "Done!");
  }
}

void CleanUp_executor::PlaceObject_code_iterative()
{
  RCLCPP_INFO(get_logger(), "PlaceObject_code_iterative!");
  //auto feedback = executor_client_->

  if (!executor_client_->execute_and_check_plan()) 
  {
    succesful_plan_ = getResult();
  }
}

void CleanUp_executor::PlaceObject_code_once()
{
  succesful_plan_ = false;
  problem_expert_->clearGoal();
  problem_expert_->setGoal(plansys2::Goal("(and(object_at lemon foodtray))"));
  if (executor_client_->start_plan_execution()) 
  {
    RCLCPP_INFO(get_logger(), "Done!");
  }
}

void CleanUp_executor::Init_code_iterative()
{
 if (!executor_client_->execute_and_check_plan()) 
  {
    succesful_plan_ = getResult();
  }
}

void CleanUp_executor::Init_code_once()
{
  problem_expert_->setGoal(plansys2::Goal("(and(robot_at r2d2 near_lemon))"));
  if (executor_client_->start_plan_execution()) 
  {
    RCLCPP_INFO(get_logger(), "Done!");
  }
}

bool CleanUp_executor::Init_2_SearchObject()
{
  return succesful_plan_;
}

bool CleanUp_executor::PlaceObject_2_SearchObject()
{
  return false;
}

bool CleanUp_executor::PickObject_2_PlaceObject()
{
  return succesful_plan_;
}

bool CleanUp_executor::SearchObject_2_PickObject()
{
  // action Approach_object
  if (graph_->add_edge(ros2_knowledge_graph::Edge{"is_near", "symbolic", "r2d2", "lemon"}))
  {
    auto edges = graph_->get_edges_from_node_by_data("r2d2", "is_near", "symbolic");
    for (auto edge : edges)
    {
      graph_->add_edge(ros2_knowledge_graph::Edge{"wanna_pick", "symbolic", "r2d2", edge.target});
    }
    return true;
  }
  return false;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  CleanUp_executor node;
  node.init();
  rclcpp::Rate rate(1);
  while (rclcpp::ok()) {
    rate.sleep();
    rclcpp::spin_some(node.get_node_base_interface());
  }
  rclcpp::shutdown();
  return 0;
}
