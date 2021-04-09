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
  zone_w_subzones_.push_back("foodTray");
  initSubZones();

  problem_expert_->addPredicate(plansys2::Predicate("(robot_at r2d2 outdoor)"));
  graph_->add_node(ros2_knowledge_graph::Node{"world", "place"});
  graph_->add_node(ros2_knowledge_graph::Node{"r2d2", "robot"});
  problem_expert_->addPredicate(plansys2::Predicate("(object_picked r2d2 lemon)"));

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
  if (executor_client_->start_plan_execution()) 
  {
    RCLCPP_INFO(get_logger(), "Done!");
  }
}

void CleanUp_executor::PickObject_code_once()
{
  auto edges = graph_->get_edges_from_node_by_data("r2d2", "wanna_pick", "symbolic");
  RCLCPP_INFO(get_logger(), "PickObject_code_once!");
  for (auto edge : edges)
  {
    problem_expert_->setGoal(plansys2::Goal("(and(robot_at r2d2 foodTray))"));
  }
}

void CleanUp_executor::PlaceObject_code_iterative()
{
  
}

void CleanUp_executor::PlaceObject_code_once()
{

}
void CleanUp_executor::Init_code_iterative()
{

}

void CleanUp_executor::Init_code_once()
{
  // problem_expert_->setGoal(plansys2::Goal("(and(object_picked r2d2 sugar))"));
}

bool CleanUp_executor::Init_2_SearchObject()
{
  return true;
}

bool CleanUp_executor::PlaceObject_2_SearchObject()
{
  return false;
}

bool CleanUp_executor::PickObject_2_PlaceObject()
{
  return false;
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
