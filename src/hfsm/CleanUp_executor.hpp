#ifndef CLEANUP_EXECUTOR_H_
#define CLEANUP_EXECUTOR_H_

#include "CleanUpHFSM.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"

#include "ros2_knowledge_graph_msgs/msg/graph.hpp"
#include "ros2_knowledge_graph_msgs/msg/node.hpp"
#include "ros2_knowledge_graph_msgs/msg/edge.hpp"
#include "ros2_knowledge_graph_msgs/msg/graph_update.hpp"

#include "ros2_knowledge_graph/GraphNode.hpp"
#include "ros2_knowledge_graph/graph_utils.hpp"

//#include "ros2_kg_tf_plugin/TFLayer.hpp"

class CleanUp_executor: public cascade_hfsm::CleanUpHFSM
{
public:
	CleanUp_executor();

	void SearchObject_code_iterative();
  void SearchObject_code_once();
  void PickObject_code_iterative();
  void PickObject_code_once();
  void PlaceObject_code_iterative();
  void PlaceObject_code_once();
  void Init_code_iterative();
  void Init_code_once();

  bool Init_2_SearchObject();
  bool PlaceObject_2_SearchObject();
  bool PickObject_2_PlaceObject();
  bool SearchObject_2_PickObject();
	void init();

private:
  bool getResult();
  std::string statusToString(int8_t status);
	void initKnowledge();
  void initSubZones();

  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;

	ros2_knowledge_graph::GraphNode * graph_;
  rclcpp::Node::SharedPtr node_;
  std::vector<std::string> zone_w_subzones_;

  int n_subzones_ = 6;
  bool succesful_plan_ = false;
};

#endif
