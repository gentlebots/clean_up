#ifndef CLEANUP_EXECUTOR_H_
#define CLEANUP_EXECUTOR_H_

#include "CleanUpHFSM.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "ros2_knowledge_graph/GraphNode.hpp"
#include "ros2_kg_tf_plugin/TFLayer.hpp"

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
	void initKnowledge();

	std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
	std::shared_ptr<ros2_knowledge_graph::GraphNode> graph_;
  rclcpp::Node::SharedPtr plansys2_node_;

};

#endif
