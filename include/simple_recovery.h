#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/costmap_model.h>
#include <string>

class SimpleRecovery : public nav_core::RecoveryBehavior
{
public:
  /**
   * @brief  Constructor, make sure to call initialize in addition to actually initialize the object
   */
  SimpleRecovery();

  /**
   * @brief  Initialization function for the SimpleRecovery recovery behavior
   * @param name Namespace used in initialization
   * @param tf (unused)
   * @param global_costmap (unused)
   * @param local_costmap A pointer to the local_costmap used by the navigation stack
   */
  void initialize(std::string name, tf::TransformListener* tf,
                  costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap);

  /**
   * @brief  Run the SimpleRecovery recovery behavior.
   */
  void runBehavior();

  /**
   * @brief  Destructor for the rotate recovery behavior
   */
  ~SimpleRecovery();

private:
  costmap_2d::Costmap2DROS* local_costmap_;
  bool initialized_;
  base_local_planner::CostmapModel* world_model_;
};
