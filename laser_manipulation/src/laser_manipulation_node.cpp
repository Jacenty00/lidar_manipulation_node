
#include "laser_manipulation_node.hpp"
#include <ros/console.h>

LaserManipulationNode::LaserManipulationNode()
{
}

LaserManipulationNode::~LaserManipulationNode()
{
}

bool LaserManipulationNode::init()
{
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param("global_frame", global_frame_id_, std::string("map"));
  agents_poses_sub_ = nh.subscribe("/simulated_agents", 1, &LaserManipulationNode::agentsPosesCB, this);
  laser_scan_sub_ = nh.subscribe("scan", 5, &LaserManipulationNode::laserScanCB, this);
  virtual_obs_pub_ = nh.advertise<sensor_msgs::LaserScan>("scan_manipulated", 1, true);

  return true;
}

void LaserManipulationNode::agentsPosesCB(const pedsim_msgs::AgentStates::ConstPtr &msg)
{
  agents_ = msg->agent_states;
}
void LaserManipulationNode::laserScanCB(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  scan_ = *msg;
  std::string sensor_frame_id_ = msg->header.frame_id;
  tf::StampedTransform robot_gb;

  try
  {
    tf_listener_.lookupTransform(global_frame_id_, sensor_frame_id_, ros::Time(0.0), robot_gb);
  }
  catch (tf::TransformException &e)
  {

    // This can happen during startup, while some part of the localization chain is missing
    // If not, probably sensor_frame is missconfigured
    ROS_WARN_THROTTLE(2, "Cannot get tf %s -> %s: %s",
                      global_frame_id_.c_str(), sensor_frame_id_.c_str(), e.what());
  }

  tf::Transform robot_gb_inv = robot_gb.inverse();
  std::vector<boost::shared_ptr<Obstacle>> obstacles;

  for (unsigned int i = 0; i < agents_.size(); i++)
  {
    tf::Transform obs_abs_tf;
    tf::poseMsgToTF(agents_[i].pose, obs_abs_tf);
    tf::Transform obs_tf = robot_gb_inv * obs_abs_tf;
    boost::shared_ptr<Obstacle> new_obs(new Column(std::to_string(agents_[i].id), obs_tf, 0.2, 1.6));
    add(new_obs, obstacles);
  }

  int ray = 0;

  // Fire sensor "rays" and register closest hit, if any. We test hits_count_ consecutive objects; in most
  // cases 2 hits should be enough, although in some cases three or more ill-arranged obstacles will return
  // a wrong shortest distance (remember that obstacles are shorted by its closest distance to the robot)
  int hits_count = 2;
  for (double ray_theta = scan_.angle_min; ray_theta <= scan_.angle_max; ray_theta += scan_.angle_increment)
  {
    double rx = scan_.range_max * std::cos(ray_theta);
    double ry = scan_.range_max * std::sin(ray_theta);

    int hits = 0;
    // scan_.ranges[ray] = scan_.range_max;

    for (unsigned int i = 0; i < obstacles.size(); i++)
    {
      double distance;
      if (obstacles[i]->intersects(rx, ry, scan_.range_max, distance) == true)
      {
        // Hit; take the shortest distance until now and keep checking until we reach hits_count_
        scan_.ranges[ray] = std::min(scan_.ranges[ray], (float)distance);
        if (hits < hits_count)
        {
          hits++;
        }
        else
        {
          break; // enough hits
        }
      }
    }

    ray++;
  }

  virtual_obs_pub_.publish(scan_);
}

bool LaserManipulationNode::add(boost::shared_ptr<Obstacle> &new_obs, std::vector<boost::shared_ptr<Obstacle>> &obstacles)
{
  if ((new_obs->minHeight() > 0.0) || (new_obs->maxHeight() < 0.0))
  {
    ROS_ERROR_STREAM("Obstacle has invalid height");
    return false; // Obstacle above/below the virtual sensor (who is a 2D scan)
  }

  if (new_obs->distance() <= 0.0)
  {
    ROS_WARN("The robot is inside an obstacle??? Ignore it (distance: %f)", new_obs->distance());
    return false;
  }

  if (new_obs->distance() <= scan_.range_max)
  {
    std::vector<boost::shared_ptr<Obstacle>>::iterator it;
    for (it = obstacles.begin(); it != obstacles.end(); ++it)
    {
      if (new_obs->distance() <= (*it)->distance())
      {
        obstacles.insert(it, new_obs);
        return true;
      }
    }

    if ((obstacles.size() == 0) || it == obstacles.end())
    {
      // This obstacle is the first inserted or is the farthest from the robot
      obstacles.push_back(new_obs);
      return true;
    }
  }
  // ROS_WARN("Obstacle out of range (distance: %f)", new_obs->distance());
  return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_manipulator");

  LaserManipulationNode node;
  if (node.init() == false)
  {
    ROS_ERROR("%s initialization failed", ros::this_node::getName().c_str());
    return -1;
  }
  ROS_INFO("%s initialized", ros::this_node::getName().c_str());

  // ros::AsyncSpinner spinner(2); // Use 2 threads
  // spinner.start();
  ros::spin();
  ros::waitForShutdown();

  return 0;
}
