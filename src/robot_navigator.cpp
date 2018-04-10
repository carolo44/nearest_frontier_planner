#include <nav_msgs/GridCells.h>

#include <nearest_frontier_planner/robot_navigator.h>

#include <set>
#include <map>

#include <limits>
#include <vector>
#include <utility>


RobotNavigator::RobotNavigator() {
  ros::NodeHandle robot_node;

  stop_server_ = robot_node.advertiseService(NAV_STOP_SERVICE,
      &RobotNavigator::receiveStop, this);
  pause_server_ = robot_node.advertiseService(NAV_PAUSE_SERVICE,
      &RobotNavigator::receivePause, this);

  ros::NodeHandle robot_node_pravite("~/");

  robot_node_pravite.param("map_frame", map_frame_, std::string("map"));
  robot_node_pravite.param("robot_frame", robot_frame_, std::string("robot"));
  robot_node_pravite.param("update_frequency", update_frequency_, 1.0);
  robot_node_pravite.param("explore_action_topic", explore_action_topic_,
      std::string(NAV_EXPLORE_ACTION));

  // Apply tf_prefix to all used frame-id's
  robot_frame_ = tf_listener_.resolve(robot_frame_);
  map_frame_ = tf_listener_.resolve(map_frame_);

  explore_action_server_ = new ExploreActionServer(explore_action_topic_,
      boost::bind(&RobotNavigator::receiveExploreGoal, this, _1), false);
  explore_action_server_->start();

  has_new_map_ = false;
  is_stopped_ = false;
  is_paused_ = false;
  //andy
  init_ = false;
  count_ = 0;
  interval_ = 16;

  goal_publisher_ = robot_node.advertise<geometry_msgs::PoseStamped>(
      "/move_base_simple/goal", 2);
  map_sub_ = robot_node.subscribe("/move_base_node/global_costmap/costmap", 1,
      &RobotNavigator::mapCallback, this);
  ros::Duration(1.0).sleep();
  scan_sub_ = robot_node.subscribe("/scan", 1,
      &RobotNavigator::scanCallback, this);
}

RobotNavigator::~RobotNavigator() {
  delete explore_action_server_;
}

bool RobotNavigator::receiveStop(std_srvs::Trigger::Request &req,
    std_srvs::Trigger::Response &res) {
  is_stopped_ = true;
  res.success = true;
  res.message = "Navigator received stop signal.";
  return true;
}

bool RobotNavigator::receivePause(std_srvs::Trigger::Request &req,
    std_srvs::Trigger::Response &res) {
  if ( is_paused_ ) {
    is_paused_ = false;
    res.success = false;
    res.message = "Navigator continues.";
  } else {
    is_paused_ = true;
    res.success = true;
    res.message = "Navigator pauses.";
  }
  return true;
}

bool RobotNavigator::preparePlan() {
  // Where am I?
  if ( !setCurrentPosition() ) return false;

  // Clear robot footprint in map

  return true;
}

void RobotNavigator::stop() {
  is_paused_ = false;
  is_stopped_ = false;
}

void RobotNavigator::receiveExploreGoal(
    const nearest_frontier_planner::ExploreGoal::ConstPtr &goal) {
  ros::Rate loop_rate(update_frequency_);
  ros::Rate long_rate(0.2);
  while ( true ) {
    // Check if we are asked to preempt
    if ( !ros::ok() || explore_action_server_->isPreemptRequested() || is_stopped_ ) {
      ROS_INFO("Exploration has been preempted externally.");
      explore_action_server_->setPreempted();
      stop();
      return;
    }

    unsigned int goal_x,goal_y;
    //to get the index of the goal
    if(init_)
    {
      goal_x = (goal_x_ - current_map_.getOriginX()) / current_map_.getResolution();
      goal_y = (goal_y_ - current_map_.getOriginY()) / current_map_.getResolution();
      current_map_.getIndex(goal_x,goal_y,goal_point_);
    }
       
    if(!init_ || count_ == interval_ || !current_map_.isFrontier(goal_point_)){
      count_ = 0;
      init_ = true;
    // Where are we now
    if ( !setCurrentPosition() ) {
      ROS_ERROR("Exploration failed, could not get current position.");
      explore_action_server_->setAborted();
      stop();
      return;
    }

    //goal_point_ = current_map_.getSize();
    if ( preparePlan() ) {
      //ROS_INFO("exploration: start = %u, end = %u.", start_point_, goal_point_);
      unsigned int x_start = 0, y_start = 0;
      current_map_.getCoordinates(x_start, y_start, start_point_);
      //ROS_INFO("start: x = %u, y = %u", x_start, y_start);
      unsigned int x_stop = 0, y_stop = 0;

      int inf_count = 0;
      int start_i = 0,end_i = 0;
      std::vector<double> sectors;
      
      tf::StampedTransform transform;
      try {
        tf_listener_.lookupTransform(map_frame_, robot_frame_, ros::Time(0), transform);
      } catch ( tf::TransformException ex ) {
        ROS_ERROR("Could not get robot position: %s", ex.what());
        return;
      }
      double theta = getYaw(transform.getRotation());
      bool sector_found = false;
   
      //to find whether there is a sector
      for(int i = 0;i < scan_.ranges.size();i++)
      {
          if(scan_.ranges[i] >= 10.0)
            inf_count++;
          else
          {
            if(inf_count > 8)
            {
              end_i = i;
              double direction = (start_i + end_i) / 2 * angle_increment_ + theta - M_PI;
              unsigned int index0,index1,index2;
              current_map_.getCoordinates(x_start, y_start, start_point_);
              current_map_.getIndex(x_start + 4 * std::cos(direction),y_start + 4 * std::sin(direction),index0);
              current_map_.getIndex(x_start + 8 * std::cos(direction),y_start + 8 * std::sin(direction),index1);
              current_map_.getIndex(x_start + 12 * std::cos(direction),y_start + 12 * std::sin(direction),index2);
              if(current_map_.isFrontier(index0) || current_map_.isFrontier(index1) || current_map_.isFrontier(index2))
              {
                if(scan_.ranges[start_i] == 10)
                {
                  if(start_i == 0)
                    start_i = scan_.ranges.size() - 1;
                  else
                    start_i--;
                }
                std::cout << "start_i " << scan_.ranges[start_i] << " end_i " << scan_.ranges[end_i] << std::endl; 
                if(scan_.ranges[end_i] > scan_.ranges[start_i])
                  sectors.push_back(/*direction*/end_i * angle_increment_ + theta - M_PI);
                else
                  sectors.push_back(/*direction*/start_i * angle_increment_ + theta - M_PI);
                  sector_found = true;
              }
            }
            inf_count = 0;
            start_i = i;
          }
      }
      if(!sector_found)  
      {
        int result = exploration_planner_.findExplorationTarget(&current_map_,
        start_point_, goal_point_);
        //bad_alloc detected(in nearest_frontier_planner.cpp)
        if(result == -10)
        {
          // Sleep remaining time
          ros::spinOnce();
          loop_rate.sleep();
          if ( loop_rate.cycleTime() > ros::Duration(1.0 / update_frequency_) )
            ROS_WARN("Missed desired rate of %.2fHz! Loop actually took %.4f seconds!",
            update_frequency_, loop_rate.cycleTime().toSec());

          continue;
        }
        current_map_.getCoordinates(x_stop,y_stop,goal_point_);
        goal_x_ = x_stop * current_map_.getResolution() + current_map_.getOriginX();
        goal_y_ = y_stop * current_map_.getResolution() + current_map_.getOriginY();
        interval_ = 16;
      }
      else
      {
        current_map_.getIndex(x_start + 100 * std::cos(sectors[0]),y_start + 100 * std::sin(sectors[0]),goal_point_);
        current_map_.getCoordinates(x_stop,y_stop,goal_point_);
        goal_x_ = x_stop * current_map_.getResolution() + current_map_.getOriginX();
        goal_y_ = y_stop * current_map_.getResolution() + current_map_.getOriginY();
        interval_ = 50;
      }

      double x_ = x_start * current_map_.getResolution() +
        current_map_.getOriginX();
      double y_ = y_start * current_map_.getResolution() +
        current_map_.getOriginY();
      //ROS_INFO("start: x = %f, y = %f", x_, y_);

      double x, y;
      bool no_vaild_goal = false;
      if ( goal_point_ == current_map_.getSize() ) {
        x = x_start * current_map_.getResolution() +
          current_map_.getOriginX();
        y = y_start * current_map_.getResolution() +
          current_map_.getOriginY();
      } else {
        current_map_.getCoordinates(x_stop, y_stop, goal_point_);
        //std::cout << "start: " << x_start << ", " << y_start << ";  stop: "
          //<< x_stop << ", " << y_stop << std::endl;

        if ( ((x_start - x_stop) * (x_start - x_stop) +
              (y_start - y_stop) * (y_start - y_stop)) <= 25 ) {
          x = x_start * current_map_.getResolution() +
            current_map_.getOriginX() +
            (longest_distance_ - 1) * cos(angles_);
          //std::cout << "x = " << x << ", boundx: " << current_map_.getBoundaryX() << std::endl;
          if ( x <= current_map_.getOriginX() )
            x = current_map_.getOriginX() + 1;
          else if ( x >= current_map_.getBoundaryX() )
            x = current_map_.getBoundaryX() - 1;

          y = y_start * current_map_.getResolution() +
            current_map_.getOriginY() +
            (longest_distance_ - 1) * sin(angles_);
          if ( y <= current_map_.getOriginY() )
            y = current_map_.getOriginY() + 1;
          else if ( y >= current_map_.getBoundaryX() )
            y = current_map_.getBoundaryX() - 1;

          no_vaild_goal = true;
          //ROS_INFO("longest_distance_: %f, angles_: %f", longest_distance_, angles_);
        } else {
          x = x_stop * current_map_.getResolution() +
            current_map_.getOriginX();
          y = y_stop * current_map_.getResolution() +
            current_map_.getOriginY();
        }
      }
      //ROS_INFO("goal: x = %f, y = %f", x, y);

      geometry_msgs::PoseStamped goal_base;
      goal_base.header.stamp = ros::Time::now();
      goal_base.header.frame_id = "map";
      goal_base.pose.position.x = x;
      goal_base.pose.position.y = y;
      goal_base.pose.orientation = tf::createQuaternionMsgFromYaw(0);
      goal_publisher_.publish(goal_base);

      if ( no_vaild_goal ) {
          ros::Rate long_rate(0.25 * 2 / longest_distance_);
          long_rate.sleep();
      }
    }
   }//andy
    has_new_map_ = false;
    count_++;
    // Sleep remaining time
    ros::spinOnce();
    loop_rate.sleep();
    if ( loop_rate.cycleTime() > ros::Duration(1.0 / update_frequency_) )
      ROS_WARN("Missed desired rate of %.2fHz! Loop actually took %.4f seconds!",
          update_frequency_, loop_rate.cycleTime().toSec());
  }
}

bool RobotNavigator::setCurrentPosition() {
  tf::StampedTransform transform;
  try {
    tf_listener_.lookupTransform(map_frame_, robot_frame_, ros::Time(0), transform);
  } catch ( tf::TransformException ex ) {
    ROS_ERROR("Could not get robot position: %s", ex.what());
    return false;
  }
  double world_x = transform.getOrigin().x();
  double world_y = transform.getOrigin().y();
  double world_theta = getYaw(transform.getRotation());

  unsigned int current_x = (world_x - current_map_.getOriginX()) /
    current_map_.getResolution();
  unsigned int current_y = (world_y - current_map_.getOriginY()) /
    current_map_.getResolution();
  unsigned int i;

  if ( !current_map_.getIndex(current_x, current_y, i) ) {
    if ( has_new_map_ || !current_map_.getIndex(current_x, current_y, i) ) {
      ROS_ERROR("Is the robot out of the map?");
      return false;
    }
  }
  start_point_ = i;
  return true;
}

void RobotNavigator::mapCallback(const nav_msgs::OccupancyGrid& global_map) {
  if ( !has_new_map_ ) {
    current_map_.update(global_map);
    current_map_.setLethalCost(80);
    has_new_map_ = true;
  }
}

void RobotNavigator::scanCallback(const sensor_msgs::LaserScan& scan) {
  //andy
  scan_ = scan;
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, scan.angle_min);
  tf::Stamped<tf::Quaternion> min_q(q, scan.header.stamp,
                                      scan.header.frame_id);
  q.setRPY(0.0, 0.0, scan.angle_min + scan.angle_increment);
  tf::Stamped<tf::Quaternion> inc_q(q, scan.header.stamp,
                                      scan.header.frame_id);
  angle_min_ = tf::getYaw(min_q);
  angle_increment_ = tf::getYaw(inc_q) - angle_min_;

  // wrapping angle to [-pi .. pi]
  angle_increment_ = fmod(angle_increment_ + 5*M_PI, 2*M_PI) - M_PI;
  //end andy
  
  /*double angle = scan.angle_min;
  int index = 0;

  int highest_score = -1;

  bool in_inf_range = false;
  double best_range = 0.0, best_angle = 0.0;
  int INDEX = 1;
  while ( angle <= scan.angle_max ) {
    int score;
    if ( !isinf(scan.ranges[index]) ) {
      if ( scan.ranges[index] < scan.range_max ) {
        if ( in_inf_range ) {
          score = scoreLine(angle - INDEX * scan.angle_increment, scan.ranges[index]);
          if ( score > highest_score ) {
            highest_score = score;
            best_range = scan.ranges[index];
            best_angle = angle - INDEX * scan.angle_increment;
          }
          
           *std::cout << "in range angle: " << angle - INDEX*scan.angle_increment
           *  << ", range: " << scan.ranges[index]
           *  << ", score: " << scoreLine(angle - INDEX*scan.angle_increment, scan.ranges[index]) << std::endl;
           

          in_inf_range = false;
        }
        score = scoreLine(angle, scan.ranges[index]);
        if ( score > highest_score ) {
          highest_score = score;
          best_range = scan.ranges[index];
          best_angle = angle;
        }
        
         *std::cout << "angle: " << angle
         *  << ", range: " << scan.ranges[index]
         *  << ", score: " << scoreLine(angle, scan.ranges[index]) << std::endl;
         
      } else {
        if ( index == 0 ) in_inf_range = true;
        if ( !in_inf_range ) {
          score = scoreLine(angle + INDEX * scan.angle_increment, scan.ranges[index - 1]);
          if ( score > highest_score ) {
            highest_score = score;
            best_range = scan.ranges[index - 1];
            best_angle = angle + INDEX * scan.angle_increment;
          }
          
           *std::cout << "in_inf_range angle: " << angle + INDEX * scan.angle_increment
           *  << ", range: " << scan.ranges[index - 1]
           *  << ", score: " << scoreLine(angle + INDEX*scan.angle_increment, scan.ranges[index - 1]) << std::endl;
           
          in_inf_range = true;
        }
      }
    } else {
      if ( index == 0 ) in_inf_range = true;

      if ( !in_inf_range ) {
        std::cout << "in_inf_range angle: " << angle
          << ", range: " << scan.ranges[index-1]
          << ", score: " << scoreLine(angle, scan.ranges[index-1]) << std::endl;
      }
    }

    index++;
    angle += scan.angle_increment;
  }

  longest_distance_ = best_range;
  angles_ = best_angle;
  ROS_INFO_STREAM("range: " << longest_distance_
      << ", angle: " << angles_
      << ", score: " << highest_score);*/

}

/*
static void walkAlongTheLongestRay(double distance, double angle,
    double cur_x, double cur_y,
    double &x, double &y) {
  x = cur_x + distance * cos(angle);
  y = cur_y + distance * sim(angle);
}
*/

enum QUADRANT {X_NEGTIVE, THRID, Y_NEGITIVE, FOURTH, X_POSITIVE, FIRST, Y_POSITIVE, SECOND};
int RobotNavigator::scoreLine(double angle, double range) {
    int score = 0;
    setCurrentPosition();
  //if ( preparePlan() ) {
    unsigned int x_start = 0, y_start = 0;
    current_map_.getCoordinates(x_start, y_start, start_point_);

    /*
    int x_stop = 0, y_stop = 0;
    current_map_.getCoordinates(x_stop, y_stop, stop_point_);
    */

    double x_start_d = x_start * current_map_.getResolution() +
            current_map_.getOriginX();
    double y_start_d = y_start * current_map_.getResolution() +
            current_map_.getOriginY();
    double x_stop_d = x_start_d + range * cos(angle);
    double y_stop_d = y_start_d + range * sin(angle);
    unsigned int x_stop = (x_stop_d - current_map_.getOriginX()) / current_map_.getResolution();
    unsigned int y_stop = (y_stop_d - current_map_.getOriginY()) / current_map_.getResolution();
    /*
     *std::cout << "start x: " << x_start << ", y: " << y_start
     *  << "; stop x: " << x_stop << ", y: " << y_stop << std::endl;
     */

    int x_step = 0, y_step = 0;
    if ( x_stop > x_start )
      x_step = 1;
    else if ( x_stop < x_start )
      x_step = -1;

    if ( y_stop > y_start )
      y_step = 1;
    else if ( y_stop < y_start )
      y_step = -1;

    enum QUADRANT quadrant = FIRST;

    if ( x_step == -1  && y_step == 0 )
      quadrant = X_NEGTIVE;
    else if ( x_step == -1 && y_step == -1 )
      quadrant = THRID;
    else if ( x_step == 0 && y_step == -1 )
      quadrant = Y_NEGITIVE;
    else if ( x_step == 1 && y_step == -1 )
      quadrant = FOURTH;
    else if ( x_step == 1 && y_step == 0 )
      quadrant = X_POSITIVE;
    else if ( x_step == 1 && y_step == 1 )
      quadrant = FIRST;
    else if ( x_step == 0 && y_step == 1 )
      quadrant = Y_POSITIVE;
    else if ( x_step == -1 && y_step == 1 )
      quadrant = SECOND;



    int x = x_start, y = y_start;
    bool reach_end = false;

    while ( true ) {
      if ( current_map_.getData(x, y) == -1 ) score++;
      int right_up, right_down, left_up, left_down;
      switch ( quadrant ) {

        case X_NEGTIVE:
          if ( x < x_stop ) reach_end = true;
          x += x_step;
          break;

        case THRID:
          if ( x < x_stop || y < y_stop ) reach_end = true;
          left_down = (x - int(x_start) + x_step) * (int(y_start) - int(y_stop)) - (int(x_start) - int(x_stop)) * (y - int(y_start) + y_step);
          left_up = (x - int(x_start) + x_step) * (int(y_start) - int(y_stop)) - (int(x_start) - int(x_stop)) * (y - int(y_start));
          right_down = (x - int(x_start)) * (int(y_start) - int(y_stop)) - (int(x_start) - int(x_stop)) * (y - int(y_start) + y_step);
          /*
           *std::cout << "x: " << x << ", y: " << y
           *  << ", left_down: " << left_down
           *  << ", left_up: " << left_up
           *  << ", right_down: " << right_down
           *  << std::endl;;
           */
          if ( left_down * right_down > 0 )
            x += x_step;
          else if ( left_down * left_up > 0 )
            y += y_step;
          else {
            x += x_step; y += y_step;
          }
          break;

        case Y_NEGITIVE:
          if ( y < y_stop ) reach_end = true;
          y += y_step;
          break;

        case FOURTH:
          if ( x > x_stop || y < y_stop ) reach_end = true;
          right_down = (x - int(x_start) + x_step) * (int(y_stop) - int(y_start)) - (int(x_stop) - int(x_start)) * (y - int(y_start) + y_step);
          right_up = (x - int(x_start) + x_step) * (int(y_stop) - int(y_start)) - (int(x_stop) - int(x_start)) * (y - int(y_start));
          left_down = (x - int(x_start)) * (int(y_stop) - int(y_start)) - (int(x_stop) - int(x_start)) * (y - int(y_start) + y_step);
          /*
           *std::cout << "x: " << x << ", y: " << y
           *  << ", right_down: " << right_down
           *  << ", right_up: " << right_up
           *  << ", left_down: " << left_down
           *  << std::endl;;
           */
          if ( right_down * left_down > 0 )
            x += x_step;
          else if ( right_down * right_up > 0 )
            y += y_step;
          else {
            x += x_step; y += y_step;
          }
          break;

        case X_POSITIVE:
          if ( x > x_stop ) reach_end = true;
          x += x_step;
          break;

        case FIRST:
          if ( x > x_stop || y > y_stop ) reach_end = true;
          right_up = (x - int(x_start) + x_step) * (int(y_stop) - int(y_start)) - (int(x_stop) - int(x_start)) * (y - int(y_start) + y_step);
          right_down = (x - int(x_start) + x_step) * (int(y_stop) - int(y_start)) - (int(x_stop) - int(x_start)) * (y - int(y_start));
          left_up = (x - int(x_start)) * (int(y_stop) - int(y_start)) - (int(x_stop) - int(x_start)) * (y - int(y_start) + y_step);
          /*
           *std::cout << "x: " << x << ", y: " << y
           *  << ", right_up: " << right_up
           *  << ", right_down: " << right_down
           *  << ", left_up: " << left_up
           *  << std::endl;;
           */
          if ( right_up * right_down > 0 )
            y += y_step;
          else if ( right_up * left_up > 0 )
            x += x_step;
          else {
            x += x_step; y += y_step;
          }
          break;

        case Y_POSITIVE:
          if ( y > y_stop ) reach_end = true;
          y += y_step;
          break;

        case SECOND:
          if ( x < x_stop || y > y_stop ) reach_end = true;
          left_up = (x - int(x_start) + x_step) * (int(y_start) - int(y_stop)) - (int(x_start) - int(x_stop)) * (y - int(y_start) + y_step);
          left_down = (x - int(x_start) + x_step) * (int(y_start) - int(y_stop)) - (int(x_start) - int(x_stop)) * (y - int(y_start));
          right_up = (x - int(x_start)) * (int(y_start) - int(y_stop)) - (int(x_start) - int(x_stop)) * (y - int(y_start) + y_step);
          /*
           *std::cout << "x: " << x << ", y: " << y
           *  << ", left_up: " << left_up
           *  << ", left_down: " << left_down
           *  << ", right_up: " << right_up
           *  << std::endl;;
           */
          if ( left_up * left_down > 0 )
            y += y_step;
          else if ( left_up * right_up > 0 )
            x += x_step;
          else {
            x += x_step; y += y_step;
          }
          break;
      }

      if ( reach_end ) break;

      /*
       *std::cout << "x: " << x
       *  << ", y: " << y
       *  << ", score: " << score << std::endl;
       */
    }

  //}

  return score;
  
}
