#include "nearest_frontier_planner/nearest_frontier_planner.h"
#include <math.h>
#include<iostream>

typedef std::multimap<double, unsigned int> Queue;
typedef std::pair<double, unsigned int> Entry;

int NearestFrontierPlanner::findExplorationTarget(GridMap* map,
    unsigned int start, unsigned int &goal) {
  // Create some workspace for the wavefront algorithm
  unsigned int map_size = map->getSize();
  double* plan;
  try{
    plan = new double[map_size];
  }catch(const std::bad_alloc&){
     return -10;
  }
  for ( unsigned int i = 0; i < map_size; i++ ) {
    plan[i] = -1;
  }

  // Initialize the queue with the robot position
  Queue queue;
  Entry start_point(0.0, start);
  queue.insert(start_point);
  plan[start] = 0;

  Queue::iterator next;
  double distance;
  double linear = map->getResolution();
  bool found_frontier = false;
  int cell_count = 0;

  // Do full search with weightless Dijkstra-Algorithm
  while ( !queue.empty() ) {
    cell_count++;
    // Get the nearest cell from the queue
    next = queue.begin();
    distance = next->first;
    unsigned int index = next->second;
    queue.erase(next);

    // Add all adjacent cells
    if ( map->isFrontier(index) ) {
      // We reached the border of the map, which is unexplored terrain as well:
      //andy to search deeper in the shadow
      unsigned int ind[5];

      ind[0] = index - 1;                // left
      ind[1] = index + 1;                // right
      ind[2] = index - map->getWidth();  // up
      ind[3] = index + map->getWidth();  // down
      ind[4] = index;                    //origin

      int curr_longest = 0;
      int decision = 4;
      double best_direction = 0.0;
      for(int i = 0; i < 5; i++){
        unsigned int curr_index = ind[i];
        if(!map->isFrontier(curr_index))
            continue;
        unsigned int start_x = 0,start_y = 0;
        //to get the coordinates of the position of the robot
        map->getCoordinates(start_x,start_y,start);
        unsigned int goal_x = 0, goal_y = 0;
        //to get the coordinates of the nearest frontier
        map->getCoordinates(goal_x,goal_y,curr_index);
        //to calculate the heading direction
        double direction = std::atan2(int(goal_y) - int(start_y),int(goal_x) - int(start_x));
        //to search deeper
        int step = 0;
        const int step_length = 5;
        unsigned int oldIndex = curr_index;
        unsigned int curr_x,curr_y;
        while(map->isFrontier(curr_index) && step < 10)
        {
            step++;
            oldIndex = curr_index;
            map->getCoordinates(curr_x,curr_y,oldIndex);
            map->getIndex(curr_x + step_length * std::cos(direction),curr_y + step_length * std::sin(direction),curr_index);
        }
        int curr_distance = (curr_x - start_x) * (curr_x - start_x) + (curr_y - start_y) * (curr_y - start_y);
        if(curr_distance > curr_longest){
            goal = oldIndex;
            curr_longest = curr_distance;
            best_direction = direction;
        }
      }//end andy

      /*if(init_)
      {
          unsigned int new_x,new_y,old_x,old_y;
          map->getCoordinates(new_x,new_y,goal);
          map->getCoordinates(old_x,old_y,old_goal_);
          unsigned int temp;
          map->getIndex((new_x + old_x) / 2,(new_y + old_y) / 2,temp);
          if(map->isFrontier(temp)){
              goal = temp;
              //std::cout << "sector found " << std::endl;
          }
      }
      else
          init_ = true;*/

      found_frontier = true;
      //original
      //goal = index;
      break;
    } else {
      unsigned int ind[4];

      ind[0] = index - 1;                // left
      ind[1] = index + 1;                // right
      ind[2] = index - map->getWidth();  // up
      ind[3] = index + map->getWidth();  // down

      for ( unsigned int it = 0; it < 4; it++ ) {
        unsigned int i = ind[it];
        if ( map->isFree(i) && plan[i] == -1 ) {
          queue.insert(Entry(distance+linear, i));
          plan[i] = distance+linear;
        }
      }
    }
  }

  ROS_DEBUG("Checked %d cells.", cell_count);
  delete[] plan;

  if ( found_frontier ) {
    return EXPL_TARGET_SET;
  } else {
    if ( cell_count > 50 )
      return EXPL_FINISHED;
    else
      return EXPL_FAILED;
  }
}
