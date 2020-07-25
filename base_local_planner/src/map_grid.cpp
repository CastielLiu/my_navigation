/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <base_local_planner/map_grid.h>
#include <costmap_2d/cost_values.h>
using namespace std;

namespace base_local_planner{

  MapGrid::MapGrid()
    : size_x_(0), size_y_(0)
  {
  }

  MapGrid::MapGrid(unsigned int size_x, unsigned int size_y) 
    : size_x_(size_x), size_y_(size_y)
  {
    commonInit();
  }

  MapGrid::MapGrid(const MapGrid& mg){
    size_y_ = mg.size_y_;
    size_x_ = mg.size_x_;
    map_ = mg.map_;
  }

  void MapGrid::commonInit(){
    //don't allow construction of zero size grid
    ROS_ASSERT(size_y_ != 0 && size_x_ != 0);

    map_.resize(size_y_ * size_x_);

    //make each cell aware of its location in the grid
    for(unsigned int i = 0; i < size_y_; ++i){
      for(unsigned int j = 0; j < size_x_; ++j){
        unsigned int id = size_x_ * i + j;
        map_[id].cx = j;
        map_[id].cy = i;
      }
    }
  }

  size_t MapGrid::getIndex(int x, int y){
    return size_x_ * y + x;
  }

  MapGrid& MapGrid::operator= (const MapGrid& mg){
    size_y_ = mg.size_y_;
    size_x_ = mg.size_x_;
    map_ = mg.map_;
    return *this;
  }

  void MapGrid::sizeCheck(unsigned int size_x, unsigned int size_y){
    if(map_.size() != size_x * size_y)
      map_.resize(size_x * size_y);

    if(size_x_ != size_x || size_y_ != size_y){
      size_x_ = size_x;
      size_y_ = size_y;

      for(unsigned int i = 0; i < size_y_; ++i){
        for(unsigned int j = 0; j < size_x_; ++j){
          int index = size_x_ * i + j;
          map_[index].cx = j;
          map_[index].cy = i;
        }
      }
    }
  }


  //更新cell到路径的距离
  inline bool MapGrid::updatePathCell(MapCell* current_cell, MapCell* check_cell,
      const costmap_2d::Costmap2D& costmap)
  {
    //if the cell is an obstacle set the max path distance
    //如果单元格是障碍物，则设置为最大路径距离
    unsigned char cost = costmap.getCost(check_cell->cx, check_cell->cy);
    //当前单元格不在机器人轨迹上，且为障碍物
    if(! getCell(check_cell->cx, check_cell->cy).within_robot &&
        (cost == costmap_2d::LETHAL_OBSTACLE ||
         cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
         cost == costmap_2d::NO_INFORMATION))
    {
      check_cell->target_dist = obstacleCosts();
      return false;
    }

    //check_cell的距离为current_cell距离+1,因为check_cell是由current_cell膨胀所得
    double new_target_dist = current_cell->target_dist + 1;
    //if (new_target_dist < check_cell->target_dist) 
    {
      check_cell->target_dist = new_target_dist;
    }

    return true;
  }

  inline bool MapGrid::reUpdatePathCell(MapCell* current_cell, MapCell* check_cell,
      const costmap_2d::Costmap2D& costmap)
  {
    //check_cell的距离为current_cell距离+1,因为check_cell是由current_cell膨胀所得
    double new_target_dist = current_cell->target_dist + 1;
    if (new_target_dist < check_cell->target_dist) 
      check_cell->target_dist = new_target_dist;
    return true;
  }


  //reset the path_dist and goal_dist fields for all cells
  void MapGrid::resetPathDist(){
    for(unsigned int i = 0; i < map_.size(); ++i) {
      map_[i].target_dist = unreachableCellCosts();
      map_[i].target_mark = false;
      map_[i].within_robot = false;
    }
  }

  void MapGrid::adjustPlanResolution(const std::vector<geometry_msgs::PoseStamped>& global_plan_in,
      std::vector<geometry_msgs::PoseStamped>& global_plan_out, double resolution) {
    if (global_plan_in.size() == 0) {
      return;
    }
    double last_x = global_plan_in[0].pose.position.x;
    double last_y = global_plan_in[0].pose.position.y;
    global_plan_out.push_back(global_plan_in[0]);

    double min_sq_resolution = resolution * resolution;

    for (unsigned int i = 1; i < global_plan_in.size(); ++i) 
    {
      double loop_x = global_plan_in[i].pose.position.x;
      double loop_y = global_plan_in[i].pose.position.y;
      double sqdist = (loop_x - last_x) * (loop_x - last_x) + (loop_y - last_y) * (loop_y - last_y);
      if (sqdist > min_sq_resolution) 
      {
        int steps = ceil((sqrt(sqdist)) / resolution);
        // add a points in-between
        double deltax = (loop_x - last_x) / steps;
        double deltay = (loop_y - last_y) / steps;
        // TODO: Interpolate orientation
        for (int j = 1; j < steps; ++j) {
          geometry_msgs::PoseStamped pose;
          pose.pose.position.x = last_x + j * deltax;
          pose.pose.position.y = last_y + j * deltay;
          pose.pose.position.z = global_plan_in[i].pose.position.z;
          pose.pose.orientation = global_plan_in[i].pose.orientation;
          pose.header = global_plan_in[i].header;
          global_plan_out.push_back(pose);
        }
      }
      global_plan_out.push_back(global_plan_in[i]);
      last_x = loop_x;
      last_y = loop_y;
    }
  }

  //update what map cells are considered path based on the global_plan
  //根据全局路径更新局部代价地图各单元格的代价值
  void MapGrid::setTargetCells(const costmap_2d::Costmap2D& costmap,
      const std::vector<geometry_msgs::PoseStamped>& global_plan) 
  {
    //首先检查代价地图尺寸与MapGrid尺寸是都一致，否则进行调整
    sizeCheck(costmap.getSizeInCellsX(), costmap.getSizeInCellsY());

    bool started_path = false;

    queue<MapCell*> path_dist_queue;//全局路径距离队列

    //按照代价地图的分辨率调整路径点间隔，此处可优化，当分辨率相同时无需调整
    std::vector<geometry_msgs::PoseStamped> adjusted_global_plan;
    adjustPlanResolution(global_plan, adjusted_global_plan, costmap.getResolution());
    if (adjusted_global_plan.size() != global_plan.size()) {
      ROS_DEBUG("Adjusted global plan resolution, added %zu points", adjusted_global_plan.size() - global_plan.size());
    }
    unsigned int i;
    // put global path points into local map until we reach the border of the local map
    for (i = 0; i < adjusted_global_plan.size(); ++i) 
    {
      double g_x = adjusted_global_plan[i].pose.position.x;
      double g_y = adjusted_global_plan[i].pose.position.y;
      unsigned int map_x, map_y;
      if (costmap.worldToMap(g_x, g_y, map_x, map_y) && costmap.getCost(map_x, map_y) != costmap_2d::NO_INFORMATION) 
      {
        MapCell& current = getCell(map_x, map_y);
        current.target_dist = 0.0;  //距离置0
        current.target_mark = true; //标记置位
        path_dist_queue.push(&current);//MapCell压入队列
        started_path = true;
      } else if (started_path) {
          break;
      }
    }
    if (!started_path) {
      ROS_ERROR("None of the %d first of %zu (%zu) points of the global plan were in the local costmap and free",
          i, adjusted_global_plan.size(), global_plan.size());
      return;
    }

    computeTargetDistance(path_dist_queue, costmap);
  }

  //mark the point of the costmap as local goal where global_plan first leaves the area (or its last point)
 //局部目标: 全局路径首次离开局部代价地图的点，或者全局路径的最后一个点
  void MapGrid::setLocalGoal(const costmap_2d::Costmap2D& costmap,
      const std::vector<geometry_msgs::PoseStamped>& global_plan) 
  {
    sizeCheck(costmap.getSizeInCellsX(), costmap.getSizeInCellsY());

    int local_goal_x = -1;
    int local_goal_y = -1;
    bool started_path = false;

    //根据代价地图调整全局路径分辨率
    //更新局部代价地图带价值时已经调整了，此处可优化
    std::vector<geometry_msgs::PoseStamped> adjusted_global_plan;
    adjustPlanResolution(global_plan, adjusted_global_plan, costmap.getResolution());

    // skip global path points until we reach the border of the local map
    for (unsigned int i = 0; i < adjusted_global_plan.size(); ++i) 
    {
      double g_x = adjusted_global_plan[i].pose.position.x;
      double g_y = adjusted_global_plan[i].pose.position.y;
      unsigned int map_x, map_y;
      if (costmap.worldToMap(g_x, g_y, map_x, map_y) && costmap.getCost(map_x, map_y) != costmap_2d::NO_INFORMATION) 
      {
        local_goal_x = map_x;
        local_goal_y = map_y;
        started_path = true;
      } else {
        if (started_path) {
          break;
        }// else we might have a non pruned path, so we just continue
      }
    }
    if (!started_path) {
      ROS_ERROR("None of the points of the global plan were in the local costmap, global plan points too far from robot");
      return;
    }

    queue<MapCell*> path_dist_queue;
    if (local_goal_x >= 0 && local_goal_y >= 0) 
    {
      MapCell& current = getCell(local_goal_x, local_goal_y);
      costmap.mapToWorld(local_goal_x, local_goal_y, goal_x_, goal_y_);
      current.target_dist = 0.0;
      current.target_mark = true;
      path_dist_queue.push(&current);
    }

    computeTargetDistance(path_dist_queue, costmap);
  }

  //计算代价地图中所有栅格到路径的距离，作为路径规划的代价值
  //好像有问题，需优化
  void MapGrid::computeTargetDistance(queue<MapCell*>& dist_queue, const costmap_2d::Costmap2D& costmap){
    MapCell* current_cell;
    MapCell* check_cell;
    unsigned int last_col = size_x_ - 1;
    unsigned int last_row = size_y_ - 1;
    while(!dist_queue.empty())
    {
      current_cell = dist_queue.front();
      dist_queue.pop();
      //left
      if(current_cell->cx > 0)
      {
        check_cell = current_cell - 1;
        if(!check_cell->target_mark) //check_cell未访问
        {
          //mark the cell as visisted
          //标记cell已访问
          check_cell->target_mark = true;
          //更新check_cell的距离
          //更新check_cell的距离时，与已有值进行了对比，如果小于已有值则覆盖
          //但是，如果check_cell的target_dist已经有值(有效值)，也就是被访问过了,也就执行不到这了吧！
          if(updatePathCell(current_cell, check_cell, costmap)) 
            dist_queue.push(check_cell);
        }
        //即便已经被访问过，也应该再次尝试更新updatePathCell,对比是否距离更短，
        //但无需在加入队列
        else
          reUpdatePathCell(current_cell, check_cell, costmap);
      }

      //right
      if(current_cell->cx < last_col)
      {
        check_cell = current_cell + 1;
        if(!check_cell->target_mark)
        {
          check_cell->target_mark = true;
          if(updatePathCell(current_cell, check_cell, costmap))
            dist_queue.push(check_cell);
        }
        else
          reUpdatePathCell(current_cell, check_cell, costmap);
      }

      //up
      if(current_cell->cy > 0)
      {
        check_cell = current_cell - size_x_;
        if(!check_cell->target_mark)
        {
          check_cell->target_mark = true;
          if(updatePathCell(current_cell, check_cell, costmap)) 
            dist_queue.push(check_cell);
        }
        else
          reUpdatePathCell(current_cell, check_cell, costmap);
      }

      //down
      if(current_cell->cy < last_row)
      {
        check_cell = current_cell + size_x_;
        if(!check_cell->target_mark)
        {
          check_cell->target_mark = true;
          if(updatePathCell(current_cell, check_cell, costmap))
            dist_queue.push(check_cell);
        }
        else
          reUpdatePathCell(current_cell, check_cell, costmap);
      }
    }
  }

};