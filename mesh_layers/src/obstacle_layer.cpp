/*
 *  Copyright 2020, Sebastian Pütz
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *
 */

/************************************************************************
 * 1. idea: get the faceIDs for possible lethal faces from ground_point_node
 * 2. compute corresponding vertices using lvr2
 * 3. flag those vertices as lethal
 * 4. update lethal map using in mesh layers (std::set<lvr2::VertexHandle> lethal_vertices);
 * 5. hope its working?
 * 6. seperate mesh navigation from reading data from hdf5 file (lukas bool)
*************************************************************************/

#include "mesh_layers/obstacle_layer.h"

#include <lvr2/algorithm/GeometryAlgorithms.hpp>
#include <lvr2/algorithm/NormalAlgorithms.hpp>
#include <pluginlib/class_list_macros.h>

#include <std_msgs/UInt32MultiArray.h>

std::vector<uint> faceIDs;

PLUGINLIB_EXPORT_CLASS(mesh_layers::ObstacleLayer, mesh_map::AbstractLayer)

namespace mesh_layers {

bool ObstacleLayer::readLayer() {
  ROS_INFO_STREAM("Try to read obstacles from map file...");
  auto obstacle_opt =
      mesh_io_ptr->getDenseAttributeMap<lvr2::DenseVertexMap<float>>(
          "obstacles");
  if (obstacle_opt) {
    ROS_INFO_STREAM("Successfully read obstacles from map file.");
    return computeLethals();
  }
  else
  {
    ROS_INFO_STREAM("No obstacles found, setting every vertex cost as 0 for obstacles");
    int cnt = 0; 
    for (auto vH : mesh_ptr->vertices())
    {
      obstacles.insert(vH, 0);      
      cnt++;
    }
    ROS_INFO_STREAM("Number of calc. vertices: " << cnt);

    return computeLethals();
  }

  return false;
}

bool ObstacleLayer::writeLayer() {
  if (mesh_io_ptr->addDenseAttributeMap(obstacles, "obstacles")) {
    ROS_INFO_STREAM("Saved obstacles to map file.");
    return true;
  } else {
    ROS_ERROR_STREAM("Could not save obstacles to map file!");
    return false;
  }

  return false;
}

void ObstacleLayer::groundPointsCallback(const std_msgs::UInt32MultiArray::ConstPtr& msg)
{
  faceIDs.clear();
  int cnt = 0;
  for(auto i : msg->data)
  {
    cnt++;
  }
  for (int i = 0; i < cnt; i++)
  {
    faceIDs.push_back(msg->data[i]);
    // printf("%d: faceID: %d \n", i, msg->data[i]);
  }
  if (ObstacleLayer::computeLethals() )
  {
    notifyChange();
    ROS_INFO_STREAM("refreshed obstacle layer");
  }
  ROS_INFO_STREAM("found " << cnt << " faceIDs for obstacle detection");
  // printf("size of vector %ld faceIDs \n", faceIDs.size() );
}


bool ObstacleLayer::computeLethals()
{
  ROS_INFO_STREAM("Compute lethals for \"" << layer_name << "\" (Obstacle Layer) with absolute threshold");
  if (mesh_ptr)
  {
    ros::WallTime t_calc_start;
    t_calc_start = ros::WallTime::now();

    auto const& mesh = *mesh_ptr;
      
    std::vector<lvr2::FaceHandle> faces;
    
    // clear lethals and costs, optimisation potential by resetting only changed faces/vertices
    // lethal_vertices.clear();
    // for (auto i : mesh_ptr->vertices())
    // {
    //   obstacles.insert(i, 0);
    // }

    // better solution, updating only existing obstacles
    set<lvr2::VertexHandle>::iterator itr;
    for (itr = lethal_vertices.begin(); itr != lethal_vertices.end(); itr++)
    {
      obstacles.insert(*itr, 0);
    }
    lethal_vertices.clear();

    face_id_sub = private_nh.subscribe("/ground_points/ground_face_ids", 1, &ObstacleLayer::groundPointsCallback, this);
    ros::spinOnce();

    for(int i = 0; i < faceIDs.size(); i++)
    {
      lvr2::FaceHandle faceI(faceIDs[i]);

      const auto vertices = mesh.getVerticesOfFace(faceI);
      // printf("calc vertices from face: %d \n", faceI);
      const lvr2::VertexHandle& a = vertices[0];
      const lvr2::VertexHandle& b = vertices[1];
      const lvr2::VertexHandle& c = vertices[2];
      // printf("Vertices: %d, %d, %d \n", a, b, c);
      obstacles.insert(a, 1);
      obstacles.insert(b, 1);
      obstacles.insert(c, 1);

      lethal_vertices.insert(a);
      lethal_vertices.insert(b);
      lethal_vertices.insert(c);
    }
    ros::WallTime t_calc_end = ros::WallTime::now();
    double obstacle_calc_duration = (t_calc_end - t_calc_start).toNSec() * 1e-6;
    ROS_INFO_STREAM("Calculation of obstacle layer took " << obstacle_calc_duration <<"ms");
  
    ROS_INFO_STREAM("Found " << lethal_vertices.size() << " lethal obstacle vertices.");
    return true;
  }
  else
  {
    ROS_INFO_STREAM("No mesh found for computing obstacles");
    return false;
  }
}

// maybe use this instead
void ObstacleLayer::updateLethal(std::set<lvr2::VertexHandle>& added_lethal, std::set<lvr2::VertexHandle>& removed_lethal)
{

}



float ObstacleLayer::threshold() { return config.threshold; }

bool ObstacleLayer::computeLayer() {
  ROS_INFO_STREAM("Computing obstacles...");
  
  return computeLethals();
}

lvr2::VertexMap<float> &ObstacleLayer::costs() { return obstacles; }

void ObstacleLayer::reconfigureCallback(mesh_layers::ObstacleLayerConfig &cfg, uint32_t level) {
  bool notify = false;

  ROS_INFO_STREAM("New obstacle layer config through dynamic reconfigure.");
  if (first_config) {
    config = cfg;
    first_config = false;
    return;
  }

  if(config.threshold != cfg.threshold)
  {
    computeLethals();
    notify = true;
  }

  if(notify) notifyChange();

  config = cfg;
}

bool ObstacleLayer::initialize(const std::string &name) {
  first_config = true;
  reconfigure_server_ptr = boost::shared_ptr<
      dynamic_reconfigure::Server<mesh_layers::ObstacleLayerConfig>>(
      new dynamic_reconfigure::Server<mesh_layers::ObstacleLayerConfig>(
          private_nh));

  config_callback =
      boost::bind(&ObstacleLayer::reconfigureCallback, this, _1, _2);
  reconfigure_server_ptr->setCallback(config_callback);
  return true;
}

} /* namespace mesh_layers */
