/*
Created by abuchegger on 2020-11-27.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <arti_graph_processing/graph_visualization_publisher.h>
#include <arti_graph_processing/edge.h>
#include <arti_graph_processing/graph.h>
#include <arti_graph_processing/vertex.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>

namespace arti_graph_processing
{

GraphVisualizationPublisher::GraphVisualizationPublisher(const ros::NodeHandle& node_handle, const std::string& topic)
  : node_handle_(node_handle), publisher_(node_handle_.advertise<visualization_msgs::MarkerArray>(topic, 1, true)),
    publisher_interpolated_(node_handle_.advertise<visualization_msgs::MarkerArray>(topic+"_interpolated", 1, true)),
    edge_increase_factor_(1.0), edge_max_number_increases_(1)
{
  cfg_server_.reset(
    new dynamic_reconfigure::Server<arti_graph_processing::GraphVisualizationConfig>(node_handle_));
  cfg_server_->setCallback(std::bind(&GraphVisualizationPublisher::reconfigure, this, std::placeholders::_1));
}

GraphVisualizationPublisher::GraphVisualizationPublisher(const ros::NodeHandle& node_handle, const std::string& topic,
                                                         double increase_factor = 1, size_t max_number_increases = 1)
  : node_handle_(node_handle), publisher_(node_handle_.advertise<visualization_msgs::MarkerArray>(topic, 1, true)),
  publisher_interpolated_(node_handle_.advertise<visualization_msgs::MarkerArray>(topic+"_interpolated", 1, true)),
    edge_increase_factor_(increase_factor), edge_max_number_increases_(max_number_increases)
{
  cfg_server_.reset(
    new dynamic_reconfigure::Server<arti_graph_processing::GraphVisualizationConfig>(node_handle_));
  cfg_server_->setCallback(std::bind(&GraphVisualizationPublisher::reconfigure, this, std::placeholders::_1));
}

void GraphVisualizationPublisher::reconfigure(const arti_graph_processing::GraphVisualizationConfig& new_config)
{
  cfg_ = new_config;

}

void GraphVisualizationPublisher::publish(const Graph& graph, bool interpolated)
{
  visualization_msgs::MarkerArray graph_markers;

  const auto& vertices = graph.getVertices();

  graph_markers.markers.reserve(std::max(vertices.size(), last_vertex_marker_count_) + 2);

  visualization_msgs::Marker vertex_poses_marker;
  if(cfg_.visualize_nodes)
  {

    vertex_poses_marker.ns = "vertex_poses";
    vertex_poses_marker.id = 0;
    vertex_poses_marker.action = visualization_msgs::Marker::ADD;
    vertex_poses_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    vertex_poses_marker.header.frame_id = graph.getFrameName();
    vertex_poses_marker.header.stamp = ros::Time::now();
    vertex_poses_marker.color.r = 235. / 255.;
    vertex_poses_marker.color.g = 235. / 255.;
    vertex_poses_marker.color.b = 52. / 255.;
    vertex_poses_marker.color.a = 1.;
    vertex_poses_marker.pose.orientation.w = 1.;
    vertex_poses_marker.scale.x = 0.25 * cfg_.scale;
    vertex_poses_marker.scale.y = 1.0; // scale in y direction for sphere == 1.0, ellipsoid for other values
    vertex_poses_marker.scale.z = 1.0; // scale in z direction for sphere == 1.0, ellipsoid for other values
    vertex_poses_marker.points.reserve(vertices.size());

    int text_id = 0;
    for (const auto& vertex: vertices)
    {
      visualization_msgs::Marker vertex_text;
      vertex_text.ns = "vertex_text";
      vertex_text.id = text_id++; //graph_markers.markers.size();
      vertex_text.action = visualization_msgs::Marker::ADD;
      vertex_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      vertex_text.header.frame_id = graph.getFrameName();
      vertex_text.header.stamp = ros::Time::now();
      vertex_text.color.r = 0.2;
      vertex_text.color.g = 0.2;
      vertex_text.color.b = 0.2;
      vertex_text.color.a = 1.;
      vertex_text.pose = vertex->getPose().pose;
      vertex_text.pose.position.x += 0.1;
      vertex_text.pose.position.y += 0.1;
      vertex_text.scale.z = 0.2  * cfg_.scale;
      vertex_text.text = vertex->getName();

      if(vertex->getPose().header.frame_id.empty() || vertex->getPose().header.frame_id.compare("") == 0)
      {
        ROS_ERROR("vertex with index [%d] has no frame! (interpolation %s)", text_id, interpolated ? "true":"false");
        ROS_ERROR_STREAM("Offending vertex pose: " << vertex->getPose());
      }
//      if(vertex->getName() == "V_106")
//      {
//        ROS_WARN("Visualize node [%s]", vertex->getName().c_str());
//      }
      vertex_poses_marker.points.push_back(vertex->getPose().pose.position);

      graph_markers.markers.push_back(vertex_text);
    }
    graph_markers.markers.push_back(vertex_poses_marker);
  }


  for (size_t i = vertices.size(); i < last_vertex_marker_count_; ++i)
  {
    visualization_msgs::Marker vertex_text;
    vertex_text.ns = "vertex_text";
    vertex_text.id = i;
    vertex_text.action = visualization_msgs::Marker::DELETE;

    graph_markers.markers.push_back(vertex_text);
  }
  last_vertex_marker_count_ = vertices.size();

  const auto& edges = graph.getEdges();

  visualization_msgs::Marker edges_marker;
  edges_marker.ns = "edges";
  edges_marker.id = 0;
  edges_marker.action = visualization_msgs::Marker::ADD;
  if(cfg_.arrow_visualization)
  {
    edges_marker.type = visualization_msgs::Marker::ARROW;
  }
  else
  {
    edges_marker.type = visualization_msgs::Marker::LINE_LIST;
  }
  edges_marker.header.frame_id = graph.getFrameName();
  edges_marker.header.stamp = ros::Time::now();
  edges_marker.color.r = 0.;
  edges_marker.color.g = 196. / 255.;
  edges_marker.color.b = 36. / 255.;
  edges_marker.color.a = 1.;
  edges_marker.pose.orientation.w = 1.;
  //for arrow x is diameter y head diameter  If scale.z is not zero, it specifies the head length
  //for line_list only x is used, controls the width of the line
  edges_marker.scale.x = 0.1 * cfg_.scale;
  if(cfg_.arrow_visualization)
  {
    edges_marker.scale.y = 0.4 * cfg_.scale;
    edges_marker.scale.z = .5 * cfg_.scale;
  }
  //edges_marker.points.reserve(2);//edges.size() * 2);

  visualization_msgs::Marker edges_text;
  edges_text.ns = "edges_text";
  edges_text.id = 0;
  edges_text.action = visualization_msgs::Marker::ADD;
  edges_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  edges_text.header.frame_id = graph.getFrameName();
  edges_text.header.stamp = ros::Time::now();
  edges_text.color.r = 0.2;
  edges_text.color.g = 0.2;
  edges_text.color.b = 0.2;
  edges_text.color.a = 1.;


  std::map<std::string, std::list<std::string>> source_list;

  for (const auto& edge: edges)
  {
    const arti_graph_processing::VertexPtr source = edge->getSource();
    const arti_graph_processing::VertexPtr destination = edge->getDestination();

    if(cfg_.one_direction_only)
    {
//      ROS_WARN("Visualization of Graph, enter one direction only ");
      auto iter = source_list.find(destination->getName());
      if (iter != source_list.end())
      {
        bool skip_edge = false;
        for (std::string old_sources: iter->second)
        {
          if (old_sources.compare(source->getName()) == 0)
          {
            skip_edge = true;
            break;
          }
        }
        if (skip_edge)
        {
          ROS_DEBUG_STREAM("skipping edge in graph for visualization");
          continue;
        }
      }
      else
      {
        std::list<std::string> sink;
        sink.push_back(destination->getName());
        source_list.insert(std::make_pair(source->getName(), sink));
      }
    }
    if (source && destination)
    {
      edges_marker.points.push_back(source->getPose().pose.position);
      edges_marker.points.push_back(destination->getPose().pose.position);

      double dist = std::hypot(source->getPose().pose.position.x - destination->getPose().pose.position.x,
                               source->getPose().pose.position.y - destination->getPose().pose.position.y);

      // alternatively normalize to initial cost!
      double cost_normalized = edge->getCosts() / dist;


      if (!std::isfinite(cost_normalized))  // set infinite edges to transparaent red
      {
        edges_marker.color.r = 1.;
        edges_marker.color.g = 0.;
        edges_marker.color.b = 0.;
        edges_marker.color.a = 0.33;
      }
      else if (cost_normalized < 1.01)   // cost according to distance in map = green
      {
        edges_marker.color.r = 0.;
        edges_marker.color.g = 1.;
        edges_marker.color.b = 0.;
        edges_marker.color.a = 1.;
      }
      else // color code between yellow and red
      {
        double cost_color_range =
          log(cost_normalized - 0.01) / log(edge_increase_factor_) / edge_max_number_increases_;

        edges_marker.color.r = 1.;
        edges_marker.color.g = std::min(1. - cost_color_range, 1.);
        edges_marker.color.b = 0.;
        edges_marker.color.a = 1.;
      }

      edges_text.id = edges_marker.id;
      double pos_offset;
      // write on right side of edge
      if (source->getPose().pose.position.y > destination->getPose().pose.position.y)
      {
        pos_offset = 0.2;
      }
      else
      {
        pos_offset = -0.2;
      }
      edges_text.pose.position.x =
        (source->getPose().pose.position.x + destination->getPose().pose.position.x) / 2. + pos_offset;
      edges_text.pose.position.y =
        (source->getPose().pose.position.y + destination->getPose().pose.position.y) / 2. + pos_offset;
      edges_text.scale.z = 0.2  * cfg_.scale;

      if (std::isfinite(edge->getCosts()))
      {
        edges_text.text = std::to_string(edge->getCosts());
      }
      else
      {
        edges_text.text = "inf";
      }

      graph_markers.markers.push_back(edges_marker);
      if(cfg_.edge_cost_visualization)
      {
        graph_markers.markers.push_back(edges_text);
      }
      edges_marker.id += 1;

    }

    edges_marker.points.clear();
  }

  if(interpolated)
  {
    publisher_interpolated_.publish(graph_markers);
  }
  else {
    publisher_.publish(graph_markers);
  }
}

}
