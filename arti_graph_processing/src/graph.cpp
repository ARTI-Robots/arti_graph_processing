/*
Created by clemens on 30.03.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <arti_graph_processing/graph.h>
#include <arti_graph_processing/edge.h>
#include <arti_graph_processing/vertex.h>
#include <cmath>
#include <limits>
#include <ros/console.h>
#include <stdexcept>
#include <utility>
#include <angles/angles.h>
#include <tf/transform_datatypes.h>

namespace arti_graph_processing
{
Graph::Graph(std::string name, std::string frame_id)
  : name_(std::move(name)), frame_id_(std::move(frame_id))
{
}

const std::string& Graph::getName() const
{
  return name_;
}

const std::string& Graph::getFrameName() const
{
  return frame_id_;
}

bool Graph::empty() const
{
  return vertices_.empty();
}

void Graph::clear()
{
  vertices_.clear();
  vertices_by_name_.clear();
  edges_.clear();
}

void Graph::addVertex(const VertexPtr& vertex)
{
  if (!vertex->getName().empty() && !vertices_by_name_.emplace(vertex->getName(), vertex).second)
  {
    throw std::invalid_argument("tried to insert vertex with duplicate name");
  }

  vertices_.emplace(vertex);
}

const std::set<VertexPtr>& Graph::getVertices() const
{
  return vertices_;
}

VertexPtr Graph::getVertex(const std::string& name) const
{
  const auto it = vertices_by_name_.find(name);

  if (it != vertices_by_name_.end())
  {
    return it->second;
  }

  return nullptr;
}

VertexPtr Graph::getClosestVertex(const geometry_msgs::PoseStamped& pose) const
{
  double min_distance = std::numeric_limits<double>::max();
  double min_distance_min_yaw_diff = std::numeric_limits<double>::max();
  VertexPtr min_vertex;

  for (const auto& vertex : vertices_)
  {
    if (vertex->getPose().header.frame_id == pose.header.frame_id)
    {
      const double distance = calculateDistance(vertex->getPose().pose.position, pose.pose.position);

      const double yaw_diff = std::abs(
                angles::shortest_angular_distance(tf::getYaw(vertex->getPose().pose.orientation), 
                    tf::getYaw(pose.pose.orientation)));

      if (distance <= vertex->getMaxDistance() && 
          (distance < min_distance || (distance == min_distance && yaw_diff < min_distance_min_yaw_diff) ))
      {
        min_distance_min_yaw_diff = yaw_diff;
        min_distance = distance;
        min_vertex = vertex;
      }
    }
    else
    {
      ROS_WARN_STREAM("Graph: ignoring vertex with frame_id: '" << vertex->getPose().header.frame_id << "' in search");
    }
  }

  return min_vertex;
}

VertexPtr Graph::getClosestVertex(const geometry_msgs::PointStamped& position) const
{
  geometry_msgs::PoseStamped pose;
  pose.header = position.header;
  pose.pose.position = position.point;

  return getClosestVertex(pose);
}

void Graph::addEdge(const EdgePtr& edge)
{
  if (!edge || vertices_.count(edge->getDestination()) == 0 || vertices_.count(edge->getSource()) == 0)
  {
    throw std::invalid_argument("tried to insert invalid edge");
  }

  if (edges_.emplace(edge).second)
  {
    edge->getSource()->addOutgoingEdge(edge);
    edge->getDestination()->addIncomingEdge(edge);
  }
}

const std::set<EdgePtr>& Graph::getEdges() const
{
  return edges_;
}

double Graph::calculateDistance(const geometry_msgs::Point& a, const geometry_msgs::Point& b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

}
