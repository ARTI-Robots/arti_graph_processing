/*
Created by clemens on 30.03.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <arti_graph_processing/vertex.h>
#include <arti_graph_processing/edge.h>
#include <cmath>
#include <utility>


namespace arti_graph_processing
{

// Required by map of weak_ptr's. While it is generally a bad idea to have map keys that can change (become invalid in
// this case) because it breaks map lookup, vertices should anyway only be deleted when the whole graph is deleted.
template<typename T>
bool operator<(const std::weak_ptr<T>& a, const std::weak_ptr<T>& b)
{
  return a.lock() < b.lock();
}


Vertex::Vertex(std::string name, geometry_msgs::PoseStamped pose, double max_distance)
  : name_(std::move(name)), pose_(std::move(pose)), max_distance_(max_distance)
{
}

std::vector<EdgePtr> Vertex::getOutgoingEdges() const
{
  return lockEdges(outgoing_edges_);
}

std::vector<EdgePtr> Vertex::getIncomingEdges() const
{
  return lockEdges(incoming_edges_);
}

double Vertex::getHeuristicsCosts(const VertexPtr& vertex) const
{
  const auto it = heuristic_costs_.find(vertex);

  if (it != heuristic_costs_.end())
  {
    return it->second;
  }

  if (use_euclidean_costs_)
  {
    double result = calculateEuclideanDistanceTo(*vertex);
    if (std::isfinite(result))
    {
      return result;
    }
  }

  return std::numeric_limits<double>::infinity();
}

void Vertex::setHeuristicsCosts(const VertexPtr& vertex, double costs)
{
  const auto it = heuristic_costs_.find(vertex);

  if (it != heuristic_costs_.end())
  {
    it->second = costs;
  }
  else
  {
    heuristic_costs_.insert(std::make_pair(vertex, costs));
  }
}

const std::string& Vertex::getName() const
{
  return name_;
}

const geometry_msgs::PoseStamped& Vertex::getPose() const
{
  return pose_;
}

double Vertex::getMaxDistance() const
{
  return max_distance_;
}

double Vertex::calculateEuclideanDistanceTo(const Vertex& other) const
{
  if (other.pose_.header.frame_id != pose_.header.frame_id)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  return std::hypot(other.pose_.pose.position.x - pose_.pose.position.x,
                    other.pose_.pose.position.y - pose_.pose.position.y);
}

void Vertex::setUseEuclideanCosts(bool use_euclidean_costs)
{
  use_euclidean_costs_ = use_euclidean_costs;
}

void Vertex::addOutgoingEdge(const EdgePtr& edge)
{
  outgoing_edges_.push_back(edge);
  this->setHeuristicsCosts(edge->getDestination(), edge->getCosts());
}

void Vertex::addIncomingEdge(const EdgePtr& edge)
{
  incoming_edges_.push_back(edge);
}

std::vector<EdgePtr> Vertex::lockEdges(const std::vector<EdgeWPtr>& edges)
{
  std::vector<EdgePtr> result;
  result.reserve(edges.size());
  for (const EdgeWPtr& edge : edges)
  {
    result.emplace_back(edge.lock());
  }
  return result;
}

}
