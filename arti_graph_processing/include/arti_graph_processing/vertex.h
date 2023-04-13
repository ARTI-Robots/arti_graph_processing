/*
Created by clemens on 30.03.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_GRAPH_PROCESSING_VERTEX_H
#define ARTI_GRAPH_PROCESSING_VERTEX_H

#include <arti_graph_processing/types.h>
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <string>

namespace arti_graph_processing
{

class Vertex : public std::enable_shared_from_this<Vertex>
{
public:
  Vertex(std::string name, geometry_msgs::PoseStamped pose, double max_distance);
  Vertex(Vertex&&) = default;
  virtual ~Vertex() = default;

  std::vector<EdgePtr> getOutgoingEdges() const;
  std::vector<EdgePtr> getIncomingEdges() const;

  double getHeuristicsCosts(const VertexPtr& vertex) const;

  void setHeuristicsCosts(const VertexPtr& vertex, double costs);

  const std::string& getName() const;

  const geometry_msgs::PoseStamped& getPose() const;

  double getMaxDistance() const;

  double calculateEuclideanDistanceTo(const Vertex& other) const;

  void setUseEuclideanCosts(bool use_euclidean_costs);

private:
  friend class Graph;

  void addOutgoingEdge(const EdgePtr& edge);
  void addIncomingEdge(const EdgePtr& edge);

  static std::vector<EdgePtr> lockEdges(const std::vector<EdgeWPtr>& edges);

  std::string name_;
  geometry_msgs::PoseStamped pose_;
  double max_distance_;

  std::vector<EdgeWPtr> outgoing_edges_;
  std::vector<EdgeWPtr> incoming_edges_;

  std::map<VertexWPtr, double> heuristic_costs_;

  bool use_euclidean_costs_ = true;
};

}

#endif //ARTI_GRAPH_PROCESSING_VERTEX_H
