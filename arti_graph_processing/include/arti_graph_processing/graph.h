/*
Created by clemens on 30.03.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_GRAPH_PROCESSING_GRAPH_H
#define ARTI_GRAPH_PROCESSING_GRAPH_H

#include <arti_graph_processing/types.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>
#include <memory>
#include <set>

namespace arti_graph_processing
{
class Graph
{
public:
  Graph(std::string name, std::string frame_id);
  Graph(Graph&&) = default;

  const std::string& getName() const;

  const std::string& getFrameName() const;

  bool empty() const;

  void clear();

  void addVertex(const VertexPtr& vertex);

  const std::set<VertexPtr>& getVertices() const;

  VertexPtr getVertex(const std::string& name) const;


  /**
   * @brief Gets the closest vertex for a given pose
   * @param[in] pose 
   * @return The closest vertex. If multiple vertex have the same distance, the vertex with closer orientation is returned.
   */
  VertexPtr getClosestVertex(const geometry_msgs::PoseStamped& pose) const;

  /**
   * @brief Gets the closest vertex for a given position
   * @param[in] position 
   * @return The closest vertex. If multiple vertex have the same distance, there is undefined behavior.
   */
  VertexPtr getClosestVertex(const geometry_msgs::PointStamped& position) const;

  void addEdge(const EdgePtr& edge);

  const std::set<EdgePtr>& getEdges() const;

private:
  static double calculateDistance(const geometry_msgs::Point& a, const geometry_msgs::Point& b);

  std::set<VertexPtr> vertices_;
  std::map<std::string, VertexPtr> vertices_by_name_;

  std::set<EdgePtr> edges_;

  std::string name_;

  std::string frame_id_;
};
}

#endif //ARTI_GRAPH_PROCESSING_GRAPH_H
