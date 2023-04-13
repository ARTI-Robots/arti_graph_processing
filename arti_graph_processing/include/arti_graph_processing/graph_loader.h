/*
Created by abuchegger on 2020-11-27.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef ARTI_GRAPH_PROCESSING_GRAPH_LOADER_H
#define ARTI_GRAPH_PROCESSING_GRAPH_LOADER_H

#include <arti_graph_processing/types.h>
#include <arti_ros_param/forward_declarations.h>
#include <boost/optional.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <map>
#include <string>
#include <vector>
#include <xmlrpcpp/XmlRpcValue.h>

namespace arti_graph_processing
{

class GraphLoader
{
public:
  virtual ~GraphLoader() = default;

  virtual GraphPtr loadGraph(const XmlRpc::XmlRpcValue& root_node);

  virtual GraphPtr loadGraph(const arti_ros_param::Param& root_param);

  virtual std::vector<GraphPtr> loadGraphVector(const XmlRpc::XmlRpcValue& root_node);

  virtual std::vector<GraphPtr> loadGraphVector(const arti_ros_param::Param& root_param);

  virtual std::map<std::string, GraphPtr> loadGraphMap(const XmlRpc::XmlRpcValue& root_node);

  virtual std::map<std::string, GraphPtr> loadGraphMap(const arti_ros_param::Param& root_param);

protected:
  virtual GraphPtr loadGraph(
    const std::string& name, const std::string& frame_id, const arti_ros_param::Param& root_param);

  virtual bool loadVerticesAndEdges(Graph& graph, const arti_ros_param::Param& root_param);

  virtual VertexPtr loadVertex(const Graph& graph, const arti_ros_param::Param& root_param);

  virtual VertexPtr loadVertex(
    const Graph& graph, const std::string& name, const geometry_msgs::PoseStamped& pose, double max_distance,
    const arti_ros_param::Param& root_param);

  virtual boost::optional<geometry_msgs::PoseStamped> loadPose(
    const Graph& graph, const arti_ros_param::Param& root_param);

  virtual EdgePtr loadEdge(const Graph& graph, const arti_ros_param::Param& root_param);

  virtual EdgePtr loadEdge(
    const Graph& graph, const VertexPtr& source, const VertexPtr& destination, double costs,
    const arti_ros_param::Param& root_param);
};

}

#endif //ARTI_GRAPH_PROCESSING_GRAPH_LOADER_H
