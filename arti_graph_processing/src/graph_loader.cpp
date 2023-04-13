/*
Created by abuchegger on 2020-11-27.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <arti_graph_processing/graph_loader.h>
#include <arti_graph_processing/edge.h>
#include <arti_graph_processing/graph.h>
#include <arti_graph_processing/vertex.h>
#include <arti_ros_param/arti_ros_param.h>
#include <arti_ros_param/collections.h>
#include <limits>
#include <tf/transform_datatypes.h>
#include <utility>

namespace arti_graph_processing
{

GraphPtr GraphLoader::loadGraph(const XmlRpc::XmlRpcValue& root_node)
{
  return loadGraph(arti_ros_param::Param{{nullptr, {}}, root_node, arti_ros_param::LOG_TYPE_ERROR});
}

GraphPtr GraphLoader::loadGraph(const arti_ros_param::Param& root_param)
{
  const auto name = root_param["name"].decode<std::string>().get_value_or({});
  const auto frame_id = root_param["frame_id"].decode<std::string>();

  if (!frame_id)
  {
    root_param.handleTypeError("required parameter 'frame_id' is missing");
    return {};
  }

  GraphPtr graph = loadGraph(name, frame_id.value(), root_param);
  if (!loadVerticesAndEdges(*graph, root_param))
  {
    return {};
  }
  return graph;
}

std::vector<GraphPtr> GraphLoader::loadGraphVector(const XmlRpc::XmlRpcValue& root_node)
{
  return loadGraphVector(arti_ros_param::Param{{nullptr, {}}, root_node, arti_ros_param::LOG_TYPE_ERROR});
}

std::vector<GraphPtr> GraphLoader::loadGraphVector(const arti_ros_param::Param& root_param)
{
  if (!root_param.isArray())
  {
    root_param.handleTypeError("parameter is not an array");
    return {};
  }

  std::vector<GraphPtr> graphs;
  graphs.reserve(root_param.getSize());
  for (const arti_ros_param::Param& item_param : root_param)
  {
    GraphPtr graph = loadGraph(item_param);
    if (!graph)
    {
      return {};
    }
    graphs.emplace_back(std::move(graph));
  }
  return graphs;
}

std::map<std::string, GraphPtr> GraphLoader::loadGraphMap(const XmlRpc::XmlRpcValue& root_node)
{
  return loadGraphMap(arti_ros_param::Param{{nullptr, {}}, root_node, arti_ros_param::LOG_TYPE_ERROR});
}

std::map<std::string, GraphPtr> GraphLoader::loadGraphMap(const arti_ros_param::Param& root_param)
{
  if (root_param.isStruct())
  {
    std::map<std::string, GraphPtr> graphs;
    for (const arti_ros_param::Param& item_param : root_param)
    {
      GraphPtr graph = loadGraph(item_param);
      if (!graph)
      {
        return {};
      }
      graphs.emplace(item_param.getPath().key, std::move(graph));
    }
    return graphs;
  }
  else if (root_param.isArray())
  {
    std::map<std::string, GraphPtr> graphs;
    for (const arti_ros_param::Param& item_param : root_param)
    {
      GraphPtr graph = loadGraph(item_param);
      if (!graph)
      {
        return {};
      }
      const std::string& key = (!graph->getName().empty()) ? graph->getName() : item_param.getPath().key;
      graphs.emplace(key, std::move(graph));
    }
    return graphs;
  }

  root_param.handleTypeError("parameter is neither struct nor array");
  return {};
}

GraphPtr GraphLoader::loadGraph(
  const std::string& name, const std::string& frame_id, const arti_ros_param::Param& /*root_param*/)
{
  return std::make_shared<Graph>(name, frame_id);
}

bool GraphLoader::loadVerticesAndEdges(Graph& graph, const arti_ros_param::Param& root_param)
{
  for (const auto& vertex_cfg : root_param["vertices"])
  {
    const auto vertex = loadVertex(graph, vertex_cfg);
    if (!vertex)
    {
      return false;
    }
    graph.addVertex(vertex);
  }

  for (const auto& edge_cfg : root_param["edges"])
  {
    const auto edge = loadEdge(graph, edge_cfg);
    if (!edge)
    {
      return false;
    }
    graph.addEdge(edge);
  }

  return true;
}

boost::optional<geometry_msgs::PoseStamped> GraphLoader::loadPose(
  const Graph& graph, const arti_ros_param::Param& root_param)
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = root_param["frame_id"].decode<std::string>().get_value_or(graph.getFrameName());
  const auto yaw = root_param["yaw"].decode<double>();
  if (!(root_param["x"].decodeInto(pose.pose.position.x) && root_param["y"].decodeInto(pose.pose.position.y) && yaw))
  {
    root_param.handleTypeError("at least one of the required parameters 'x', 'y', and 'yaw' is missing");
    return boost::none;
  }
  root_param["z"].decodeInto(pose.pose.position.z);
  pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
    root_param["roll"].decode<double>().get_value_or(0.0),
    root_param["pitch"].decode<double>().get_value_or(0.0),
    yaw.value());
  return pose;
}

VertexPtr GraphLoader::loadVertex(
  const Graph& /*graph*/, const std::string& name, const geometry_msgs::PoseStamped& pose, double max_distance,
  const arti_ros_param::Param& /*root_param*/)
{
  return std::make_shared<Vertex>(name, pose, max_distance);
}

VertexPtr GraphLoader::loadVertex(const Graph& graph, const arti_ros_param::Param& root_param)
{
  const auto name = root_param["name"].decode<std::string>();
  if (!name)
  {
    root_param.handleTypeError("required parameter 'name' is missing");
    return nullptr;
  }

  const auto pose = loadPose(graph, root_param);
  if (!pose)
  {
    return nullptr;
  }

  return loadVertex(graph, name.value(), pose.value(),
                    root_param["max_distance"].decode<double>().get_value_or(std::numeric_limits<double>::infinity()),
                    root_param);
}

EdgePtr GraphLoader::loadEdge(
  const Graph& /*graph*/, const VertexPtr& source, const VertexPtr& destination, double costs,
  const arti_ros_param::Param& /*root_param*/)
{
  return std::make_shared<Edge>(source, destination, costs);
}

EdgePtr GraphLoader::loadEdge(const Graph& graph, const arti_ros_param::Param& root_param)
{
  const auto source_name = root_param["source"].decode<std::string>().get_value_or({});
  const VertexPtr source = graph.getVertex(source_name);
  if (!source)
  {
    root_param.handleTypeError("tried to load edge with nonexistent source '" + source_name + "'");
    return {};
  }

  const auto sink_param = root_param["sink"];
  const auto destination_name = sink_param.exists() ? sink_param.decode<std::string>().get_value_or({})
                                                    : root_param["destination"].decode<std::string>().get_value_or({});
  const VertexPtr destination = graph.getVertex(destination_name);
  if (!destination)
  {
    root_param.handleTypeError("tried to load edge with nonexistent destination '" + destination_name + "'");
    return {};
  }

  return loadEdge(graph, source, destination, root_param["costs"].decode<double>().get_value_or(0.0), root_param);
}

}
