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
//  geometry_msgs::PoseStamped pose_copy(pose);

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

GraphPtr GraphLoader::interpolateGraph(GraphPtr graph, double max_interpolation_distance, const arti_ros_param::Param& root_param)
{
  ROS_INFO("start interpolating graph, input vertices count: %zu", graph->getVertices().size());
  std::string graph_name = graph->getName()+"_interpolated";

  if(graph->getFrameName().empty() )
  {
    ROS_ERROR_STREAM("Frame Id in input-graph "<< graph->getName() <<" missing: "<< graph->getFrameName());
    return GraphPtr();
  }
  GraphPtr graph_interpolated = std::make_shared<Graph>(Graph(graph_name,graph->getFrameName()));

  std::map<std::string, std::vector<std::string>> done_edges;

  for( auto e : graph->getEdges())
  {

    VertexPtr source_v = loadVertex(*graph_interpolated, e->getSource()->getName(), geometry_msgs::PoseStamped(e->getSource()->getPose()),
                                                           e->getSource()->getMaxDistance(), root_param );
    VertexPtr dest_v = loadVertex(*graph_interpolated,e->getDestination()->getName(), geometry_msgs::PoseStamped(e->getDestination()->getPose()),
                                                       e->getDestination()->getMaxDistance(), root_param);

    if(source_v->getPose().header.frame_id.empty() || source_v->getPose().header.frame_id == "")
    {
        ROS_ERROR("Source vertex has no frame id, (vertex %s)", source_v->getName().c_str());
//      source_v->getPose().header.frame_id = std::string(graph->getFrameName());
    }
      if(dest_v->getPose().header.frame_id.empty() || dest_v->getPose().header.frame_id == "")
      {
          ROS_ERROR("Destination vertex has no frame id, (vertex %s)", dest_v->getName().c_str());
//      source_v->getPose().header.frame_id = std::string(graph->getFrameName());
      }
    /*
     * check if the current edge was already visited, in reverse direction
     * */
//    auto res = done_edges.find(dest_v->getName());
//    std::map<std::string, std::vector<std::string>>::iterator result =
      bool visited = edgeVisited(source_v, dest_v, done_edges);
      if(visited)
      {
        ROS_DEBUG("found edge from [%s] to [%s] skip", source_v->getName().c_str(), dest_v->getName().c_str());
        continue;
      }

    EdgePtr rev_edge;

    for(auto edge_opp : e->getDestination()->getOutgoingEdges())
    {
      if(edge_opp->getDestination()->getName() == source_v->getName())
      {
        ROS_DEBUG("found reverse edge AKA bidirectional");
        rev_edge = edge_opp;
        break;
      }
    }
    double distance = getEuclideanDistance(e);
    double cost_full = e->getCosts();

    if(!graph_interpolated->getVertex(source_v->getName()))
    {
      ROS_DEBUG("Insert vertex (new source) [%s] now!", source_v->getName().c_str());
      graph_interpolated->addVertex(source_v);
    }
    else
    {
      source_v = graph_interpolated->getVertex(source_v->getName());
    }
    //in case vertex already exist in graph, use existing instance
    if(!graph_interpolated->getVertex(dest_v->getName()))
    {
      ROS_DEBUG("Insert vertex (old dest) [%s] now!", dest_v->getName().c_str());
      graph_interpolated->addVertex(dest_v);
    }
    else
    {
      ROS_DEBUG("use existing destination vertex from interpolated graph [%s]", dest_v->getName().c_str());
      dest_v = graph_interpolated->getVertex(dest_v->getName());
    }

    /*
     * interpolation of edge
     *
     * */
    if(distance > max_interpolation_distance)
    {

      int div = std::ceil(distance/max_interpolation_distance);

      double dx = -(source_v->getPose().pose.position.x - dest_v->getPose().pose.position.x);
      double dy = -(source_v->getPose().pose.position.y - dest_v->getPose().pose.position.y);
      double dz = dest_v->getPose().pose.position.z - source_v->getPose().pose.position.z;
      double s_yaw, d_yaw, t_yaw, s_roll, d_roll, t_roll, s_pitch, d_pitch, t_pitch;

      /*
       * make 3D rotation
       */
      // get source RPY
      tf::Quaternion bt_q;
      quaternionMsgToTF(source_v->getPose().pose.orientation, bt_q);
      tf::Matrix3x3(bt_q).getRPY( s_roll, s_pitch,s_yaw);
      //target RPY
      quaternionMsgToTF(dest_v->getPose().pose.orientation, bt_q);
      tf::Matrix3x3(bt_q).getRPY( t_roll, t_pitch,t_yaw);
      // delta RPY
      d_roll = -(s_roll - t_roll);
      d_pitch = -(s_pitch - t_pitch);
      d_yaw = -(s_yaw - t_yaw);


      VertexPtr prev_vertex = source_v;
      if(graph_interpolated->getVertex(source_v->getName()))
      {
        ROS_DEBUG("use existing source vertex from inerpolated graph [%s]", source_v->getName().c_str());
        prev_vertex = graph_interpolated->getVertex(source_v->getName());
      }

      for(int j=1; j <= div; j++)
      {
        // add new Vertex
        VertexPtr new_v;
        if(j < div)
        {
          geometry_msgs::PoseStamped next_pose;
          next_pose.header.frame_id = source_v->getPose().header.frame_id;
          next_pose.pose.position.x = source_v->getPose().pose.position.x + (dx * float(j) / float(div));
          next_pose.pose.position.y = source_v->getPose().pose.position.y + (dy * float(j) / float(div));
          next_pose.pose.position.z = source_v->getPose().pose.position.z + (dz * float(j) / float(div));
          //next_pose.pose.orientation = tf::createQuaternionFromYaw(tf::getYaw(source_v->getPose().pose.orientation) +
          //                                                         (dYaw * float(j) / float(div)));
          next_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
            (d_roll * float(j) / float(div)),
            (d_pitch * float(j) / float(div)),
            (d_yaw* float(j) / float(div)));

          std::string name = source_v->getName() +"_"+ dest_v->getName() +"_sub_" + std::to_string(j);

          new_v = loadVertex(*graph_interpolated,name, next_pose, prev_vertex->getMaxDistance(), root_param);
          if(!graph_interpolated->getVertex(new_v->getName()))
          {
            ROS_DEBUG("Insert interpolation vertex [%s] now!, source [%s] dest [%s]", name.c_str(), source_v->getName()
            .c_str(),
                     dest_v->getName().c_str());
            if(source_v->getName()=="V_106" || dest_v->getName() == "V_106")
            {
              ROS_WARN("add new vertex [%s] ", name.c_str());
            }
            graph_interpolated->addVertex(new_v);
          }
          else
          {
            ROS_ERROR("vertex already exits [%s]", name.c_str());
          }
        }
        else
        {
          //connect back to the old destination point
          new_v = dest_v;
        }

        // add new Edge
        double new_cost = cost_full / float(div);

        auto new_e = loadEdge(*graph_interpolated, prev_vertex, new_v, new_cost, root_param);
        graph_interpolated->addEdge(new_e);
        ROS_DEBUG("done inserting  interpolation edge");

        if(rev_edge)
        {
          // also consider the reverese edge here
          double cost_full_rev = rev_edge->getCosts();
          double new_cost_rev = cost_full_rev/float(div);
          ROS_DEBUG("Insert reverse edge");
          EdgePtr new_e_rev = loadEdge(*graph_interpolated,new_v,prev_vertex,  new_cost_rev, root_param);
          graph_interpolated->addEdge(new_e_rev);
        }
        prev_vertex = new_v;
      }
    }
    else
    {
      if(!graph_interpolated->getVertex(dest_v->getName()))
      {
        ROS_DEBUG("Insert vertex (old dest) [%s] now!", dest_v->getName().c_str());
        graph_interpolated->addVertex(dest_v);
      }
      ROS_DEBUG("add uninterpolated edge between [%s] and [%s]",source_v->getName().c_str() , dest_v->getName().c_str());
      EdgePtr new_e = loadEdge(*graph_interpolated,source_v, dest_v, e->getCosts(), root_param);
      graph_interpolated->addEdge(new_e);
      if(rev_edge)
      {
        ROS_DEBUG("add uninterpolated Reverse edge between [%s] and [%s]",dest_v->getName().c_str() , source_v->getName().c_str());
        EdgePtr new_e = loadEdge(*graph_interpolated,dest_v, source_v, rev_edge->getCosts(), root_param);
      }
    }
  }

  ROS_INFO("Resulting interpolating graph, contained vertices count: [%zu], edge count [%zu]",
           graph_interpolated->getVertices().size(), graph_interpolated->getEdges().size());
  return graph_interpolated;
}

double GraphLoader::getEuclideanDistance(EdgePtr e)
{
  VertexPtr s = e->getSource();
  VertexPtr d = e->getDestination();
  double dx = s->getPose().pose.position.x - d->getPose().pose.position.x;
  double dy = s->getPose().pose.position.y - d->getPose().pose.position.y;
  double dz = s->getPose().pose.position.z - d->getPose().pose.position.z;

  return std::hypot(dx,dy,dz);
}

bool GraphLoader::edgeVisited(VertexPtr source, VertexPtr dest,
                             std::map<std::string, std::vector<std::string>>& edge_map)
{
  auto res = edge_map.find(dest->getName());
  if(res != edge_map.end())
  {
    for(auto visited_source : res->second)
    {
      if(visited_source == "V_145" || visited_source == "V_106")
      {
        ROS_INFO("list of sources, from [%s] to [%s] skip", source->getName().c_str(), dest->getName().c_str());
      }
      if(visited_source == source->getName())
      {
        return true;
      }
    }
    res->second.push_back(source->getName());
    return false;
  }
  // unlikely case that this edge exists 2 times or is called twice
  res = edge_map.find(source->getName());
  if(res != edge_map.end())
  {
    for(auto visited_source : res->second)
    {
      if(visited_source == dest->getName())
      {
        return true;
      }
    }
    // Do nothing, edge might exist, we only look for duplicates
    res->second.push_back(dest->getName());
    return false;
  }

  // found neither source nor destination in the map, create new entry
  std::vector<std::string> destinations;
  destinations.push_back(dest->getName());
  edge_map.insert(std::make_pair(source->getName(), destinations));

  return false;
}

}
