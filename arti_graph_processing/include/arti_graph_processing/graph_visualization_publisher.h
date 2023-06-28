/*
Created by abuchegger on 2020-11-27.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef ARTI_GRAPH_PROCESSING_GRAPH_VISUALIZATION_PUBLISHER_H
#define ARTI_GRAPH_PROCESSING_GRAPH_VISUALIZATION_PUBLISHER_H

#include <arti_graph_processing/types.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <arti_graph_processing/GraphVisualizationConfig.h>
#include <dynamic_reconfigure/server.h>

namespace arti_graph_processing
{

class GraphVisualizationPublisher
{
public:
  GraphVisualizationPublisher(const ros::NodeHandle& node_handle, const std::string& topic);

  GraphVisualizationPublisher(const ros::NodeHandle& node_handle, const std::string& topic,
                              double edge_increase_factor, size_t edge_max_number_increases);

  void reconfigure(const arti_graph_processing::GraphVisualizationConfig& new_config);

  std::unique_ptr<dynamic_reconfigure::Server<arti_graph_processing::GraphVisualizationConfig>> cfg_server_;

  void publish(const Graph& graph, bool interpolated=false);

private:
  ros::NodeHandle node_handle_;
  ros::Publisher publisher_;
  ros::Publisher publisher_interpolated_;
  size_t last_vertex_marker_count_{0};
  double edge_increase_factor_;
  size_t edge_max_number_increases_;
  GraphVisualizationConfig cfg_;
};

}

#endif //ARTI_GRAPH_PROCESSING_GRAPH_VISUALIZATION_PUBLISHER_H
