/*
Created by clemens on 30.03.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_graph_processing/a_star_algorithm.h>
#include <arti_graph_processing/edge.h>
#include <arti_graph_processing/vertex.h>
#include <ros/console.h>
#include <set>
#include <stack>

namespace arti_graph_processing
{
std::vector<std::pair<VertexPtr, EdgePtr>> AStarAlgorithm::computePath(const VertexPtr& start, const VertexPtr& goal)
{
  const std::map<VertexPtr, EdgePtr> reverse_path = computePathInternal(start, goal);

  VertexPtr current_vertex = goal;
  EdgePtr current_edge;

  std::stack<std::pair<VertexPtr, EdgePtr>> path_stack;
  while (current_vertex)
  {
    path_stack.emplace(current_vertex, current_edge);

    if (current_vertex == start)
    {
      break;
    }

    const auto& it = reverse_path.find(current_vertex);
    if (it != reverse_path.end())
    {
      current_edge = it->second;
      current_vertex = current_edge->getSource();
    }
    else
    {
      ROS_ERROR("Reverse map did not find a path");
      current_vertex.reset();
    }
  }

  std::vector<std::pair<VertexPtr, EdgePtr>> result;
  if (!path_stack.empty())
  {
    result.reserve(path_stack.size());
    while (!path_stack.empty())
    {
      result.push_back(path_stack.top());
      path_stack.pop();
    }
  }

  return result;
}

std::map<VertexPtr, EdgePtr> AStarAlgorithm::computePathInternal(
  const VertexPtr& start, const VertexPtr& goal)
{

  std::set<VertexPtr> visited_vertices;
  std::vector<std::shared_ptr<AStarSearchStep>> open_list;
  std::map<VertexPtr, std::shared_ptr<AStarSearchStep>> total_costs;
  updateOpenList(open_list, total_costs, start, 0., start->getHeuristicsCosts(goal));

  std::map<VertexPtr, EdgePtr> result;
  while (!open_list.empty())
  {
    std::shared_ptr<AStarSearchStep> current_vertex = getMin(open_list, total_costs);

    if (current_vertex->vertex == goal)
    {
      break;
    }

    visited_vertices.insert(current_vertex->vertex);

    std::vector<EdgePtr> edges = current_vertex->vertex->getOutgoingEdges();

    for (const auto& edge: edges)
    {
      // check if vertex is already in visited list and continue
      if (visited_vertices.count(edge->getDestination()) > 0)
      {
        continue;
      }

      double new_costs_from_start = current_vertex->costs_from_start + edge->getCosts();

      const auto& it = total_costs.find(edge->getDestination());

      if ((it != total_costs.end()) && (it->second->costs_from_start <= new_costs_from_start))
      {
        continue;
      }

      result.insert(std::make_pair(edge->getDestination(), edge));

      updateOpenList(open_list, total_costs, edge->getDestination(), new_costs_from_start,
                     edge->getDestination()->getHeuristicsCosts(goal));
    }
  }

  return result;
}

void AStarAlgorithm::updateOpenList(
  std::vector<std::shared_ptr<AStarSearchStep>>& open_list,
  std::map<VertexPtr, std::shared_ptr<AStarSearchStep>>& total_costs,
  const VertexPtr& new_vertex, double costs_from_start, double heuristic_costs)
{
  const auto it = total_costs.find(new_vertex);

  if (it != total_costs.end())
  {
    it->second->total_costs = costs_from_start + heuristic_costs;

    std::make_heap(open_list.begin(), open_list.end(), std::greater<std::shared_ptr<AStarSearchStep>>());
  }
  else
  {
    std::shared_ptr<AStarSearchStep> new_entry(new AStarSearchStep);
    new_entry->costs_from_start = costs_from_start;
    new_entry->heuristic_costs = heuristic_costs;
    new_entry->total_costs = costs_from_start + heuristic_costs;
    new_entry->vertex = new_vertex;

    open_list.push_back(new_entry);
    total_costs.insert(std::make_pair(new_vertex, new_entry));

    std::push_heap(open_list.begin(), open_list.end(), std::greater<std::shared_ptr<AStarSearchStep>>());
  }
}

std::shared_ptr<AStarAlgorithm::AStarSearchStep> AStarAlgorithm::getMin(
  std::vector<std::shared_ptr<AStarSearchStep>>& open_list,
  std::map<VertexPtr, std::shared_ptr<AStarSearchStep>>& total_costs)
{
  // first sort the heap
  std::sort_heap(open_list.begin(), open_list.end(), std::greater<std::shared_ptr<AStarSearchStep>>());
  // get first result as it is the smallest
  std::shared_ptr<AStarSearchStep> result = open_list.front();
  // then pop heap to return the smallest to the back
  std::pop_heap(open_list.begin(), open_list.end(), std::greater<std::shared_ptr<AStarSearchStep>>());
  // then remove the smallest from the list...
  open_list.pop_back();

  total_costs.erase(result->vertex);

  return result;
}

bool operator>(
  const std::shared_ptr<AStarAlgorithm::AStarSearchStep>& lhs,
  const std::shared_ptr<AStarAlgorithm::AStarSearchStep>& rhs)
{
  if (!lhs || !rhs)
  {
    return lhs < rhs;
  }

  return (*lhs) < (*rhs);
}
}
