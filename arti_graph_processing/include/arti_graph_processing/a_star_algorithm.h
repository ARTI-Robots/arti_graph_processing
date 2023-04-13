/*
Created by clemens on 30.03.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_GRAPH_PROCESSING_A_STAR_ALGORITHM_H
#define ARTI_GRAPH_PROCESSING_A_STAR_ALGORITHM_H

#include <arti_graph_processing/types.h>
#include <map>
#include <memory>
#include <vector>

namespace arti_graph_processing
{
class AStarAlgorithm
{
public:
  AStarAlgorithm() = default;

  static std::vector<std::pair<VertexPtr, EdgePtr>> computePath(const VertexPtr& start, const VertexPtr& goal);

private:
  struct AStarSearchStep
  {
    double total_costs;
    double heuristic_costs;
    double costs_from_start;
    VertexPtr vertex;

    bool operator<(const AStarSearchStep& rhs) const
    {
      return std::tie(total_costs, heuristic_costs, costs_from_start, vertex)
             < std::tie(rhs.total_costs, rhs.heuristic_costs, rhs.costs_from_start, rhs.vertex);
    }

    bool operator>(const AStarSearchStep& rhs) const
    {
      return rhs < *this;
    }

    bool operator<=(const AStarSearchStep& rhs) const
    {
      return !(rhs < *this);
    }

    bool operator>=(const AStarSearchStep& rhs) const
    {
      return !(*this < rhs);
    }
  };

  friend bool operator>(const std::shared_ptr<AStarSearchStep>& lhs, const std::shared_ptr<AStarSearchStep>& rhs);

  static std::map<VertexPtr, EdgePtr> computePathInternal(
    const VertexPtr& start, const VertexPtr& goal);

  static void updateOpenList(
    std::vector<std::shared_ptr<AStarSearchStep>>& open_list,
    std::map<VertexPtr, std::shared_ptr<AStarSearchStep>>& total_costs,
    const VertexPtr& new_vertex, double costs_from_start, double heuristic_costs);

  static std::shared_ptr<AStarSearchStep> getMin(
    std::vector<std::shared_ptr<AStarSearchStep>>& open_list,
    std::map<VertexPtr, std::shared_ptr<AStarSearchStep>>& total_costs);
};

bool operator>(
  const std::shared_ptr<AStarAlgorithm::AStarSearchStep>& lhs,
  const std::shared_ptr<AStarAlgorithm::AStarSearchStep>& rhs);
}

#endif //ARTI_GRAPH_PROCESSING_A_STAR_ALGORITHM_H
