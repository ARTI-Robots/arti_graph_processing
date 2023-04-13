/*
Created by clemens on 30.03.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_graph_processing/floyd_warshall_algorithm.h>
#include <arti_graph_processing/vertex.h>
#include <ros/ros.h>

namespace arti_graph_processing
{

void FloydWarshallAlgorithm::computeDistanceValues(Graph& graph)
{
  // algorithm according to pseudo code of https://de.wikipedia.org/wiki/Algorithmus_von_Floyd_und_Warshall
  auto& vertices = graph.getVertices();

  for (auto& k_vertex : vertices)
  {
    for (auto& i_vertex : vertices)
    {
      if (k_vertex == i_vertex)
      {
        continue;
      }

      for (auto& j_vertex : vertices)
      {
        if (k_vertex == j_vertex || i_vertex == j_vertex)
        {
          continue;
        }

        const double to_k_costs = i_vertex->getHeuristicsCosts(k_vertex);

        if (std::isfinite(to_k_costs))
        {
          const double from_k_costs = k_vertex->getHeuristicsCosts(j_vertex);
          if (std::isfinite(from_k_costs))
          {
            const double current_costs = i_vertex->getHeuristicsCosts(j_vertex);
            const double possible_new_costs = to_k_costs + from_k_costs;

            if (possible_new_costs < current_costs)
            {
              i_vertex->setHeuristicsCosts(j_vertex, possible_new_costs);
              i_vertex->setUseEuclideanCosts(false);
            }
          }
        }
      }
    }
  }
}

}
