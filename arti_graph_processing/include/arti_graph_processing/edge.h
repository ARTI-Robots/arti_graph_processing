/*
Created by clemens on 30.03.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_GRAPH_PROCESSING_EDGE_H
#define ARTI_GRAPH_PROCESSING_EDGE_H

#include <arti_graph_processing/types.h>
#include <memory>

namespace arti_graph_processing
{

class Edge : public std::enable_shared_from_this<Edge>
{
public:
  Edge(const VertexPtr& source, const VertexPtr& destination);
  Edge(const VertexPtr& source, const VertexPtr& destination, double costs);
  Edge(Edge&&) = default;
  virtual ~Edge() = default;

  VertexPtr getSource() const;
  VertexPtr getDestination() const;

  double getCosts() const;

private:
  friend class Graph;

  VertexWPtr source_;
  VertexWPtr destination_;

protected:
  double costs_;
};

}

#endif //ARTI_GRAPH_PROCESSING_EDGE_H
