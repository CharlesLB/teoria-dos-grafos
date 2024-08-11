#ifndef GRAPH_UTILS_H
#define GRAPH_UTILS_H

#include <iostream>
#include <string>

#include "../../lib/Edge/edge.hpp"
#include "../../lib/Graph/graph.hpp"
#include "../../lib/Node/node.hpp"

Graph* createGraphCopy(Graph* graph);
Graph* convertMultigraphToGraph(Graph* multigraph);

#endif