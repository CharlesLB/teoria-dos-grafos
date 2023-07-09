#ifndef TESTS_H
#define TESTS_H

#include <iostream>
#include <string>

#include "../../lib/Edge/edge.hpp"
#include "../../lib/Graph/graph.hpp"
#include "../../lib/Node/node.hpp"

Graph* createGraphCopy(Graph* graph);
Graph* convertMultigraphToGraph(Graph* multigraph);

#endif