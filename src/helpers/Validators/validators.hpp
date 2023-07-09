#ifndef TESTS_H
#define TESTS_H

#include <iostream>
#include <string>

#include "../../lib/Edge/edge.hpp"
#include "../../lib/Graph/graph.hpp"
#include "../../lib/Node/node.hpp"

bool checkGraphIsKRegularByK(Graph* graph, int k);

bool checkGraphIsTrivial(Graph* graph);

bool checkGraphIsNull(Graph* graph);

bool checkGraphIsComplete(Graph* graph);

bool checkGraphIsBipartite(Graph* graph);

bool checkGraphIsEulerian(Graph* graph);

#endif