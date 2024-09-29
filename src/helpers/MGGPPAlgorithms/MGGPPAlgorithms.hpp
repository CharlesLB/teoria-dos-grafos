#ifndef MGGPP_ALGORITHMS_H
#define MGGPP_ALGORITHMS_H

#include <algorithm>
#include <iostream>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "../../lib/Edge/edge.hpp"
#include "../../lib/Graph/graph.hpp"
#include "../../lib/Node/node.hpp"

Graph* getMGGPPByGreedyAlgorithm(Graph* graph);

Graph* getMGGPPByGRASPAlgorithm(Graph* graph);

Graph* getMGGPPByReactiveGRASPAlgorithm(Graph* graph);

#endif
