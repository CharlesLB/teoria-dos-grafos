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

vector<Graph*> getMGGPPByGreedyAlgorithm(Graph* graph, int numClusters);
Node* findConnectedNode(Graph* graph, Node* node, const vector<Node*>& nodeList);
Node* findNodeConnectedTo(Graph* graph, Node* newNode, Graph* cluster);

Graph* getMGGPPByGRASPAlgorithm(Graph* graph, int numClusters);

Graph* getMGGPPByReactiveGRASPAlgorithm(Graph* graph, int numClusters);

Graph* mergeGraphs(vector<Graph*> graphs);

#endif
