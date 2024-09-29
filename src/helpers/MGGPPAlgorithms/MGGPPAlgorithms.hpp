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

Graph* getMGGPPByGRASPAlgorithm(Graph* graph, int numClusters);

Graph* getMGGPPByReactiveGRASPAlgorithm(Graph* graph, int numClusters);

// Helper functions
unordered_map<int, int> getNodeDegreeMap(Graph* graph);
pair<int, int> getClusterWeightLimitAndGraphWeightAverage(Graph* graph, int numClusters);
Node* getNextNode(Node* currentNode, int graphWeightAverage);
Node* checkIfNodeHasDegreeOne(Node* currentNode);
void deleteNodeFromGraph(Graph* graph, Node* node, unordered_map<int, int>& nodesDegree);
void addNodeToCluster(Graph* cluster, Node* node, Node* parentNode, int& clusterWeight);
int getClusterWeight(Graph* graph, int numClusters);
int getClusterSizeLimit(Graph* graph, int numClusters);
int getSmallerDegreeNode(std::unordered_map<int, int>& nodesDegree);

Graph* mergeGraphs(vector<Graph*> graphs);

#endif
