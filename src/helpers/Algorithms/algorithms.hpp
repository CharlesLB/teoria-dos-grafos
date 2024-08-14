#ifndef ALGORITHMS_H
#define ALGORITHMS_H

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

vector<Node*> getOpenNeighborhoodNodesByNode(Node* node);
vector<Node*> getClosedNeighborhoodNodesByNode(Node* node);
vector<Node*> getArticulationNodesInGraph(Graph* graph);
vector<Edge*> getBridgeEdgesInGraph(Graph* graph);
Graph* getMinimumPathAndCostByDijkstra(Graph* graph, Node* sourceNode, Node* targetNode);
Graph* getMinimumPathAndCostByFloyd(Graph* graph, Node* sourceNode, Node* targetNode);
Graph* getComplementGraph(Graph* graph);
Graph* createInducedSubgraph(Graph* graph, const vector<Node*>& selectedNodes);
void printStronglyConnectedComponents(Graph* graph);
vector<Node*> getDirectTransitiveClosureByNode(Node* node);
vector<Node*> getIndirectTransitiveClosureByNode(Graph* graph, Node* node);
Graph* getMinimumSpanningTreeByKruskal(Graph* graph, vector<Node*>& vertexSubset);
Graph* getMinimumSpanningTreeByPrim(Graph* originalGraph, vector<Node*>& nodeVector);
void printDepthFirstSearchTree(Graph* graph, int startId);

struct GraphMetrics {
    int diameter;
    int radius;
    vector<int> center;
    vector<int> periphery;
};

GraphMetrics getGraphMetricsInGraph(Graph* graph);

#endif
