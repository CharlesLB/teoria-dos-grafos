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
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> fec38ea (✨ finish 2024 homework)
void DFSArticulationPoints(Node* node, int parent, unordered_map<Node*, bool>& visited,
                           unordered_map<Node*, int>& discoveryTime,
                           unordered_map<Node*, int>& low,
                           unordered_map<Node*, Node*>& parentMap,
                           unordered_map<Node*, bool>& articulationPoint,
                           int& time, vector<Node*>& articulationNodes);

<<<<<<< HEAD
<<<<<<< HEAD
vector<Edge*> getBridgeEdgesInGraph(Graph* graph);
void DFSBridgeEdges(Node* node, int parent, unordered_map<Node*, bool>& visited,
                    unordered_map<Node*, int>& discoveryTime,
                    unordered_map<Node*, int>& low,
                    int& time, vector<Edge*>& bridgeEdges);

=======
>>>>>>> ffb1ffa (✨ finish 2024 homework)
=======
>>>>>>> fcdcef4 (🐛 remove useless exports)
=======
>>>>>>> fec38ea (✨ finish 2024 homework)
vector<Edge*> getBridgeEdgesInGraph(Graph* graph);
void DFSBridgeEdges(Node* node, int parent, unordered_map<Node*, bool>& visited,
                    unordered_map<Node*, int>& discoveryTime,
                    unordered_map<Node*, int>& low,
                    int& time, vector<Edge*>& bridgeEdges);

vector<Edge*> getBridgeEdgesInGraph(Graph* graph);
void DFSBridgeEdges(Node* node, int parent, unordered_map<Node*, bool>& visited,
                    unordered_map<Node*, int>& discoveryTime,
                    unordered_map<Node*, int>& low,
                    int& time, vector<Edge*>& bridgeEdges);

Graph* getMinimumPathAndCostByDijkstra(Graph* graph, Node* sourceNode, Node* targetNode);
Graph* getMinimumPathAndCostByFloyd(Graph* graph, Node* sourceNode, Node* targetNode);
Graph* getComplementGraph(Graph* graph);

Graph* createInducedSubgraph(Graph* graph, const vector<Node*>& selectedNodes);

void printStronglyConnectedComponents(Graph* graph);
void fillOrder(Node* node, stack<Node*>& Stack, unordered_map<Node*, bool>& visited);
void DFSUtil(Node* node, unordered_map<Node*, bool>& visited, vector<Node*>& component);
Graph* getTranspose(Graph* graph);

void depthFirstSearch(Node* node, unordered_set<Node*>& visited);
vector<Node*> getDirectTransitiveClosureByNode(Node* node);
vector<Node*> getIndirectTransitiveClosureByNode(Graph* graph, Node* node);

struct GraphMetrics {
    int diameter;
    int radius;
    vector<int> center;
    vector<int> periphery;
};

GraphMetrics getGraphMetricsInGraph(Graph* graph);

Graph* getMinimumSpanningTreeByKruskal(Graph* graph, vector<Node*>& vertexSubset);

Graph* getMinimumSpanningTreeByPrim(Graph* originalGraph, vector<Node*>& nodeVector);

void dfsHelper(Node* node, unordered_set<int>& visited, vector<Edge*>& treeEdges, vector<Edge*>& backEdges);
void printDepthFirstSearchTree(Graph* graph, Node* startNode);

#endif
