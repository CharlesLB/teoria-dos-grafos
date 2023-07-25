#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include <iostream>
#include <string>
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
Graph* getMinimumPathAndCostByBellmanFord(Graph* graph, Node* sourceNode, Node* targetNode);

#endif
