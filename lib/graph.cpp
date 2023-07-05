#include "../includes/graph.hpp"

#include <iostream>

#include "../includes/edge.hpp"
#include "../includes/node.hpp"

Graph::Graph(bool isDirected, bool weightedEdge, bool weightedNodes) {
    this->firstNode = nullptr;
    this->totalNodes = 0;
    this->totalEdges = 0;
    this->directed = isDirected;
    this->weightedEdges = weightedEdge;
    this->weightedNodes = weightedNodes;
}

Graph::~Graph() {
}
