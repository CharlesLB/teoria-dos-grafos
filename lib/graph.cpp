#include "../includes/graph.hpp"

#include <iostream>

#include "../includes/edge.hpp"
#include "../includes/node.hpp"

Graph::Graph() {
    this->firstNode = NULL;
    this->edges = std::vector<Edge*>();
}

Graph::~Graph() {
}
