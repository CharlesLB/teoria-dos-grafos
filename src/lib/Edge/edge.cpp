#include "./edge.hpp"

#include <iostream>

#include "../Node/node.hpp"

int edgeIndex = 0;

Edge::Edge(Node* head, Node* tail, int weight)
    : id(edgeIndex++),
      head(head),
      tail(tail),
      weight(weight),
      nextEdge(nullptr) {}

Edge::~Edge() {
}

int Edge::getId() {
    return id;
}

int Edge::getWeight() {
    return weight;
}

void Edge::setWeight(int weight) {
    this->weight = weight;
}

Node* Edge::getHead() {
    return head;
}

Node* Edge::getTail() {
    return tail;
}

Edge* Edge::getNextEdge() {
    return nextEdge;
}

void Edge::setNextEdge(Edge* edge) {
    nextEdge = edge;
}
