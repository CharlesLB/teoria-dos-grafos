#include "./edge.hpp"

#include <iostream>

#include "../Node/node.hpp"

Edge::Edge(Node* head, Node* tail, int weight)
    : head(head), tail(tail), weight(weight), nextEdge(nullptr) {}

Edge::~Edge() {
    if (nextEdge != nullptr) {
        delete nextEdge;
    }
}

int Edge::getId() {
    return id;
}

int Edge::getWeight() {
    return weight;
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
