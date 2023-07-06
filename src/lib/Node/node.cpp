#include "../Node/node.hpp"

#include <iostream>

#include "../Edge/edge.hpp"

Node::Node(int id, int weight)
    : id(id), weight(weight), degreeIn(0), degreeOut(0), nextNode(nullptr), firstEdge(nullptr) {}

Node::~Node() {
}

int Node::getId() {
    return id;
}

int Node::getDegreeIn() {
    return degreeIn;
}

int Node::getDegreeOut() {
    return degreeOut;
}

int Node::getWeight() {
    return weight;
}

void Node::setWeight(int weight) {
    this->weight = weight;
}

Edge* Node::getEdge(int id) {
    for (Edge* edge = firstEdge; edge != nullptr; edge = edge->getNextEdge()) {
        if (edge->getId() == id) {
            return edge;
        }
    }
    return nullptr;
}

vector<Edge*> Node::getEdges() {
    vector<Edge*> edges;
    for (Edge* edge = firstEdge; edge != nullptr; edge = edge->getNextEdge()) {
        edges.push_back(edge);
    }
    return edges;
}

Node* Node::getNextNode() {
    return nextNode;
}

void Node::setNextNode(Node* node) {
    nextNode = node;
}

Edge* Node::getFirstEdge() {
    return firstEdge;
}

void Node::setFirstEdge(Edge* firstEdge) {
    this->firstEdge = firstEdge;
}

void Node::addEdge(Edge* edge) {
    if (edge == nullptr) {
        return;
    }

    if (edge->getHead() == this) {
        incrementDegreeOut();
    }

    if (edge->getTail() == this) {
        incrementDegreeIn();
    }

    edge->setNextEdge(firstEdge);
    firstEdge = edge;
}

void Node::removeEdge(Edge* edge) {
    if (edge == nullptr) {
        return;
    }

    if (edge->getTail() == this) {
        decrementDegreeIn();
    }

    if (edge->getHead() == this) {
        decrementDegreeOut();
    }

    Edge* prevEdge = nullptr;
    Edge* currEdge = firstEdge;
    while (currEdge != nullptr) {
        if (currEdge == edge) {
            if (prevEdge != nullptr) {
                prevEdge->setNextEdge(currEdge->getNextEdge());
            } else {
                firstEdge = currEdge->getNextEdge();
            }
            break;
        }
        prevEdge = currEdge;
        currEdge = currEdge->getNextEdge();
    }
}

void Node::decrementDegreeIn() {
    degreeIn--;
}

void Node::decrementDegreeOut() {
    degreeOut--;
}

void Node::incrementDegreeIn() {
    degreeIn++;
}

void Node::incrementDegreeOut() {
    degreeOut++;
}