#include "../includes/node.hpp"

#include <iostream>

Node::Node(int id) {
    this->id = id;
    this->degreeIn = 0;
    this->degreeOut = 0;
}

Node::~Node() {
    this->id = 0;
    this->degreeIn = 0;
    this->degreeOut = 0;
    this->edges.clear();
}

int Node::getId() {
    return this->id;
}

void Node::setId(int id) {
    this->id = id;
}

int Node::getDegreeIn() {
    return this->degreeIn;
}

int Node::getDegreeOut() {
    return this->degreeOut;
}

vector<Edge*> Node::getEdges() {
    return this->edges;
}

void Node::addEdge(Edge* edge) {
    this->edges.push_back(edge);
    if (edge->getHead() == this) {
        this->degreeOut++;
    } else if (edge->getTail() == this) {
        this->degreeIn++;
    }
}

void Node::removeEdge(Edge* edge) {
    for (int i = 0; i < this->edges.size(); i++) {
        if (this->edges[i] == edge) {
            this->edges.erase(this->edges.begin() + i);
            if (edge->getHead() == this) {
                this->degreeOut--;
            } else if (edge->getTail() == this) {
                this->degreeIn--;
            }
            break;
        }
    }
}

Edge* Node::getEdge(int id) {
    for (int i = 0; i < this->edges.size(); i++) {
        if (this->edges[i]->getId() == id) {
            return this->edges[i];
        }
    }
    return nullptr;
}

void Node::setEdges(vector<Edge*> edges) {
    this->edges = edges;
}