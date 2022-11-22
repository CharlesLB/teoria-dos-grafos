#include "../includes/graph.hpp"

#include <iostream>

using namespace std;

Graph::Graph() {
    this->nodes = vector<Node*>();
    this->edges = vector<Edge*>();
}

Graph::~Graph() {
    this->nodes.clear();
    this->edges.clear();
}

int Graph::getNumNodes() {
    return this->nodes.size();
}

int Graph::getNumEdges() {
    return this->edges.size();
}

vector<Node*> Graph::getNodes() {
    return this->nodes;
}

vector<Edge*> Graph::getEdges() {
    return this->edges;
}

void Graph::addNode(Node* node) {
    this->nodes.push_back(node);
}

void Graph::addEdge(Edge* edge) {
    this->edges.push_back(edge);
}

void Graph::removeNode(int id) {
    for (int i = 0; i < this->nodes.size(); i++) {
        if (this->nodes[i]->getId() == id) {
            this->nodes.erase(this->nodes.begin() + i);
            break;
        }
    }
}

void Graph::removeEdge(int id) {
    for (int i = 0; i < this->edges.size(); i++) {
        if (this->edges[i]->getId() == id) {
            this->edges.erase(this->edges.begin() + i);
            break;
        }
    }
}

Node* Graph::getNode(int id) {
    for (int i = 0; i < this->nodes.size(); i++) {
        if (this->nodes[i]->getId() == id) {
            return this->nodes[i];
        }
    }
    return NULL;
}

Edge* Graph::getEdge(int id) {
    for (int i = 0; i < this->edges.size(); i++) {
        if (this->edges[i]->getId() == id) {
            return this->edges[i];
        }
    }
    return NULL;
}

void Graph::setNodes(vector<Node*> nodes) {
    this->nodes = nodes;
}

void Graph::setEdges(vector<Edge*> edges) {
    this->edges = edges;
}
