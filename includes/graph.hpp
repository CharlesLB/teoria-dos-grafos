#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <vector>

#include "./edge.hpp"
#include "./graph.hpp"

using namespace std;

class Graph {
   public:
    Graph();
    ~Graph();

    int getNumNodes();
    int getNumEdges();

    vector<Node*> getNodes();
    vector<Edge*> getEdges();

    void addNode(Node* node);
    void addEdge(Edge* edge);

    void removeNode(Node* node);
    void removeNode(int id);
    void removeEdge(Edge* edge);
    void removeEdge(int id);

    Node* getNode(int id);
    Edge* getEdge(int id);

    void setNodes(vector<Node*> nodes);
    void setEdges(vector<Edge*> edges);

   private:
    vector<Node*> nodes;
    vector<Edge*> edges;
};

#endif