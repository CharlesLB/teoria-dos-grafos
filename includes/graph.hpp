#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <vector>

using namespace std;

class Edge;
class Node;
class Graph {
   public:
    Graph(bool directed, bool weightedEdge, bool weightedNodes);
    ~Graph();

    Node* getFirstNode();
    int getNumNodes();
    int getNumEdges();
    bool isWeightedEdges();
    bool isWeightedNodes();
    bool isDirected();

    Node* createOrUpdateNode(int id, int weight);
    Edge* createEdge(Node* head, Node* tail, int weight);
    void deleteEdge(Edge* edge);
    void deleteNode(Node* node);

   private:
    Node* firstNode;
    vector<Edge*> edges;

    int totalNodes;
    int totalEdges;
    bool weightedEdges;
    bool weightedNodes;
    bool directed;
};

#endif