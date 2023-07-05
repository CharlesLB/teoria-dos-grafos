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

    int getNumNodes();
    int getNumEdges();

    bool isWeightedEdges();
    bool isWeightedNodes();
    bool isDirected();

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