#ifndef NODE_H
#define NODE_H

#include <iostream>
#include <vector>

#include "../Edge/edge.hpp"
#include "../Graph/graph.hpp"

using namespace std;

class Edge;
class Node {
   public:
    Node(int id, int weight);
    ~Node();

    int getId();
    int getDegreeIn();
    int getDegreeOut();

    int getWeight();
    void setWeight(int weight);

    Edge* getEdge(int id);
    vector<Edge*> getEdges();

    Node* getNextNode();
    void setNextNode(Node* node);

    Edge* getFirstEdge();
    void setFirstEdge(Edge* firstEdge);

    void addEdge(Edge* edge);
    void removeEdge(Edge* edge);

    int getDegree(bool directed);

    void decrementDegreeIn();
    void decrementDegreeOut();
    void incrementDegreeIn();
    void incrementDegreeOut();

    vector<Node*> getNeighbors();

   private:
    int id;
    int degreeIn;
    int degreeOut;
    int weight;

    int color;

    Node* nextNode;
    Edge* firstEdge;
};

#endif