#ifndef EDGE_H
#define EDGE_H

#include <iostream>

#include "../Graph/graph.hpp"
#include "../Node/node.hpp"

class Node;
class Edge {
   public:
    Edge(Node* head, Node* tail, int weight);
    ~Edge();

    int getId();
    int getWeight();
    Node* getHead();
    Node* getTail();

    Edge* getNextEdge();
    void setNextEdge(Edge* edge);

   private:
    int id;
    int weight;

    Node* head;
    Node* tail;
    Edge* nextEdge;
};

#endif