#ifndef EDGE_H
#define EDGE_H

#include <iostream>

#include "./graph.hpp"
#include "./node.hpp"

using namespace std;

class Edge {
   public:
    Edge(Node* head, Node* tail, int weight);
    ~Edge();

    int getId();

    Node* getHead();
    Node* getTail();
    int getWeight();

    void setHead(Node* head);
    void setTail(Node* tail);
    void setWeight(int weight);

   private:
    int id;
    Node* head;
    Node* tail;
    int weight;
};

#endif