#ifndef NODE_H
#define NODE_H

#include <iostream>
#include <vector>

#include "./edge.hpp"
#include "./graph.hpp"

using namespace std;

class Node {
   public:
    Node(int id);
    ~Node();

    int getId();
    void setId(int id);

    int getDegreeIn();
    int getDegreeOut();

    vector<Edge*> getEdges();

    void addEdge(Edge* edge);

    void removeEdge(Edge* edge);

    Edge* getEdge(int id);

    void setEdges(vector<Edge*> edges);

   private:
    int id;
    int degreeIn;
    int degreeOut;
    vector<Edge*> edges;
};

#endif