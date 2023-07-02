#ifndef NODE_H
#define NODE_H

#include <iostream>
#include <vector>

#include "./edge.hpp"
#include "./graph.hpp"

using namespace std;

class Edge;
class Node {
   public:
    Node(int id);
    ~Node();

    int getId();
    void setId(int id);

    int getDegreeIn();
    int getDegreeOut();

    Edge* getEdge(int id);
    vector<Edge*> getEdges();

    void addEdge(Edge* edge);

    void removeEdge(Edge* edge);

    void setEdges(vector<Edge*> edges);

   private:
    int id;
    int degreeIn;
    int degreeOut;
    vector<Edge*> edges;

    void decrementDegreeIn();
    void decrementDegreeOut();
    void incrementDegreeIn();
    void incrementDegreeOut();
};

#endif