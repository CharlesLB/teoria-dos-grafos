#ifndef MANAGER_H
#define MANAGER_H

#include <string>
#include <vector>

#include "../../lib/Edge/edge.hpp"
#include "../../lib/Graph/graph.hpp"
#include "../../lib/Node/node.hpp"

using namespace std;

class Graph;
class Node;
class Edge;
class Manager {
   public:
    static void processOperation(Graph* graph);
    static void getNodeDegree(Graph* graph);

   private:
    static void createOrUpdateNode(Graph* graph);
    static void createOrUpdateEdge(Graph* graph);
    static void removeNode(Graph* graph);
    static void removeEdge(Graph* graph);
};

#endif