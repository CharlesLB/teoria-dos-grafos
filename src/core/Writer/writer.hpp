#ifndef WRITER_H
#define WRITER_H

#include <string>
#include <vector>

using namespace std;

#include "../../lib/Graph/graph.hpp"

class Writer {
   public:
    static void printMenu(string* options);
    static void printGraph(Graph* graph);

   private:
    static void printNodes(Graph* graph);
    static void printEdges(Graph* graph);
};

#endif