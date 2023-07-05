#ifndef WRITER_H
#define WRITER_H

#include <string>
#include <vector>

using namespace std;

#include "../../lib/Graph/graph.hpp"

class Writer {
   public:
    static void printMenu();
    static void printGraph(Graph* graph);
    static void printGraphInFile(Graph* graph, string filename);

   private:
};

#endif