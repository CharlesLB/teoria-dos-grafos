#ifndef WRITER_H
#define WRITER_H

#include <string>
#include <vector>

using namespace std;

#include "../../lib/Graph/graph.hpp"

class Writer {
   public:
    static void printMenu(string* options);
    static void printGraphOptions(Graph* graph, string fileName);
    static void printGraph(Graph* graph);
    static void printGraphInDotFile(Graph* graph, string fileName);
    static void printGraphInTxtFile(Graph* graph, string fileName);

   private:
    static void printNodes(Graph* graph);
    static void printEdges(Graph* graph);
    static vector<string> getGraphDotFile(Graph* graph);
    static vector<string> getDigraphDotFile(Graph* graph);
};

#endif