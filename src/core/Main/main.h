#ifndef MAIN_H
#define MAIN_H

#include <string>
#include <vector>

#include "../../lib/Graph/graph.hpp"

using namespace std;

class Graph;
class Main {
   public:
    static int main(int argc, char* argv[]);

   private:
    static void processOperation(char* argv[], bool hasWeightedNode, bool hasWeightedEdge, bool isDirected, Graph* graph);
};

#endif