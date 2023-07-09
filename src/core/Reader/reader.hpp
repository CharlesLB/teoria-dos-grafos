#ifndef READER_H
#define READER_H

#include <string>
#include <vector>

#include "../../lib/Graph/graph.hpp"

using namespace std;

class Graph;
class Reader {
   public:
    static int integer();

    static Graph* graph(string filename, bool directed, bool weightedEdges, bool weightedNodes);
    static char readChar();
    static int readInt();

    static void continueConfirmation();

   private:
    static FILE* openFile(string filename);
    static vector<string> readLines(string filename);
};

#endif