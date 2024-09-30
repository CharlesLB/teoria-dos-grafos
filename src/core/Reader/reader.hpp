#ifndef READER_H
#define READER_H

#include <set>
#include <string>
#include <vector>

#include "../../lib/Graph/graph.hpp"

using namespace std;

struct GraphAMPL {
    int numClusters;
    set<int> nodes;
    vector<pair<int, int>> edges;
    vector<pair<int, int>> nodeWeights;
};

class Graph;
class Reader {
   public:
    static int integer();

    static Graph *graph(string filename, bool directed, bool weightedEdges, bool weightedNodes);
    static char readChar();
    static int readInt();
    static float readFloat(float minVal, float maxVal);
    static pair<Graph *, int> graphAMPL(string filename);

    static void continueConfirmation();

   private:
    static GraphAMPL getDataFromAMPL(string filename);
    static FILE *openFile(string filename);
    static vector<string> readLines(string filename);
};

#endif