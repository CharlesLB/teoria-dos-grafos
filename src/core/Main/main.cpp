#include "./main.hpp"

#include <dirent.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
#include <sstream>

#include "../../lib/Edge/edge.hpp"
#include "../../lib/Graph/graph.hpp"
#include "../../lib/Node/node.hpp"
#include "../Controller/controller.hpp"
#include "../Reader/reader.hpp"
#include "../Writer/writer.hpp"

using namespace std;

int Main::main(int argc, char* argv[]) {
    string path, inputPath, outputPath;
    bool directed, weightedEdge, weightedNode;

    if (argc <= 5) {
        cout << "Missing arguments.\n";
        cout << "Usage: " << argv[0] << " <input_file>"
             << " <output_file>"
             << " <directed[0,1]> <weightedEdge[0,1]> <weightedNode[0,1]>"
             << endl;
        return 1;
    }

    inputPath = argv[1];
    outputPath = argv[2];
    directed = atoi(argv[3]);
    weightedEdge = atoi(argv[4]);
    weightedNode = atoi(argv[5]);

    Graph* graph = Reader::graph(inputPath, directed, weightedEdge, weightedNode);
    Controller::processOperation(argv, weightedNode, weightedEdge, directed, graph);

    return 0;
}
