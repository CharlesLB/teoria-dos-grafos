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

int Main::main(int argc, char *argv[]) {
    if (argc < 2) {
        cout << "Missing arguments.\n";
        cout << "Usage: " << argv[0] << " <input_file> [output_file] [directed] [weightedEdge] [weightedNode]" << endl;
        return 1;
    }

    string inputPath = argv[1];

    if (inputPath.find(".dat") != string::npos) {
        return DATProcess(argc, argv);
    } else if (inputPath.find(".txt") != string::npos) {
        return AMPLProcess(argc, argv);
    } else {
        cout << "Unsupported file type. Only .dat and .txt files are supported.\n";
        return 1;
    }
}

int Main::DATProcess(int argc, char *argv[]) {
    if (argc < 5) {
        cout << "Missing arguments for .dat file.\n";
        cout << "Usage: " << argv[0] << " <output_file> <directed[0,1]> <weightedEdge[0,1]> <weightedNode[0,1]>" << endl;
        return 1;
    }

    string inputPath = argv[1];
    string outputPath = argv[2];
    bool directed = atoi(argv[3]);
    bool weightedEdge = atoi(argv[4]);
    bool weightedNode = atoi(argv[5]);

    Graph *graph = Reader::graph(inputPath, directed, weightedEdge, weightedNode);
    if (!graph) {
        cout << "Error reading graph from file " << inputPath << endl;
        return 1;
    }

    Controller::processOperation(argv, graph);

    return 0;
}

int Main::AMPLProcess(int argc, char *argv[]) {
    if (argc < 2) {
        cout << "Missing arguments for .txt file.\n";
        cout << "Usage: " << argv[0] << " <output_file>" << endl;
        return 1;
    }

    string inputPath = argv[1];
    string outputPath = argv[2];

    Graph *graph = Reader::graphAMPL(inputPath);
    if (!graph) {
        cout << "Error reading graph from file " << inputPath << endl;
        return 1;
    }

    Controller::processOperationAMPL(argv, graph);

    return 0;
}
