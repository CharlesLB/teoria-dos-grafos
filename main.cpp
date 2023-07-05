#include <dirent.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
#include <sstream>

#include "./lib/graph.cpp"
#include "./lib/reader.cpp"
#include "./lib/writer.cpp"

using namespace std;

void processOperation(char* argv[], bool hasWeightedNode, bool hasWeightedEdge, bool isDirected, Graph* graph) {
    int option;

    while (true) {
        Writer::printMenu();
        cin >> option;

        switch (option) {
            case 0:
                cout << "Exiting...\n";
                exit(0);
                break;
            case 'a':
                cout << "Option a\n";
                break;

            default:
                cout << "Invalid option\n";
                break;
        }
    }
}

int main(int argc, char* argv[]) {
    string path, inputPath, outputPath;
    bool directed, weightedEdge, weightedNode;

    if (argc <= 5) {
        cout << "Missing arguments.\n";
        cout << "Usage: " << argv[0] << " <input_file>"
             << " <output_file>"
             << " <directed[0,1]> <weightedEdge[0,1]> <weightedNode[0,1]>";
        return 1;
    }

    inputPath = argv[1];
    outputPath = argv[2];
    directed = atoi(argv[3]);
    weightedEdge = atoi(argv[4]);
    weightedNode = atoi(argv[5]);

    Graph* graph = Reader::graph(inputPath, directed, weightedEdge, weightedNode);
    processOperation(argv, weightedNode, weightedEdge, directed, graph);

    return 0;
}
