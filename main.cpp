#include <dirent.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
#include <sstream>

#include "./lib/graph.cpp"

using namespace std;

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

    cout << "Input path: " << inputPath << endl;
    cout << "Output path: " << outputPath << endl;
    cout << "Directed: " << directed << endl;
    cout << "Weighted edge: " << weightedEdge << endl;
    cout << "Weighted node: " << weightedNode << endl;

    return 0;
}
