#include "./reader.hpp"

#include <time.h>

#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "../../lib/Graph/graph.hpp"

Graph* Reader::graph(string filename, bool directed, bool weightedEdges, bool weightedNodes) {
    FILE* file = openFile(filename);

    Graph* graph = new Graph(directed, weightedEdges, weightedNodes);

    return graph;
}

FILE* Reader::openFile(string filename) {
    FILE* file = fopen(filename.c_str(), "r");

    if (file == NULL) {
        cout << "Error opening file " << filename << endl;
        exit(1);
    }

    return file;
}

char Reader::readChar() {
    char ch;
    std::cout << "Enter a character: ";
    std::cin >> ch;

    while (std::cin.fail()) {
        std::cout << "Invalid input. Please enter a character: ";
        std::cin.clear();
        std::cin >> ch;
    }

    return ch;
}
