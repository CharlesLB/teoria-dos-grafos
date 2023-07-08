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

int Reader::readInt() {
    int num;
    std::cout << "Enter an integer: ";
    std::cin >> num;

    while (std::cin.fail()) {
        std::cout << "Invalid input. Please enter an integer: ";
        std::cin.clear();
        std::cin >> num;
    }

    return num;
}

void Reader::continueConfirmation() {
    std::string answer;
    do {
        std::cout << "Continue? (y/n): ";
        std::cin >> answer;
        if (answer != "y" && answer != "n") {
            std::cout << "Invalid input. Please enter 'y' or 'n'." << std::endl;
        }
    } while (answer != "y" && answer != "n");

    if (answer == "y") {
        return;
    }

    cout << "Good Bye!" << endl;
    exit(0);
}