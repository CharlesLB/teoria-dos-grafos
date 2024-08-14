#include "./reader.hpp"

#include <time.h>

#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "../../lib/Graph/graph.hpp"

vector<string> Reader::readLines(string filename) {
    FILE* file = openFile(filename);
    vector<string> lines;

    char buffer[256];
    while (fgets(buffer, sizeof(buffer), file)) {
        lines.emplace_back(buffer);
    }

    fclose(file);

    return lines;
}

Graph* Reader::graph(string filename, bool directed, bool weightedEdges, bool weightedNodes) {
    FILE* file = openFile(filename);
    vector<string> lines;

    Graph* graph = new Graph(directed, weightedEdges, weightedNodes);

    lines = readLines(filename);
    int totalNodes = stoi(lines[0]);

    for (int i = 1; i <= totalNodes; i++) {
        graph->createOrUpdateNode(i, 1);
    }

    lines.erase(lines.begin());

    if (weightedEdges) {
        for (const string& line : lines) {
            istringstream iss(line);
            int source, destination, weight;
            iss >> source >> destination >> weight;

            Node* sourceNode = graph->findNodeById(source);
            Node* destinationNode = graph->findNodeById(destination);

            graph->createEdge(sourceNode, destinationNode, weight);
        }

        return graph;
    }

    for (const string& line : lines) {
        istringstream iss(line);
        int source, destination;
        iss >> source >> destination;

        Node* sourceNode = graph->findNodeById(source);
        Node* destinationNode = graph->findNodeById(destination);

        graph->createEdge(sourceNode, destinationNode, 1);
    }

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
    cout << "Enter a character: ";
    cin >> ch;

    while (cin.fail()) {
        cout << "Invalid input. Please enter a character: ";
        cin.clear();
        cin >> ch;
    }

    return ch;
}

int Reader::readInt() {
    int num;
    cout << "Enter an integer: ";
    cin >> num;

    while (cin.fail()) {
        cout << "Invalid input. Please enter an integer: ";
        cin.clear();
        cin >> num;
    }

    return num;
}

void Reader::continueConfirmation() {
    string answer;
    do {
        cout << "Continue? (y/n): ";
        cin >> answer;
        if (answer != "y" && answer != "n") {
            cout << "Invalid input. Please enter 'y' or 'n'." << endl;
        }
    } while (answer != "y" && answer != "n");

    if (answer == "y") {
        return;
    }

    cout << "Good Bye!" << endl;
    exit(0);
}