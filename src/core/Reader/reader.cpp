#include "./reader.hpp"

#include <time.h>

#include <cstring>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "../../lib/Graph/graph.hpp"

vector<string> Reader::readLines(string filename) {
    FILE *file = openFile(filename);
    vector<string> lines;

    char buffer[256];
    while (fgets(buffer, sizeof(buffer), file)) {
        lines.emplace_back(buffer);
    }

    fclose(file);

    return lines;
}

Graph *Reader::graph(string filename, bool directed, bool weightedEdges, bool weightedNodes) {
    FILE *file = openFile(filename);
    vector<string> lines;

    Graph *graph = new Graph(directed, weightedEdges, weightedNodes);

    lines = readLines(filename);
    int totalNodes = stoi(lines[0]);

    for (int i = 1; i <= totalNodes; i++) {
        graph->createOrUpdateNode(i, 1);
    }

    lines.erase(lines.begin());

    if (weightedEdges) {
        for (const string &line : lines) {
            istringstream iss(line);
            int source, destination, weight;
            iss >> source >> destination >> weight;

            Node *sourceNode = graph->findNodeById(source);
            Node *destinationNode = graph->findNodeById(destination);

            graph->createEdge(sourceNode, destinationNode, weight);
        }

        return graph;
    }

    for (const string &line : lines) {
        istringstream iss(line);
        int source, destination;
        iss >> source >> destination;

        Node *sourceNode = graph->findNodeById(source);
        Node *destinationNode = graph->findNodeById(destination);

        graph->createEdge(sourceNode, destinationNode, 1);
    }

    return graph;
}

FILE *Reader::openFile(string filename) {
    FILE *file = fopen(filename.c_str(), "r");

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

Graph *Reader::graphAMPL(string filename) {
    GraphAMPL graphAMPL = getDataFromAMPL(filename);

    Graph *graph = new Graph(false, true, true);

    for (int i = 1; i <= graphAMPL.numClusters; i++) {
        graph->createOrUpdateNode(i, 1);
    }

    for (int i = 0; i < graphAMPL.nodeWeights.size(); i++) {
        graph->createOrUpdateNode(graphAMPL.nodeWeights[i].first, graphAMPL.nodeWeights[i].second);
    }

    for (int i = 0; i < graphAMPL.edges.size(); i++) {
        Node *sourceNode = graph->findNodeById(graphAMPL.edges[i].first);
        Node *destinationNode = graph->findNodeById(graphAMPL.edges[i].second);

        if (sourceNode == nullptr || destinationNode == nullptr) {
            continue;
        }

        graph->createEdge(sourceNode, destinationNode, 1);
    }

    return graph;
}

GraphAMPL Reader::getDataFromAMPL(string filename) {
    ifstream file(filename);
    GraphAMPL graphAMPL;

    string line;

    while (getline(file, line)) {
        if (line.find("param p :=") != string::npos) {
            stringstream ss(line);
            string temp;
            ss >> temp >> temp >> temp >> graphAMPL.numClusters;
        } else if (line.find("set V :=") != string::npos) {
            getline(file, line);
            stringstream ss(line);
            int vertex;
            while (ss >> vertex) {
                graphAMPL.nodes.insert(vertex);
            }
        } else if (line.find("param w :=") != string::npos) {
            while (getline(file, line) && !line.empty() && line.find(";") == string::npos) {
                stringstream ss(line);
                int vertex, weight;
                ss >> vertex >> weight;
                graphAMPL.nodeWeights.push_back({vertex, weight});
            }
        } else if (line.find("set E :=") != string::npos) {
            while (getline(file, line) && !line.empty() && line.find("end;") == string::npos) {
                stringstream ss(line);
                char ignore;
                int v1, v2;
                while (ss >> ignore >> v1 >> ignore >> v2 >> ignore) {
                    graphAMPL.edges.push_back({v1, v2});
                }
            }
        } else if (line.find("set Y0 :=") != string::npos) {
            while (getline(file, line) && line.find(");") == string::npos) {
                // Do nothing
            }
        }
    }

    file.close();
    return graphAMPL;
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