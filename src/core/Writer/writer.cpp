#include "./writer.hpp"

#include <sys/stat.h>
#include <time.h>

#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "../../lib/Edge/edge.hpp"
#include "../../lib/Graph/graph.hpp"
#include "../../lib/Node/node.hpp"
#include "../../utils/filesystem/filesystem.hpp"
#include "../Reader/reader.hpp"

using namespace std;

string output_directory = "./output/";

void Writer::printMenu(string* options) {
    cout << "\nChoose an option:\n"
         << endl;

    char index = 'a';
    for (const string* option = options; *option != ""; option++) {
        cout << "(" << index << ") " << *option << endl;

        if (index == 'z') {
            index = '0';
        } else {
            index++;
        }
    }
}

void Writer::printNodes(Graph* graph) {
    Node* currentNode = graph->getFirstNode();
    while (currentNode != nullptr) {
        cout << "Node ID: " << currentNode->getId() << endl;
        cout << "Weight: " << currentNode->getWeight() << endl;
        cout << "Degree In: " << currentNode->getDegreeIn() << endl;
        cout << "Degree Out: " << currentNode->getDegreeOut() << endl;
        cout << endl;

        currentNode = currentNode->getNextNode();
    }
}

void Writer::printEdges(Graph* graph) {
    vector<Edge*> edges = graph->getEdges();

    for (Edge* edge : edges) {
        cout << "Weight: " << edge->getWeight() << endl;
        cout << "Head Node ID: " << edge->getHead()->getId() << endl;
        cout << "Tail Node ID: " << edge->getTail()->getId() << endl;
        cout << endl;
    }
}

void Writer::printGraphOptions(Graph* graph, string fileName) {
    cout << "Choose an option:\n"
         << endl;

    string options[] = {"Print Graph", "Print Graph in Dot File", "Print Graph in Txt File", "Exit", ""};

    printMenu(options);

    char option = Reader::readChar();

    switch (option) {
        case 'a':
            printGraph(graph);
            break;

        case 'b':
            printGraphInDotFile(graph, fileName);
            break;

        case 'c':
            printGraphInTxtFile(graph, fileName);
            break;

        case 'd':
            return;

        default:
            cout << "Invalid option\n";
            break;
    }
}

void Writer::printGraph(Graph* graph) {
    cout << "Directed: " << graph->isDirected() << endl;
    cout << "Weighted Nodes: " << graph->isWeightedNodes() << endl;
    cout << "Weighted Edges: " << graph->isWeightedEdges() << endl;
    cout << endl;

    cout << "Nodes:" << endl;
    printNodes(graph);

    cout << "\n Edges:" << endl;
    printEdges(graph);
}

void Writer::printGraphInDotFile(Graph* graph, string fileName) {
    vector<string> dotFile;
    string rawFileName = getRawFileName(fileName);

    if (graph->isDirected()) {
        dotFile = getDigraphDotFile(graph);
    } else {
        dotFile = getGraphDotFile(graph);
    }

    if (!existsFolder(output_directory)) {
        mkdirSync(output_directory);
    }

    string filePath = output_directory + rawFileName + ".dot";

    ofstream file(filePath);

    for (string line : dotFile) {
        file << line << endl;
    }

    file.close();

    cout << "File saved in: " << filePath << endl;

    string command = "sfdp -x -Goverlap=scale  -Tsvg " + filePath + " > " + output_directory + rawFileName + ".svg";

    system(command.c_str());

    cout << "Image saved in: " << output_directory << rawFileName << ".svg" << endl;

    cout << endl;
}

vector<string> Writer::getGraphDotFile(Graph* graph) {
    vector<string> dotFile;

    dotFile.push_back("graph {");

    Node* currentNode = graph->getFirstNode();
    while (currentNode != nullptr) {
        string nodeLine = "    " + to_string(currentNode->getId()) + " [label=\"" + to_string(currentNode->getId()) + "\"";

        if (graph->isWeightedNodes()) {
            nodeLine += ", weight=" + to_string(currentNode->getWeight());
        }

        nodeLine += "];";

        dotFile.push_back(nodeLine);

        currentNode = currentNode->getNextNode();
    }

    vector<Edge*> edges = graph->getEdges();

    for (Edge* edge : edges) {
        string edgeLine = "    " + to_string(edge->getHead()->getId()) + " -- " + to_string(edge->getTail()->getId());

        if (graph->isWeightedEdges()) {
            edgeLine += " [label=\"" + to_string(edge->getWeight()) + "\"]";
        }

        edgeLine += ";";

        dotFile.push_back(edgeLine);
    }

    dotFile.push_back("}");

    return dotFile;
}

vector<string> Writer::getDigraphDotFile(Graph* graph) {
    vector<string> dotFile;

    dotFile.push_back("digraph {");

    Node* currentNode = graph->getFirstNode();
    while (currentNode != nullptr) {
        string nodeLine = "    " + to_string(currentNode->getId()) + " [label=\"" + to_string(currentNode->getId()) + "\"";

        if (graph->isWeightedNodes()) {
            nodeLine += ", weight=" + to_string(currentNode->getWeight());
        }

        nodeLine += "];";

        dotFile.push_back(nodeLine);

        currentNode = currentNode->getNextNode();
    }

    vector<Edge*> edges = graph->getEdges();

    for (Edge* edge : edges) {
        string edgeLine = "    " + to_string(edge->getHead()->getId()) + " -> " + to_string(edge->getTail()->getId());

        if (graph->isWeightedEdges()) {
            edgeLine += " [label=\"" + to_string(edge->getWeight()) + "\"]";
        }

        edgeLine += ";";

        dotFile.push_back(edgeLine);
    }

    dotFile.push_back("}");

    return dotFile;
}

void Writer::printGraphInTxtFile(Graph* graph, string fileName) {
    vector<Edge*> edges = graph->getEdges();
    Node* node = graph->getFirstNode();
    string rawFileName = getRawFileName(fileName);

    string filePath = output_directory + rawFileName + ".txt";

    ofstream file(filePath);

    file << graph->getNumNodes() << endl;

    for (Edge* edge : edges) {
        file << edge->getHead()->getId() << " " << edge->getTail()->getId() << " " << edge->getWeight() << endl;
    }

    file.close();

    cout << "File saved in: " << filePath << endl;
}

void Writer::printVectorNodes(vector<Node*> nodes) {
    cout << "{";

    for (int i = 0; i < nodes.size(); i++) {
        cout << nodes[i]->getId();

        if (i == nodes.size() - 1) {
            cout << "}\n";
            break;
        }

        cout << ", ";
    }
}

void Writer::printVectorEdges(vector<Edge*> edges) {
    for (Edge* edge : edges) {
        cout << "(" << edge->getHead()->getId() << ", " << edge->getTail()->getId() << ") ";
    }
}