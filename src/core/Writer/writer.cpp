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

using namespace std;

string output_directory = "./output/";

void Writer::printMenu(string* options) {
    std::cout << "\nChoose an option:\n"
              << std::endl;

    char index = 'a';
    for (const std::string* option = options; *option != ""; option++) {
        std::cout << "(" << index << ") " << *option << std::endl;

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
        std::cout << "Node ID: " << currentNode->getId() << std::endl;
        std::cout << "Weight: " << currentNode->getWeight() << std::endl;
        std::cout << "Degree In: " << currentNode->getDegreeIn() << std::endl;
        std::cout << "Degree Out: " << currentNode->getDegreeOut() << std::endl;
        std::cout << std::endl;

        currentNode = currentNode->getNextNode();
    }
}

void Writer::printEdges(Graph* graph) {
    std::vector<Edge*> edges = graph->getEdges();

    for (Edge* edge : edges) {
        std::cout << "Weight: " << edge->getWeight() << std::endl;
        std::cout << "Head Node ID: " << edge->getHead()->getId() << std::endl;
        std::cout << "Tail Node ID: " << edge->getTail()->getId() << std::endl;
        std::cout << std::endl;
    }
}

void Writer::printGraph(Graph* graph) {
    std::cout << "Directed: " << graph->isDirected() << std::endl;
    std::cout << "Weighted Nodes: " << graph->isWeightedNodes() << std::endl;
    std::cout << "Weighted Edges: " << graph->isWeightedEdges() << std::endl;
    std::cout << std::endl;

    std::cout << "Nodes:" << std::endl;
    printNodes(graph);

    std::cout << "\n Edges:" << std::endl;
    printEdges(graph);
}

void Writer::printGraphInDotFile(Graph* graph, string fileName) {
    std::vector<string> dotFile;

    if (graph->isDirected()) {
        dotFile = getDigraphDotFile(graph);
    } else {
        dotFile = getGraphDotFile(graph);
    }

    if (!existsFolder(output_directory)) {
        mkdirSync(output_directory);
    }

    string filePath = output_directory + fileName + ".dot";

    ofstream file(filePath);

    for (string line : dotFile) {
        file << line << endl;
    }

    file.close();

    cout << "File saved in: " << filePath << endl;

    string command = "sfdp -x -Goverlap=scale  -Tsvg " + filePath + " > " + output_directory + fileName + ".svg";

    system(command.c_str());

    cout << "Image saved in: " << output_directory << fileName << ".png" << endl;

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

    std::vector<Edge*> edges = graph->getEdges();

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

    std::vector<Edge*> edges = graph->getEdges();

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
