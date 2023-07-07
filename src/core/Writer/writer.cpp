#include "./writer.hpp"

#include <time.h>

#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "../../lib/Edge/edge.hpp"
#include "../../lib/Graph/graph.hpp"
#include "../../lib/Node/node.hpp"

using namespace std;

void Writer::printMenu(string* options) {
    std::cout << "\nChoose an option:\n"
              << std::endl;

    char index = 'a';
    for (const std::string* option = options; *option != ""; option++) {
        std::cout << "(" << index << ") " << *option << std::endl;
        index++;
    }

    std::cout << "(0) Exit" << std::endl;
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