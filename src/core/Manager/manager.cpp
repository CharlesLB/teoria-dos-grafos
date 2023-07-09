
#include "./manager.hpp"

#include <dirent.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
#include <sstream>

#include "../../helpers/Validators/validators.hpp"
#include "../../lib/Edge/edge.hpp"
#include "../../lib/Graph/graph.hpp"
#include "../../lib/Node/node.hpp"
#include "../Reader/reader.hpp"
#include "../Writer/writer.hpp"

using namespace std;

void Manager::processOperation(Graph* graph) {
    char option;

    string options[] = {"Adicionar nó", "Adicionar aresta", "Remover nó", "Remover aresta", "Exit", ""};

    while (true) {
        Writer::printMenu(options);
        option = Reader::readChar();

        switch (option) {
            case 'a':
                createOrUpdateNode(graph);
                break;

            case 'b':
                createEdge(graph);
                break;

            case 'c':
                removeNode(graph);
                break;

            case 'd':
                removeEdge(graph);
                break;

            case 'e':
                return;

            default:
                cout << "Invalid option\n";
                break;
        }
    }
}

void Manager::createOrUpdateNode(Graph* graph) {
    int id, weight = 1;
    bool exists = false;

    cout << "Enter the node's id (int): ";
    id = Reader::readInt();

    if (graph->findNodeById(id) != NULL) {
        exists = true;

        if (graph->isWeightedNodes()) {
            cout << "Node already exists. Updating node's weight...\n";
        } else {
            cout << "Node already exists. Skipping...\n";
            return;
        }
    }

    if (graph->isWeightedNodes()) {
        cout << "Enter the node's weight (int): ";
        weight = Reader::readInt();
    }

    graph->createOrUpdateNode(id, weight);

    exists ? cout << "Node updated successfully\n" : cout << "Node created successfully\n";
}

void Manager::createEdge(Graph* graph) {
    int headId, tailId, weight = 1;
    Node* head;
    Node* tail;
    bool exists = false;

    while (head == nullptr) {
        cout << "Enter the head node's id (int): ";
        headId = Reader::readInt();

        head = graph->findNodeById(headId);

        if (head == nullptr) {
            cout << "Node doesn't exist. Please try again.\n";
        }
    }

    while (tail == nullptr) {
        cout << "Enter the tail node's id (int): ";
        tailId = Reader::readInt();

        tail = graph->findNodeById(tailId);

        if (tail == nullptr) {
            cout << "Node doesn't exist. Please try again.\n";
        }
    }

    if (graph->findEdgeByNodes(head, tail) != NULL) {
        exists = true;

        if (graph->isWeightedEdges()) {
            cout << "Edge already exists. Updating edge's weight...\n";
        } else {
            cout << "Edge already exists. Skipping...\n";
            return;
        }
    }

    if (graph->isWeightedEdges()) {
        cout << "Enter the edge's weight (int): ";
        weight = Reader::readInt();
    }

    graph->createEdge(head, tail, weight);

    exists ? cout << "Edge updated successfully\n" : cout << "Edge created successfully\n";
}

void Manager::removeNode(Graph* graph) {
    int id;
    Node* node;

    cout << "Enter the node's id (int): ";
    id = Reader::readInt();

    node = graph->findNodeById(id);

    if (node == nullptr) {
        cout << "Node doesn't exist. Skipping...\n";
        return;
    }

    graph->deleteNode(node);

    cout << "Node removed successfully.\n";
}

void Manager::removeEdge(Graph* graph) {
    int headId, tailId;
    Node* head;
    Node* tail;
    Edge* edge;

    while (head == nullptr) {
        cout << "Enter the head node's id (int): ";
        headId = Reader::readInt();

        head = graph->findNodeById(headId);

        if (head == nullptr) {
            cout << "Node doesn't exist. Please try again.\n";
        }
    }

    while (tail == nullptr) {
        cout << "Enter the tail node's id (int): ";
        tailId = Reader::readInt();

        tail = graph->findNodeById(tailId);

        if (tail == nullptr) {
            cout << "Node doesn't exist. Please try again.\n";
        }
    }

    edge = graph->findEdgeByNodes(head, tail);

    if (edge == nullptr) {
        cout << "Edge doesn't exist. Skipping...\n";
        return;
    }

    graph->deleteEdge(edge);

    cout << "Edge removed successfully.\n";
}

void Manager::getNodeDegree(Graph* graph) {
    int id;
    Node* node;

    cout << "Enter the node's id (int): ";
    id = Reader::readInt();

    node = graph->findNodeById(id);

    if (node == nullptr) {
        cout << "Node doesn't exist. Skipping...\n";
        return;
    }

    if (graph->isDirected()) {
        cout << "Node's degree in: " << node->getDegreeIn() << endl;
        cout << "Node's degree out: " << node->getDegreeOut() << endl;
    } else {
        cout << "Node's degree: " << node->getDegree(graph->isDirected()) / 2 << endl;
    }
}