#include "../Graph/graph.hpp"

#include <iostream>

#include "../Edge/edge.hpp"
#include "../Node/node.hpp"

Graph::Graph(bool directed, bool weightedEdge, bool weightedNodes)
    : firstNode(nullptr), totalNodes(0), totalEdges(0), weightedEdges(weightedEdge), weightedNodes(weightedNodes), directed(directed) {}

Graph::~Graph() {
    Node* currentNode = firstNode;
    while (currentNode != nullptr) {
        Node* nextNode = currentNode->getNextNode();
        delete currentNode;
        currentNode = nextNode;
    }

    for (Edge* edge : edges) {
        delete edge;
    }
}

Node* Graph::getFirstNode() {
    return firstNode;
}

vector<Edge*> Graph::getEdges() {
    return edges;
}

int Graph::getNumNodes() {
    return totalNodes;
}

int Graph::getNumEdges() {
    return totalEdges;
}

bool Graph::isWeightedEdges() {
    return weightedEdges;
}

bool Graph::isWeightedNodes() {
    return weightedNodes;
}

bool Graph::isDirected() {
    return directed;
}

Node* Graph::findNodeById(int id) {
    Node* currentNode = firstNode;
    while (currentNode != nullptr) {
        if (currentNode->getId() == id) {
            return currentNode;
        }
        currentNode = currentNode->getNextNode();
    }
    return nullptr;
}

Edge* Graph::findEdgeByNodes(Node* head, Node* tail) {
    if (head == nullptr || tail == nullptr) {
        cout << "Invalid nodes for searching an edge." << endl;
        return nullptr;
    }

    for (Edge* edge : edges) {
        if (edge->getHead() == head && edge->getTail() == tail) {
            return edge;
        }
    }
    return nullptr;
}

Node* Graph::createOrUpdateNode(int id, int weight) {
    Node* existingNode = nullptr;

    Node* currentNode = firstNode;
    while (currentNode != nullptr) {
        if (currentNode->getId() == id) {
            existingNode = currentNode;
            break;
        }
        currentNode = currentNode->getNextNode();
    }

    if (existingNode != nullptr) {
        existingNode->setWeight(weight);
        return existingNode;
    }

    Node* newNode = new Node(id, weight);
    newNode->setNextNode(firstNode);
    firstNode = newNode;
    totalNodes++;
    return newNode;
}

Edge* Graph::createEdge(Node* head, Node* tail, int weight) {
    if (head == nullptr || tail == nullptr) {
        cout << "Invalid nodes for creating an edge." << endl;
        return nullptr;
    }

    Edge* newEdge = new Edge(head, tail, weight);

    head->addEdge(newEdge);

    if (directed) {
        tail->incrementDegreeIn();
    } else {
        Edge* reverseEdge = new Edge(tail, head, weight);
        tail->addEdge(reverseEdge);

        totalEdges++;
        tail->incrementDegreeIn();
        head->incrementDegreeIn();
    }

    edges.push_back(newEdge);
    totalEdges++;
    return newEdge;
}

void Graph::deleteEdge(Edge* edge) {
    if (edge == nullptr) {
        cout << "Edge is null." << endl;
        return;
    }

    Node* headNode = edge->getHead();
    Node* tailNode = edge->getTail();

    headNode->removeEdge(edge);

    if (headNode != tailNode) {
        tailNode->removeEdge(edge);
    }

    for (auto it = edges.begin(); it != edges.end(); ++it) {
        if (*it == edge) {
            edges.erase(it);
            break;
        }
    }

    totalEdges = edges.size();

    delete edge;
}

void Graph::deleteNode(Node* node) {
    if (node == nullptr) {
        cout << "Node is null." << endl;
        return;
    }

    vector<Edge*> edgesToRemove = node->getEdges();
    for (Edge* edge : edgesToRemove) {
        deleteEdge(edge);
    }

    Node* prevNode = nullptr;
    Node* currentNode = firstNode;
    while (currentNode != nullptr) {
        if (currentNode == node) {
            if (prevNode != nullptr) {
                prevNode->setNextNode(currentNode->getNextNode());
            } else {
                firstNode = currentNode->getNextNode();
            }
            totalNodes--;
            delete currentNode;
            return;
        }
        prevNode = currentNode;
        currentNode = currentNode->getNextNode();
    }

    cout << "Node not found in the graph." << endl;
}

int Graph::getDegree() {
    int maxDegree = 0;
    Node* nodeWithHighestDegree = nullptr;

    Node* currentNode = firstNode;
    while (currentNode != nullptr) {
        if (currentNode->getDegree(isDirected()) > maxDegree) {
            maxDegree = currentNode->getDegree(isDirected());
            nodeWithHighestDegree = currentNode;
        }
        currentNode = currentNode->getNextNode();
    }

    return nodeWithHighestDegree->getId();
}