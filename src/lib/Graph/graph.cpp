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
        std::cout << "Invalid nodes for searching an edge." << std::endl;
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

Edge* Graph::createOrUpdateEdge(Node* head, Node* tail, int weight) {
    if (head == nullptr || tail == nullptr) {
        std::cout << "Invalid nodes for creating an edge." << std::endl;
        return nullptr;
    }

    Edge* existingEdge = nullptr;

    for (Edge* edge : edges) {
        if (edge->getHead() == head && edge->getTail() == tail) {
            existingEdge = edge;
            break;
        }
    }

    if (existingEdge != nullptr) {
        deleteEdge(existingEdge);
    }

    Edge* newEdge = new Edge(head, tail, weight);

    head->addEdge(newEdge);

    if (directed) {
        tail->incrementDegreeIn();
    } else {
        Edge* reverseEdge = new Edge(tail, head, weight);
        tail->addEdge(reverseEdge);
        tail->incrementDegreeIn();
        head->incrementDegreeIn();
    }

    edges.push_back(newEdge);
    totalEdges++;
    return newEdge;
}

void Graph::deleteEdge(Edge* edge) {
    if (edge == nullptr) {
        std::cout << "Edge is null." << std::endl;
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
        std::cout << "Node is null." << std::endl;
        return;
    }

    std::vector<Edge*> edgesToRemove = node->getEdges();
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

    std::cout << "Node not found in the graph." << std::endl;
}
