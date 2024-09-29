#include "MGGPPAlgorithms.hpp"

#include <algorithm>
#include <iostream>
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "../../lib/Edge/edge.hpp"
#include "../../lib/Graph/graph.hpp"
#include "../../lib/Node/node.hpp"

vector<Graph*> getMGGPPByGreedyAlgorithm(Graph* graph, int numClusters) {
    int clusterWeightLimit = getClusterWeightLimit(graph, numClusters);
    int clusterWeight = 0;
    Node *nextNode, *oldNode, *oneDegreeNode, *currentNode;

    vector<Graph*> clusters(numClusters);
    unordered_map<int, int> nodesDegree = getNodeDegreeMap(graph);

    for (int clusterIndex = 0; graph->getFirstNode() != nullptr; clusterIndex++) {
        cout << "Iniciando cluster " << clusterIndex << endl;

        nodesDegree = getNodeDegreeMap(graph);
        Graph* cluster = new Graph(graph->isDirected(), graph->isWeightedEdges(), graph->isWeightedNodes());
        clusters[clusterIndex] = cluster;
        clusterWeight = 0;

        currentNode = graph->findNodeById(getSmallerDegreeNode(nodesDegree));
        cluster->createOrUpdateNode(currentNode->getId(), currentNode->getWeight());
        cout << "Adicionando nó inicial " << currentNode->getId() << " ao cluster com peso " << currentNode->getWeight() << endl;
        clusterWeight += currentNode->getWeight();

        while (clusterWeight < clusterWeightLimit || cluster->getNumNodes() < 2) {
            Node* nextNode = getNextNode(currentNode);  // 15

            if (nextNode == nullptr) {
                cout << "Próximo nó é nulo. Saindo do loop." << endl;
                break;
            }

            addNodeToCluster(cluster, nextNode, currentNode, clusterWeight);

            if (nextNode->getDegreeOut() == 1) {
                cout << "O nó " << nextNode->getId() << " tem grau 1. Removendo do grafo." << endl;
                graph->deleteNode(nextNode);
                nodesDegree.erase(nextNode->getId());
                continue;
            }

            oldNode = currentNode;
            currentNode = nextNode;

            cout << "Removendo nó " << oldNode->getId() << " do grafo." << endl;
            graph->deleteNode(oldNode);
            nodesDegree.erase(oldNode->getId());

            cout << "Peso do cluster: " << clusterWeight << endl;
        }

        oneDegreeNode = checkIfNodeHasDegreeOne(currentNode);
        if (oneDegreeNode != nullptr) {
            addNodeToCluster(cluster, oneDegreeNode, currentNode, clusterWeight);
            cout << "Adicionando nó com grau 1 " << oneDegreeNode->getId() << " ao cluster." << endl;
            graph->deleteNode(oneDegreeNode);
        }

        graph->deleteNode(currentNode);
        nodesDegree.erase(currentNode->getId());
    }

    return clusters;
}

std::unordered_map<int, int> getNodeDegreeMap(Graph* graph) {
    std::unordered_map<int, int> nodesDegree;

    Node* currentNode = graph->getFirstNode();

    while (currentNode != nullptr) {
        nodesDegree[currentNode->getId()] = currentNode->getDegreeOut();
        cout << "Nó " << currentNode->getId() << " tem grau " << currentNode->getDegreeOut() << endl;
        currentNode = currentNode->getNextNode();
    }

    return nodesDegree;
}

int getClusterWeightLimit(Graph* graph, int numClusters) {
    int clusterWeight = 0;

    Node* currentNode = graph->getFirstNode();
    while (currentNode != nullptr) {
        cout << "Nó " << currentNode->getId() << " tem peso " << currentNode->getWeight() << endl;
        clusterWeight += currentNode->getWeight();
        currentNode = currentNode->getNextNode();
    }

    int weightLimit = clusterWeight / numClusters;
    cout << "Limite de peso do cluster: " << weightLimit << endl;
    return weightLimit;
}

/**
 * Se o vértice do vizinho tiver grau 1, ele será selecionado.
 * Se o vértice do vizinho tiver grau maior que 1, será selecionado o vértice de maior peso.
 */
Node* getNextNode(Node* currentNode) {
    vector<Node*> neighbors = currentNode->getNeighbors();

    Node* nextNode = nullptr;
    int maxWeight = 0;

    for (Node* neighbor : neighbors) {
        if (neighbor->getDegreeOut() == 1) {
            cout << "Vizinho " << neighbor->getId() << " tem grau 1. Selecionando." << endl;
            return neighbor;
        }

        if (neighbor->getWeight() > maxWeight) {
            maxWeight = neighbor->getWeight();
            nextNode = neighbor;
        }
    }

    if (nextNode != nullptr) {
        cout << "Próximo nó selecionado é " << nextNode->getId() << " com peso " << nextNode->getWeight() << endl;
    } else {
        cout << "Nenhum próximo nó foi selecionado." << endl;
    }

    return nextNode;
}

/**
 * Verifica se o nó tem grau 1 e retorna o nó vizinho que tem grau 1.
 */
Node* checkIfNodeHasDegreeOne(Node* currentNode) {
    vector<Node*> neighbors = currentNode->getNeighbors();

    for (Node* neighbor : neighbors) {
        if (neighbor->getDegreeOut() == 1) {
            cout << "Nó vizinho " << neighbor->getId() << " tem grau 1." << endl;
            return neighbor;
        }
    }

    return nullptr;
}

void deleteNodeFromGraph(Graph* graph, Node* node, std::unordered_map<int, int>& nodesDegree) {
    graph->deleteNode(node);
    nodesDegree.erase(node->getId());
}

void addNodeToCluster(Graph* cluster, Node* node, Node* parentNode, int& clusterWeight) {
    cout << "Adicionando nó " << node->getId() << " ao cluster com peso " << node->getWeight() << endl;
    cluster->createOrUpdateNode(node->getId(), node->getWeight());
    cluster->createEdge(node, parentNode, 1);
    clusterWeight += node->getWeight();
}

int getSmallerDegreeNode(std::unordered_map<int, int>& nodesDegree) {
    int smallerDegree = 501;
    int smallerDegreeNode = -1;

    for (auto node : nodesDegree) {
        if (node.second < smallerDegree) {
            smallerDegree = node.second;
            smallerDegreeNode = node.first;
        }
    }

    return smallerDegreeNode;
}

Graph* getMGGPPByGRASPAlgorithm(Graph* graph, int numClusters) {
    return graph;
}

Graph* getMGGPPByReactiveGRASPAlgorithm(Graph* graph, int numClusters) {
    return graph;
}
