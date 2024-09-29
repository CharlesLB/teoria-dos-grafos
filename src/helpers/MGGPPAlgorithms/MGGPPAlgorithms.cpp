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
    Node *nextNode, *oldNode, *oneDegreeNode;

    vector<Graph*> clusters(numClusters);
    vector<pair<int, int>> nodesDegree = getNodeDegreeOrdered(graph);

    for (int clusterIndex = 0; graph->getFirstNode() != nullptr; clusterIndex++) {
        cout << "Iniciando cluster " << clusterIndex << endl;

        // printar o nodesDegree
        for (auto node : nodesDegree) {
            cout << "Nó " << node.first << " tem grau " << node.second << endl;
        }

        Graph* cluster = new Graph(graph->isDirected(), graph->isWeightedEdges(), graph->isWeightedNodes());
        clusters[clusterIndex] = cluster;
        clusterWeight = 0;

        Node* currentNode = graph->findNodeById(nodesDegree[0].first);
        cluster->createOrUpdateNode(currentNode->getId(), currentNode->getWeight());
        cout << "Adicionando nó inicial " << currentNode->getId() << " ao cluster com peso " << currentNode->getWeight() << endl;

        while (clusterWeight < clusterWeightLimit) {
            Node* nextNode = getNextNode(currentNode);

            if (nextNode == nullptr) {
                cout << "Próximo nó é nulo. Saindo do loop." << endl;
                break;
            }

            addNodeToCluster(cluster, nextNode, currentNode, clusterWeight);
            cout << "Adicionando nó " << nextNode->getId() << " ao cluster com peso " << nextNode->getWeight() << endl;

            if (nextNode->getDegree(false) == 1) {
                cout << "O nó " << nextNode->getId() << " tem grau 1. Removendo do grafo." << endl;
                deleteNodeFromGraph(graph, currentNode, nodesDegree);
                continue;
            }

            oldNode = currentNode;
            currentNode = nextNode;

            cout << "Removendo nó " << oldNode->getId() << " do grafo." << endl;
            deleteNodeFromGraph(graph, oldNode, nodesDegree);
        }

        oneDegreeNode = checkIfNodeHasDegreeOne(currentNode);
        if (oneDegreeNode != nullptr) {
            addNodeToCluster(cluster, oneDegreeNode, currentNode, clusterWeight);
            cout << "Adicionando nó com grau 1 " << oneDegreeNode->getId() << " ao cluster." << endl;
            graph->deleteNode(oneDegreeNode);
        }
    }

    return clusters;
}

// Funções auxiliares com logs

vector<pair<int, int>> getNodeDegreeOrdered(Graph* graph) {
    vector<pair<int, int>> nodesDegree;

    Node* currentNode = graph->getFirstNode();

    while (currentNode != nullptr) {
        nodesDegree.push_back({currentNode->getId(), currentNode->getDegree(false)});
        cout << "Nó " << currentNode->getId() << " tem grau " << currentNode->getDegree(false) << endl;
        currentNode = currentNode->getNextNode();
    }

    sort(nodesDegree.begin(), nodesDegree.end(),
         [](const pair<int, int>& a, const pair<int, int>& b) {
             return a.second < b.second;
         });

    return nodesDegree;
}

int getClusterWeightLimit(Graph* graph, int numClusters) {
    int clusterWeight = 0;

    Node* currentNode = graph->getFirstNode();
    for (int i = 0; i < numClusters; i++) {
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
        if (neighbor->getDegree(false) == 1) {
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
        if (neighbor->getDegree(false) == 1) {
            cout << "Nó vizinho " << neighbor->getId() << " tem grau 1." << endl;
            return neighbor;
        }
    }

    return nullptr;
}

void deleteNodeFromGraph(Graph* graph, Node* node, vector<pair<int, int>>& nodesDegree) {
    cout << "Deletando nó " << node->getId() << " do grafo." << endl;
    graph->deleteNode(node);
    nodesDegree.erase(
        std::remove_if(nodesDegree.begin(), nodesDegree.end(),
                       [&](const std::pair<int, int>& p) { return p.first == node->getId(); }),
        nodesDegree.end());

    for (auto node : nodesDegree) {
        cout << "Nó " << node.first << " tem grau " << node.second << endl;
    }
}

void addNodeToCluster(Graph* cluster, Node* node, Node* parentNode, int& clusterWeight) {
    cout << "Adicionando nó " << node->getId() << " ao cluster com peso " << node->getWeight() << endl;
    cluster->createOrUpdateNode(node->getId(), node->getWeight());
    cluster->createEdge(node, parentNode, 1);
    clusterWeight += node->getWeight();
}

Graph* mergeGraphs(vector<Graph*> subgraphs) {
    if (subgraphs.empty()) {
        cout << "Nenhum subgrafo para mesclar." << endl;
        return nullptr;
    }

    Graph* mergedGraph = new Graph(subgraphs[0]->isDirected(), subgraphs[0]->isWeightedEdges(), subgraphs[0]->isWeightedNodes());

    set<int> insertedNodeIds;

    for (Graph* subgraph : subgraphs) {
        Node* currentNode = subgraph->getFirstNode();

        while (currentNode != nullptr) {
            int nodeId = currentNode->getId();

            if (insertedNodeIds.find(nodeId) == insertedNodeIds.end()) {
                mergedGraph->createOrUpdateNode(nodeId, currentNode->getWeight());
                insertedNodeIds.insert(nodeId);
                cout << "Nó " << nodeId << " adicionado ao grafo mesclado." << endl;
            }

            vector<Edge*> edges = currentNode->getEdges();
            for (Edge* edge : edges) {
                Node* head = edge->getHead();
                Node* tail = edge->getTail();

                if (mergedGraph->findNodeById(head->getId()) && mergedGraph->findNodeById(tail->getId())) {
                    mergedGraph->createEdge(head, tail, edge->getWeight());
                    cout << "Aresta criada entre " << head->getId() << " e " << tail->getId() << endl;
                }
            }

            currentNode = currentNode->getNextNode();
        }
    }

    return mergedGraph;
}

Graph* getMGGPPByGRASPAlgorithm(Graph* graph, int numClusters) {
    return graph;
}

Graph* getMGGPPByReactiveGRASPAlgorithm(Graph* graph, int numClusters) {
    return graph;
}
