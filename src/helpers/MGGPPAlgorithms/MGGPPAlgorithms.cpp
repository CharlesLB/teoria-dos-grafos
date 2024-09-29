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
    vector<Graph*> clusters;
    vector<Node*> uninsertedNodes;
    unordered_set<int> visitedNodes;  // Conjunto de nós já visitados
    Node* currentNode = graph->getFirstNode();

    // Calcula o peso total dos vértices no grafo
    int totalWeight = 0;
    while (currentNode != nullptr) {
        totalWeight += currentNode->getWeight();
        uninsertedNodes.push_back(currentNode);
        currentNode = currentNode->getNextNode();
    }

    // Ordena os nós por peso decrescente para começar pelo maior peso
    sort(uninsertedNodes.begin(), uninsertedNodes.end(), [](Node* a, Node* b) {
        return a->getWeight() > b->getWeight();
    });

    // Peso alvo por cluster
    int targetWeightPerCluster = totalWeight / numClusters;

    // Inicializa clusters vazios
    for (int i = 0; i < numClusters; ++i) {
        clusters.push_back(new Graph(graph->isDirected(), graph->isWeightedEdges(), graph->isWeightedNodes()));
    }

    // Variável para acompanhar o peso acumulado em cada cluster
    vector<int> clusterWeights(numClusters, 0);

    // Itera sobre todos os nós, começando do maior peso
    int clusterIndex = 0;
    while (!uninsertedNodes.empty() && clusterIndex < numClusters) {
        Node* currentNode = uninsertedNodes.front();     // Começa com o nó de maior peso
        uninsertedNodes.erase(uninsertedNodes.begin());  // Remove da lista de não inseridos

        if (visitedNodes.count(currentNode->getId())) {
            continue;  // Pula nós já visitados
        }

        Graph* currentCluster = clusters[clusterIndex];
        currentCluster->createOrUpdateNode(currentNode->getId(), currentNode->getWeight());
        clusterWeights[clusterIndex] += currentNode->getWeight();
        visitedNodes.insert(currentNode->getId());  // Marca o nó como visitado

        // Insere todas as arestas conectadas ao nó atual no cluster, mas respeita o limite de peso
        vector<Node*> connectedNodes;
        vector<Edge*> nodeEdges = currentNode->getEdges();
        for (Edge* edge : nodeEdges) {
            Node* head = edge->getHead();
            Node* tail = edge->getTail();
            Node* connectedNode = (head == currentNode) ? tail : head;

            // Se o nó conectado ainda não foi visitado
            if (!visitedNodes.count(connectedNode->getId())) {
                if (clusterWeights[clusterIndex] + connectedNode->getWeight() <= targetWeightPerCluster) {
                    // Adiciona o nó conectado no cluster
                    currentCluster->createOrUpdateNode(connectedNode->getId(), connectedNode->getWeight());
                    currentCluster->createEdge(currentNode, connectedNode, edge->getWeight());
                    clusterWeights[clusterIndex] += connectedNode->getWeight();
                    visitedNodes.insert(connectedNode->getId());
                    connectedNodes.push_back(connectedNode);
                } else {
                    // Se atingir o limite de peso, pula para o próximo cluster
                    uninsertedNodes.push_back(connectedNode);
                }
            }
        }

        // Se o peso do cluster atingiu o limite, passa para o próximo cluster
        if (clusterWeights[clusterIndex] >= targetWeightPerCluster && clusterIndex < numClusters - 1) {
            clusterIndex++;
        }

        // Continua com os próximos nós conectados (bfs-like)
        for (Node* nextNode : connectedNodes) {
            uninsertedNodes.push_back(nextNode);
        }
    }

    return clusters;
}

// Função auxiliar para encontrar um nó conectado
Node* findConnectedNode(Graph* graph, Node* node, const vector<Node*>& nodeList) {
    for (Node* potentialNode : nodeList) {
        if (graph->findEdgeByNodes(node, potentialNode) != nullptr) {
            cout << "Nó " << node->getId() << " está conectado ao nó " << potentialNode->getId() << "." << endl;
            return potentialNode;
        }
    }
    cout << "Nó " << node->getId() << " não está conectado a nenhum dos nós restantes." << endl;
    return nullptr;
}

// Função auxiliar para encontrar um nó no cluster conectado a um novo nó
Node* findNodeConnectedTo(Graph* graph, Node* newNode, Graph* cluster) {
    Node* clusterNode = cluster->getFirstNode();
    while (clusterNode != nullptr) {
        if (graph->findEdgeByNodes(newNode, clusterNode) != nullptr) {
            cout << "Nó " << newNode->getId() << " está conectado ao nó " << clusterNode->getId() << " no cluster." << endl;
            return clusterNode;
        }
        clusterNode = clusterNode->getNextNode();
    }
    return nullptr;
}

Graph* getMGGPPByGRASPAlgorithm(Graph* graph, int numClusters) {
    return graph;
}

Graph* getMGGPPByReactiveGRASPAlgorithm(Graph* graph, int numClusters) {
    return graph;
}

Graph* mergeGraphs(vector<Graph*> subgraphs) {
    if (subgraphs.empty()) {
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
            }

            vector<Edge*> edges = currentNode->getEdges();
            for (Edge* edge : edges) {
                Node* head = edge->getHead();
                Node* tail = edge->getTail();

                if (mergedGraph->findNodeById(head->getId()) && mergedGraph->findNodeById(tail->getId())) {
                    mergedGraph->createEdge(head, tail, edge->getWeight());
                }
            }

            currentNode = currentNode->getNextNode();
        }
    }

    return mergedGraph;
}