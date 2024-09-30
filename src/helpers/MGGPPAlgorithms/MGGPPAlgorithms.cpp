#include "MGGPPAlgorithms.hpp"

#include <math.h>

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

vector<Graph*> getMGGPPByGreedyAlgorithm(Graph* graph, int numClusters, float alpha) {
    int clusterWeight = 0;
    bool breakLoop = false;
    Node *nextNode, *oldNode, *oneDegreeNode, *currentNode;

    vector<Graph*> clusters(numClusters);
    unordered_map<int, int> nodesDegree = getNodeDegreeMap(graph);

    for (int clusterIndex = 0; graph->getFirstNode() != nullptr; clusterIndex++, breakLoop = false) {
        pair<int, int> info = getClusterWeightLimitAndGraphWeightAverage(graph, numClusters - clusterIndex);

        int clusterWeightLimit = info.first;
        int graphWeightAverage = info.second;

        cout << "\n\n\nIniciando cluster " << clusterIndex << "   com peso:" << clusterWeightLimit << endl;

        nodesDegree = getNodeDegreeMap(graph);
        Graph* cluster = new Graph(graph->isDirected(), graph->isWeightedEdges(), graph->isWeightedNodes());
        clusters[clusterIndex] = cluster;
        clusterWeight = 0;

        currentNode = graph->findNodeById(getSmallerDegreeNodeId(nodesDegree, alpha));
        cluster->createOrUpdateNode(currentNode->getId(), currentNode->getWeight());
        cout << "Adicionando nó inicial " << currentNode->getId() << " ao cluster com peso " << currentNode->getWeight() << endl;
        clusterWeight += currentNode->getWeight();

        while ((clusterWeight < clusterWeightLimit || cluster->getNumNodes() < 2)) {
            Node* nextNode = getNextNode(currentNode, graphWeightAverage);

            if (nextNode == nullptr) {
                breakLoop = true;
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

        if (breakLoop) {
            break;
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

unordered_map<int, int> getNodeDegreeMap(Graph* graph) {
    unordered_map<int, int> nodesDegree;

    Node* currentNode = graph->getFirstNode();

    while (currentNode != nullptr) {
        nodesDegree[currentNode->getId()] = currentNode->getDegreeOut();
        currentNode = currentNode->getNextNode();
    }

    return nodesDegree;
}

pair<int, int> getClusterWeightLimitAndGraphWeightAverage(Graph* graph, int numClusters) {
    int graphWeight = 0;

    Node* currentNode = graph->getFirstNode();
    while (currentNode != nullptr) {
        cout << "Nó " << currentNode->getId() << " tem peso " << currentNode->getWeight() << endl;
        graphWeight += currentNode->getWeight();
        currentNode = currentNode->getNextNode();
    }

    int clusterWeightLimit = graphWeight / numClusters;
    int graphWeightAverage = graphWeight / graph->getNumNodes();
    cout << "Limite de peso do cluster: " << clusterWeightLimit << endl;
    return make_pair(clusterWeightLimit, graphWeightAverage);
}

int getClusterSizeLimit(Graph* graph, int numClusters) {
    int totalNodes = graph->getNumNodes();

    return ceil(totalNodes / numClusters);
}

/**
 * Se o vértice do vizinho tiver grau 1, ele será selecionado.
 * Se o vértice do vizinho tiver grau maior que 1, será selecionado o vértice de maior peso.
 */
Node* getNextNode(Node* currentNode, int graphWeightAverage) {
    vector<Node*> neighbors = currentNode->getNeighbors();

    bool aboveAverage = currentNode->getWeight() >= graphWeightAverage;

    Node* nextNode = nullptr;
    int maxWeight = 0;
    int minWeight = 501;

    for (Node* neighbor : neighbors) {
        if (neighbor->getDegreeOut() == 1) {
            cout << "Vizinho " << neighbor->getId() << " tem grau 1. Selecionando." << endl;
            return neighbor;
        }

        if (neighbor->getWeight() > maxWeight && aboveAverage) {
            maxWeight = neighbor->getWeight();
            nextNode = neighbor;
        }

        if (neighbor->getWeight() < minWeight && !aboveAverage) {
            minWeight = neighbor->getWeight();
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

void deleteNodeFromGraph(Graph* graph, Node* node, unordered_map<int, int>& nodesDegree) {
    graph->deleteNode(node);
    nodesDegree.erase(node->getId());
}

void addNodeToCluster(Graph* cluster, Node* node, Node* parentNode, int& clusterWeight) {
    cout << "Adicionando nó " << node->getId() << " ao cluster com peso " << node->getWeight() << endl;
    cluster->createOrUpdateNode(node->getId(), node->getWeight());
    cluster->createEdge(node, parentNode, 1);
    clusterWeight += node->getWeight();
}

int getSmallerDegreeNodeId(unordered_map<int, int>& nodesDegree, float alpha) {
    unordered_map<int, vector<int>> nodesDegreesMap;

    int smallerDegree = 501;

    for (const auto& node : nodesDegree) {
        int nodeId = node.first;
        int degree = node.second;

        if (node.second < smallerDegree) {
            smallerDegree = degree;
        }

        nodesDegreesMap[degree].push_back(nodeId);
    }

    if (alpha > 0) {
        int listSize = nodesDegreesMap[smallerDegree].size();

        int maxIndex = static_cast<int>(ceil(listSize * alpha));
        int randomIndex = rand() % (maxIndex);
        return nodesDegreesMap[smallerDegree][randomIndex];
    }

    return nodesDegreesMap[smallerDegree][0];
}

MGGPPInfo* getMGGPPByReactiveGRASPAlgorithm(Graph* graph, int numClusters, vector<float> alphas, int maxIterations) {
    MGGPPInfo* bestSolution = nullptr;
    vector<MGGPPAlphaTable> alphaTable(alphas.size());

    for (size_t i = 0; i < alphas.size(); i++) {
        alphaTable[i].alpha = alphas[i];
        alphaTable[i].average = 0;
        alphaTable[i].best = 0;
        alphaTable[i].iterations = 0;
        alphaTable[i].probability = 10.0;
    }

    for (int iter = 0; iter < maxIterations; iter++) {
        float chosenAlpha = getAlphaValue(alphaTable);

        vector<Graph*> clusters = getMGGPPByGreedyAlgorithm(graph, numClusters, chosenAlpha);

        MGGPPInfo* currentSolution = getMGGPPInfo(clusters);

        if (bestSolution == nullptr || currentSolution->gapSum < bestSolution->gapSum) {
            if (bestSolution) {
                delete bestSolution;
            }
            bestSolution = currentSolution;
        } else {
            delete currentSolution;
        }

        for (auto& entry : alphaTable) {
            if (entry.alpha == chosenAlpha) {
                entry.iterations++;
                entry.average = ((entry.average * (entry.iterations - 1)) + currentSolution->gapSum) / entry.iterations;

                if (currentSolution->gapSum < entry.best) {
                    entry.best = currentSolution->gapSum;
                    entry.probability *= 1.2;
                }

                break;
            }
        }
    }

    return bestSolution;
}

float getAlphaValue(vector<MGGPPAlphaTable>& alphaTable) {
    double sumProbabilities = 0;
    for (const auto& entry : alphaTable) {
        sumProbabilities += entry.probability;
    }

    double randomValue = ((double)rand() / RAND_MAX) * sumProbabilities;
    double cumulativeProbability = 0;

    for (const auto& entry : alphaTable) {
        cumulativeProbability += entry.probability;
        if (randomValue <= cumulativeProbability) {
            return entry.alpha;
        }
    }

    return alphaTable.back().alpha;
}

MGGPPInfo* getMGGPPInfo(vector<Graph*> clusters) {
    MGGPPInfo* info = new MGGPPInfo();
    int clusterGap = 0;

    for (Graph* cluster : clusters) {
        MGGPPInfoCluster clusterInfo;
        vector<int> clusterNodes;

        clusterGap = 0;

        Node* currentNode = cluster->getFirstNode();
        while (currentNode != nullptr) {
            clusterGap += currentNode->getWeight();

            clusterNodes.push_back(currentNode->getId());

            currentNode = currentNode->getNextNode();
        }

        clusterInfo.gap = clusterGap;
        clusterInfo.nodes = clusterNodes;

        info->gapSum += clusterGap;
        info->clusters.push_back(clusterInfo);
    }

    return info;
}

void printMGGPPInfo(const MGGPPInfo* info) {
    if (info == nullptr) {
        std::cout << "MGGPPInfo is null." << std::endl;
        return;
    }

    std::cout << "MGGPPInfo: " << std::endl;
    std::cout << "Gap Sum: " << info->gapSum << std::endl;
    std::cout << "Alpha: " << info->alpha << std::endl;

    for (size_t i = 0; i < info->clusters.size(); ++i) {
        const MGGPPInfoCluster& cluster = info->clusters[i];
        std::cout << "Cluster " << i + 1 << ": " << std::endl;
        std::cout << "  Gap: " << cluster.gap << std::endl;
        std::cout << "  Nodes: ";
        for (int nodeId : cluster.nodes) {
            std::cout << nodeId << " ";
        }
        std::cout << std::endl;
    }
}
