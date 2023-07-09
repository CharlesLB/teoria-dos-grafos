#include <iostream>
#include <map>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "../../lib/Edge/edge.hpp"
#include "../../lib/Graph/graph.hpp"
#include "../../lib/Node/node.hpp"
#include "../../utils/graph/graph.hpp"

bool checkGraphIsKRegularByK(Graph* graph, int k) {
    Node* currentNode = graph->getFirstNode();
    while (currentNode != nullptr) {
        if (currentNode->getDegree(graph->isDirected()) != k) {
            return false;
        }
        currentNode = currentNode->getNextNode();
    }
    return true;
}

bool checkGraphIsTrivial(Graph* graph) {
    return (graph->getNumNodes() == 1 && graph->getNumEdges() == 0);
}

bool checkGraphIsNull(Graph* graph) {
    return (graph->getNumNodes() == 0 && graph->getNumEdges() == 0);
}

bool checkGraphIsComplete(Graph* graph) {
    int numNodes = graph->getNumNodes();
    int maxEdges = numNodes * (numNodes - 1);
    return (graph->getNumEdges() == maxEdges);
}

bool checkGraphIsBipartite(Graph* graph) {
    std::unordered_map<Node*, int> nodeColors;
    std::queue<Node*> bfsQueue;
    Node* startNode = graph->getFirstNode();
    bfsQueue.push(startNode);
    nodeColors[startNode] = 0;

    while (!bfsQueue.empty()) {
        Node* currentNode = bfsQueue.front();
        bfsQueue.pop();

        int currentColor = nodeColors[currentNode];
        int neighborColor = 1 - currentColor;

        for (Edge* edge : currentNode->getEdges()) {
            Node* neighborNode = (currentNode == edge->getHead()) ? edge->getTail() : edge->getHead();

            if (nodeColors.find(neighborNode) == nodeColors.end()) {
                nodeColors[neighborNode] = neighborColor;
                bfsQueue.push(neighborNode);
            } else if (nodeColors[neighborNode] != neighborColor) {
                return false;
            }
        }
    }

    std::unordered_set<int> colors;

    for (const auto& entry : nodeColors) {
        colors.insert(entry.second);
        if (colors.size() > 2) {
            return false;
        }
    }

    return true;
}

bool checkGraphIsEulerian(Graph* graph) {
    if (graph->isDirected()) {
        Node* currentNode = graph->getFirstNode();
        while (currentNode != nullptr) {
            if (currentNode->getDegreeIn() != currentNode->getDegreeOut()) {
                return false;
            }
            currentNode = currentNode->getNextNode();
        }
    } else {
        Node* currentNode = graph->getFirstNode();
        while (currentNode != nullptr) {
            if (currentNode->getDegree(graph->isDirected()) % 2 != 0) {
                return false;
            }
            currentNode = currentNode->getNextNode();
        }
    }

    return true;
}

struct checkGraphIsMultigraphPairHash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& pair) const {
        auto hash1 = std::hash<T1>{}(pair.first);
        auto hash2 = std::hash<T2>{}(pair.second);
        return hash1 ^ hash2;
    }
};

bool checkGraphIsMultigraph(Graph* graph) {
    std::unordered_map<std::pair<int, int>, int, checkGraphIsMultigraphPairHash> edgeCounts;

    for (Edge* edge : graph->getEdges()) {
        int source = edge->getHead()->getId();
        int destination = edge->getTail()->getId();
        std::pair<int, int> edgePair = std::make_pair(source, destination);
        edgeCounts[edgePair]++;
    }

    for (auto& pair : edgeCounts) {
        if (pair.second > 1) {
            return true;
        }
    }

    return false;
}