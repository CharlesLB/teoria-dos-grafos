#include <algorithm>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "../../lib/Edge/edge.hpp"
#include "../../lib/Graph/graph.hpp"
#include "../../lib/Node/node.hpp"
#include "../../utils/graph/graph.hpp"

vector<Node*> getOpenNeighborhoodNodesByNode(Node* node) {
    vector<Node*> openNeighborhoodNodes;
    unordered_set<Node*> visitedNodes;

    if (node == nullptr) {
        return openNeighborhoodNodes;
    }

    Edge* edge = node->getFirstEdge();

    while (edge != nullptr) {
        Node* neighborNode = edge->getTail();

        if (neighborNode != nullptr && visitedNodes.find(neighborNode) == visitedNodes.end()) {
            openNeighborhoodNodes.push_back(neighborNode);
            visitedNodes.insert(neighborNode);
        }

        edge = edge->getNextEdge();
    }

    return openNeighborhoodNodes;
}

vector<Node*> getClosedNeighborhoodNodesByNode(Node* node) {
    vector<Node*> openNeighborhoodNodes = getOpenNeighborhoodNodesByNode(node);
    vector<Node*> closedNeighborhoodNodes;

    closedNeighborhoodNodes.push_back(node);

    for (Node* openNeighborhoodNode : openNeighborhoodNodes) {
        closedNeighborhoodNodes.push_back(openNeighborhoodNode);
    }

    return closedNeighborhoodNodes;
}

vector<Node*> getArticulationNodesInGraph(Graph* graph) {
    vector<Node*> articulationNodes;
    return articulationNodes;
}

vector<Edge*> getBridgeEdgesInGraph(Graph* graph) {
    vector<Edge*> bridgeEdges;
    return bridgeEdges;
}

struct NodeDistance {
    Node* node;
    int distance;

    NodeDistance(Node* n, int d) : node(n), distance(d) {}

    bool operator>(const NodeDistance& other) const {
        return distance > other.distance;
    }
};

Graph* getMinimumPathAndCostByDijkstra(Graph* graph, Node* sourceNode, Node* targetNode) {
    if (graph == nullptr || sourceNode == nullptr || targetNode == nullptr) {
        return nullptr;
    }

    std::unordered_map<Node*, int> distances;
    std::unordered_map<Node*, Node*> previousNodes;
    std::priority_queue<std::pair<int, Node*>, std::vector<std::pair<int, Node*>>, std::greater<>> priorityQueue;

    for (Edge* edge : graph->getEdges()) {
        distances[edge->getHead()] = std::numeric_limits<int>::max();
        distances[edge->getTail()] = std::numeric_limits<int>::max();
    }
    distances[sourceNode] = 0;

    priorityQueue.push({0, sourceNode});

    while (!priorityQueue.empty()) {
        Node* currentNode = priorityQueue.top().second;
        priorityQueue.pop();

        if (currentNode == targetNode) {
            break;
        }

        for (Edge* edge = currentNode->getFirstEdge(); edge != nullptr; edge = edge->getNextEdge()) {
            Node* neighborNode = edge->getTail();
            int newDist = distances[currentNode] + edge->getWeight();

            if (newDist < distances[neighborNode]) {
                distances[neighborNode] = newDist;
                previousNodes[neighborNode] = currentNode;
                priorityQueue.push({newDist, neighborNode});
            }
        }
    }

    Graph* minimumPathGraph = new Graph(graph->isDirected(), graph->isWeightedEdges(), graph->isWeightedNodes());
    Node* currentNode = targetNode;

    while (currentNode != nullptr) {
        Node* previousNode = previousNodes[currentNode];
        if (previousNode != nullptr) {
            Edge* edge = graph->findEdgeByNodes(previousNode, currentNode);
            minimumPathGraph->createEdge(minimumPathGraph->createOrUpdateNode(previousNode->getId(), previousNode->getWeight()),
                                         minimumPathGraph->createOrUpdateNode(currentNode->getId(), currentNode->getWeight()),
                                         edge->getWeight());
        }
        currentNode = previousNode;
    }

    return minimumPathGraph;
}

Graph* getMinimumPathAndCostByFloyd(Graph* graph, Node* sourceNode, Node* targetNode) {
    int numNodes = graph->getNumNodes();

    if (numNodes == 0) {
        return nullptr;
    }

    std::vector<std::vector<int>> dist(numNodes, std::vector<int>(numNodes, std::numeric_limits<int>::max()));
    std::vector<std::vector<Node*>> next(numNodes, std::vector<Node*>(numNodes, nullptr));

    for (Edge* edge : graph->getEdges()) {
        if (edge == nullptr || edge->getHead() == nullptr || edge->getTail() == nullptr) {
            continue;
        }
        int u = edge->getHead()->getId();
        int v = edge->getTail()->getId();

        if (u < 0 || u >= numNodes || v < 0 || v >= numNodes) {
            continue;
        }

        int weight = edge->getWeight();

        dist[u][v] = weight;
        next[u][v] = edge->getTail();

        if (!graph->isDirected()) {
            dist[v][u] = weight;
            next[v][u] = edge->getHead();
        }
    }

    for (int i = 0; i < numNodes; ++i) {
        dist[i][i] = 0;
        next[i][i] = graph->findNodeById(i);
    }

    for (int k = 0; k < numNodes; ++k) {
        for (int i = 0; i < numNodes; ++i) {
            for (int j = 0; j < numNodes; ++j) {
                if (dist[i][k] != std::numeric_limits<int>::max() && dist[k][j] != std::numeric_limits<int>::max()) {
                    if (dist[i][j] > dist[i][k] + dist[k][j]) {
                        dist[i][j] = dist[i][k] + dist[k][j];
                        next[i][j] = next[i][k];
                    }
                }
            }
        }
    }

    int sourceId = sourceNode->getId();
    int targetId = targetNode->getId();

    if (sourceId < 0 || sourceId >= numNodes || targetId < 0 || targetId >= numNodes) {
        return nullptr;
    }

    if (next[sourceId][targetId] == nullptr) {
        return nullptr;
    }

    Graph* minimumPathGraph = new Graph(graph->isDirected(), graph->isWeightedEdges(), graph->isWeightedNodes());
    Node* currentNode = sourceNode;

    while (currentNode != targetNode) {
        Node* nextNode = next[currentNode->getId()][targetNode->getId()];
        Edge* edge = graph->findEdgeByNodes(currentNode, nextNode);
        minimumPathGraph->createEdge(
            minimumPathGraph->createOrUpdateNode(currentNode->getId(), currentNode->getWeight()),
            minimumPathGraph->createOrUpdateNode(nextNode->getId(), nextNode->getWeight()),
            edge->getWeight());
        currentNode = nextNode;
    }

    return minimumPathGraph;
}

Graph* getComplementGraph(Graph* graph) {
    if (graph == nullptr || graph->getNumNodes() == 0) {
        std::cout << "Grafo inválido ou vazio." << std::endl;
        return nullptr;
    }

    Graph* complementGraph = new Graph(graph->isDirected(), graph->isWeightedEdges(), graph->isWeightedNodes());

    Node* currentNode = graph->getFirstNode();
    while (currentNode != nullptr) {
        complementGraph->createOrUpdateNode(currentNode->getId(), currentNode->getWeight());
        currentNode = currentNode->getNextNode();
    }

    Node* nodeU = graph->getFirstNode();
    while (nodeU != nullptr) {
        Node* nodeV = nodeU->getNextNode();
        while (nodeV != nullptr) {
            if (graph->findEdgeByNodes(nodeU, nodeV) == nullptr) {
                complementGraph->createEdge(
                    complementGraph->findNodeById(nodeU->getId()),
                    complementGraph->findNodeById(nodeV->getId()),
                    1);
                if (!graph->isDirected()) {
                    complementGraph->createEdge(
                        complementGraph->findNodeById(nodeV->getId()),
                        complementGraph->findNodeById(nodeU->getId()),
                        1);
                }
            }
            nodeV = nodeV->getNextNode();
        }
        nodeU = nodeU->getNextNode();
    }

    return complementGraph;
};

void depthFirstSearch(Node* node, unordered_set<Node*>& visited) {
    if (visited.find(node) != visited.end()) {
        return;
    }

    visited.insert(node);

    for (Edge* edge = node->getFirstEdge(); edge != nullptr; edge = edge->getNextEdge()) {
        Node* neighbor = (node == edge->getHead()) ? edge->getTail() : edge->getHead();
        depthFirstSearch(neighbor, visited);
    }
}

vector<Node*> getDirectTransitiveClosureByNode(Node* node) {
    vector<Node*> directTransitiveClosure;
    unordered_set<Node*> visited;

    depthFirstSearch(node, visited);

    for (Node* visitedNode : visited) {
        directTransitiveClosure.push_back(visitedNode);
    }

    return directTransitiveClosure;
}

vector<Node*> getIndirectTransitiveClosureByNode(Graph* graph, Node* node) {
    vector<Node*> indirectTransitiveClosure;
    Node* firstNode = graph->getFirstNode();

    if (firstNode == nullptr) {
        return indirectTransitiveClosure;
    }

    for (Node* currentNode = firstNode; currentNode != nullptr; currentNode = currentNode->getNextNode()) {
        if (currentNode == node) {
            continue;
        }

        vector<Node*> directTransitiveClosure = getDirectTransitiveClosureByNode(currentNode);

        if (find(directTransitiveClosure.begin(), directTransitiveClosure.end(), node) != directTransitiveClosure.end()) {
            indirectTransitiveClosure.push_back(currentNode);
        }
    }

    return indirectTransitiveClosure;
}

Graph* createInducedSubgraph(Graph* graph, const vector<Node*>& selectedNodes) {
    Graph* subgraph = new Graph(graph->isDirected(), graph->isWeightedEdges(), graph->isWeightedNodes());

    for (Node* node : selectedNodes) {
        subgraph->createOrUpdateNode(node->getId(), node->getWeight());
    }

    for (Node* node : selectedNodes) {
        for (Edge* edge : node->getEdges()) {
            Node* head = edge->getHead();
            Node* tail = edge->getTail();

            if (find(selectedNodes.begin(), selectedNodes.end(), head) != selectedNodes.end() &&
                find(selectedNodes.begin(), selectedNodes.end(), tail) != selectedNodes.end()) {
                subgraph->createEdge(
                    subgraph->createOrUpdateNode(head->getId(), head->getWeight()),
                    subgraph->createOrUpdateNode(tail->getId(), tail->getWeight()),
                    edge->getWeight());
            }
        }
    }

    return subgraph;
}