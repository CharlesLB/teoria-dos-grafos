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

    unordered_map<Node*, int> distances;
    for (Node* node = graph->getFirstNode(); node != nullptr; node = node->getNextNode()) {
        distances[node] = (node == sourceNode) ? 0 : numeric_limits<int>::max();
    }

    priority_queue<NodeDistance, vector<NodeDistance>, greater<NodeDistance>> pq;
    pq.push(NodeDistance(sourceNode, 0));

    while (!pq.empty()) {
        Node* currentNode = pq.top().node;
        int currentDistance = pq.top().distance;
        pq.pop();

        if (currentNode == targetNode) {
            break;
        }

        for (Edge* edge = currentNode->getFirstEdge(); edge != nullptr; edge = edge->getNextEdge()) {
            Node* neighbor = (currentNode == edge->getHead()) ? edge->getTail() : edge->getHead();
            int weight = edge->getWeight();

            if (distances[currentNode] + weight < distances[neighbor]) {
                distances[neighbor] = distances[currentNode] + weight;
                pq.push(NodeDistance(neighbor, distances[neighbor]));
            }
        }
    }

    Graph* shortestPathGraph = new Graph(graph->isDirected(), graph->isWeightedEdges(), graph->isWeightedNodes());

    Node* currentNode = targetNode;
    while (currentNode != sourceNode) {
        int shortestDistance = distances[currentNode];

        Edge* shortestEdge = nullptr;
        for (Edge* edge = currentNode->getFirstEdge(); edge != nullptr; edge = edge->getNextEdge()) {
            Node* neighbor = (currentNode == edge->getHead()) ? edge->getTail() : edge->getHead();
            int weight = edge->getWeight();

            if (distances[neighbor] + weight == shortestDistance) {
                shortestEdge = edge;
                break;
            }
        }

        if (shortestEdge) {
            Node* neighborNode = (currentNode == shortestEdge->getHead()) ? shortestEdge->getTail() : shortestEdge->getHead();
            shortestPathGraph->createOrUpdateNode(currentNode->getId(), currentNode->getWeight());
            shortestPathGraph->createOrUpdateNode(neighborNode->getId(), neighborNode->getWeight());
            shortestPathGraph->createEdge(currentNode, neighborNode, shortestEdge->getWeight());
            currentNode = neighborNode;
            continue;
        }

        break;
    }

    return shortestPathGraph;
}

Graph* getMinimumPathAndCostByFloyd(Graph* graph, Node* sourceNode, Node* targetNode) {
    if (graph == nullptr || sourceNode == nullptr || targetNode == nullptr) {
        return nullptr;
    }

    // Step 1: Initialize the distance matrix.
    int numNodes = graph->getNumNodes();
    vector<vector<int>> distance(numNodes, vector<int>(numNodes, numeric_limits<int>::max()));

    for (Node* node = graph->getFirstNode(); node != nullptr; node = node->getNextNode()) {
        distance[node->getId()][node->getId()] = 0;
    }

    for (Edge* edge : graph->getEdges()) {
        Node* head = edge->getHead();
        Node* tail = edge->getTail();
        int weight = edge->getWeight();
        distance[head->getId()][tail->getId()] = weight;
    }

    // Step 2: Calculate the shortest paths between all pairs of nodes.
    for (int k = 0; k < numNodes; ++k) {
        for (int i = 0; i < numNodes; ++i) {
            for (int j = 0; j < numNodes; ++j) {
                if (distance[i][k] != numeric_limits<int>::max() &&
                    distance[k][j] != numeric_limits<int>::max() &&
                    distance[i][k] + distance[k][j] < distance[i][j]) {
                    distance[i][j] = distance[i][k] + distance[k][j];
                }
            }
        }
    }

    // Step 3: Extract the shortest path from sourceNode to targetNode.
    int shortestDistance = distance[sourceNode->getId()][targetNode->getId()];
    if (shortestDistance == numeric_limits<int>::max()) {
        return nullptr;
    }

    Graph* shortestPathGraph = new Graph(graph->isDirected(), graph->isWeightedEdges(), graph->isWeightedNodes());

    Node* currentNode = targetNode;
    while (currentNode != sourceNode) {
        for (Node* node = graph->getFirstNode(); node != nullptr; node = node->getNextNode()) {
            int weight = distance[sourceNode->getId()][node->getId()] +
                         distance[node->getId()][targetNode->getId()];

            if (weight == shortestDistance) {
                shortestPathGraph->createOrUpdateNode(node->getId(), node->getWeight());
                shortestDistance -= distance[node->getId()][targetNode->getId()];
                currentNode = node;
                break;
            }
        }
    }

    return shortestPathGraph;
}

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