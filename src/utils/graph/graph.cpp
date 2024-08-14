#include "../../lib/Graph/graph.hpp"

#include <iostream>
#include <unordered_map>
#include <unordered_set>

#include "../../lib/Edge/edge.hpp"
#include "../../lib/Node/node.hpp"

Graph* createGraphCopy(Graph* graph) {
    Graph* copiedGraph = new Graph(graph->isDirected(), graph->isWeightedEdges(), graph->isWeightedNodes());

    unordered_map<Node*, Node*> nodeMap;

    Node* originalNode = graph->getFirstNode();
    while (originalNode != nullptr) {
        int id = originalNode->getId();
        int weight = originalNode->getWeight();

        Node* copiedNode = copiedGraph->createOrUpdateNode(id, weight);

        nodeMap[originalNode] = copiedNode;

        originalNode = originalNode->getNextNode();
    }

    for (Edge* originalEdge : graph->getEdges()) {
        Node* originalHead = originalEdge->getHead();
        Node* originalTail = originalEdge->getTail();

        Node* copiedHead = nodeMap[originalHead];
        Node* copiedTail = nodeMap[originalTail];

        int weight = originalEdge->getWeight();

        copiedGraph->createEdge(copiedHead, copiedTail, weight);
    }

    return copiedGraph;
}

struct convertMultigraphToGraphPairHash {
    template <class T1, class T2>
    size_t operator()(const pair<T1, T2>& pair) const {
        auto hash1 = hash<T1>{}(pair.first);
        auto hash2 = hash<T2>{}(pair.second);
        return hash1 ^ hash2;
    }
};

Graph* convertMultigraphToGraph(Graph* multigraph) {
    Graph* graph = new Graph(multigraph->isDirected(), multigraph->isWeightedEdges(), multigraph->isWeightedNodes());

    unordered_set<pair<Node*, Node*>, convertMultigraphToGraphPairHash> uniqueEdges;

    for (Edge* edge : multigraph->getEdges()) {
        Node* head = edge->getHead();
        Node* tail = edge->getTail();

        if (uniqueEdges.find(make_pair(head, tail)) == uniqueEdges.end()) {
            graph->createEdge(head, tail, edge->getWeight());

            uniqueEdges.insert(make_pair(head, tail));
        }
    }

    return graph;
}
