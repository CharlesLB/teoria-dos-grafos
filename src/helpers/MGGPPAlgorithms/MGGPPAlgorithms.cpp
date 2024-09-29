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
    int clusterWeight = getClusterWeight(graph, numClusters);
    vector<Graph*> clusters(numClusters);
    vector<pair<int, int>> nodesDegree = getNodeDegreeOrdered(graph);

    while (graph->getFirstNode() != nullptr) {
        Node* currentNode = graph->findNodeById(nodesDegree[0].first);

        // code here
    }

    return clusters;
}

vector<pair<int, int>> getNodeDegreeOrdered(Graph* graph) {
    vector<pair<int, int>> nodesDegree;

    Node* currentNode = graph->getFirstNode();

    while (currentNode != nullptr) {
        nodesDegree.push_back({currentNode->getId(), currentNode->getDegree(false)});
        currentNode = currentNode->getNextNode();
    }

    sort(nodesDegree.begin(), nodesDegree.end(),
         [](const pair<int, int>& a, const pair<int, int>& b) {
             return a.second < b.second;
         });

    return nodesDegree;
}

int getClusterWeight(Graph* graph, int numClusters) {
    int clusterWeight = 0;

    Node* currentNode = graph->getFirstNode();
    for (int i = 0; i < numClusters; i++) {
        clusterWeight += currentNode->getWeight();
        currentNode = currentNode->getNextNode();
    }

    return clusterWeight / numClusters;
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