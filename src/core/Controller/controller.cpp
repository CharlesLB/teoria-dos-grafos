#include "./controller.hpp"

#include <dirent.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <sstream>

#include "../../helpers/Algorithms/algorithms.hpp"
#include "../../helpers/MGGPPAlgorithms/MGGPPAlgorithms.hpp"
#include "../../helpers/Validators/validators.hpp"
#include "../../lib/Edge/edge.hpp"
#include "../../lib/Graph/graph.hpp"
#include "../../lib/Node/node.hpp"
#include "../Manager/manager.hpp"
#include "../Reader/reader.hpp"
#include "../Writer/writer.hpp"

using namespace std;

void Controller::processOperation(char* argv[], Graph* graph) {
    char option;

    string options[] = {
        "Impressão do grafo em um arquivo",
        "[a] Obter fecho transitivo direto de um nó",
        "[b] Obter fecho transitivo indireto de um nó",
        "[c] e [d] Caminho mínimo e custo entre dois vértices",
        "[e] Árvore geradora mínima com Prim",
        "[f] Árvore geradora mínima com Kruskal",
        "[g] Árvore de profundidade",
        "[h] Obter raio, diâmetro, centro e periferia do grafo",
        "[i] Obter o conjunto de nós de articulação",
        "Exit", ""};

    while (true) {
        Writer::printMenu(options);
        option = Reader::readChar();
        bool invalidOption = false;

        switch (option) {
            case 'a':
                printGraphToFile(graph, argv[2]);
                break;

            case 'b':
                getDirectTransitiveClosure(graph);
                break;

            case 'c':
                getIndirectTransitiveClosure(graph);
                break;

            case 'd':
                getMinimumPathAndCost(graph);
                break;

            case 'e':
                getMinimumSpanningTreeKruscal(graph);
                break;

            case 'f':
                getMinimumSpanningTreePrim(graph);
                break;

            case 'g':
                getDepthFirstSearchTree(graph);
                break;

            case 'h':
                getGraphMetrics(graph);
                break;

            case 'i':
                getArticulationNodes(graph);
                break;

            case 'j':
                exitSystem();
                break;

            default:
                invalidOption = true;
                cout << "Invalid option\n";
                break;
        }

        if (invalidOption) continue;
        Reader::continueConfirmation();
    }
}

void Controller::processOperationAMPL(char* argv[], Graph* graph, int numClusters) {
    char option;

    string options[] = {
        "Impressão do grafo em um arquivo",
        "MGGPP com algoritmo guloso",
        "MGGPP com algoritmo guloso randomizado adaptativo",
        "MGGPP com algoritmo guloso randomizado adaptativo reativo",
        "Exit", ""};

    while (true) {
        Writer::printMenu(options);
        option = Reader::readChar();
        bool invalidOption = false;

        switch (option) {
            case 'a':
                printGraphToFile(graph, argv[2]);
                break;

            case 'b':
                runGreedyAlgorithm(graph, numClusters);
                break;

            case 'c':
                runGRASPAlgorithm(graph, numClusters);
                break;

            case 'd':
                runReactiveGRASPAlgorithm(graph, numClusters);
                break;

            case 'e':
                exitSystem();
                break;

            default:
                invalidOption = true;
                cout << "Invalid option\n";
                break;
        }

        if (invalidOption) continue;
        Reader::continueConfirmation();
    }
}

void Controller::exitSystem() {
    cout << "Good Bye!\n";
    exit(0);
}

void Controller::printGraphToFile(Graph* graph, string outputPath) {
    Writer::printGraphOptions(graph, outputPath);
}

void Controller::nodeAndEdgeInsertionDeletion(Graph* graph) {
    Manager::processOperation(graph);
}

void Controller::getNodeDegree(Graph* graph) {
    Manager::getNodeDegree(graph);
}

void Controller::checkGraphRegular(Graph* graph) {
    cout << "Enter the degree to check if the graph is regular: ";
    int degree = Reader::readInt();

    bool isRegular = checkGraphIsKRegularByK(graph, degree);

    if (isRegular) {
        cout << "The graph is " << degree << "-regular\n";
        return;
    }

    cout << "The graph is not " << degree << "-regular\n";
}

void Controller::getGraphOrder(Graph* graph) {
    cout << "The graph order is " << graph->getNumNodes() << "\n";
}

void Controller::isGraphTrivial(Graph* graph) {
    bool isTrivial = checkGraphIsTrivial(graph);

    if (isTrivial) {
        cout << "The graph is trivial\n";
        return;
    }

    cout << "The graph is not trivial\n";
}

void Controller::isGraphNull(Graph* graph) {
    bool isNull = checkGraphIsNull(graph);

    if (isNull) {
        cout << "The graph is null\n";
        return;
    }

    cout << "The graph is not null\n";
}

void Controller::showOpenNeighborhood(Graph* graph) {
    Node* node = Manager::selectNode(graph);
    vector<Node*> openNeightborhoodNodes = getOpenNeighborhoodNodesByNode(node);

    cout << "The open neighborhood of node is " << node->getId();

    if (openNeightborhoodNodes.size() == 0) {
        cout << "empty\n";
        return;
    }

    Writer::printVectorNodes(openNeightborhoodNodes);
}

void Controller::showClosedNeighborhood(Graph* graph) {
    Node* node = Manager::selectNode(graph);
    vector<Node*> closeNeightborhoodNodes = getClosedNeighborhoodNodesByNode(node);

    cout << "The closed neighborhood of node is " << node->getId();

    if (closeNeightborhoodNodes.size() == 0) {
        cout << "empty\n";
        return;
    }

    Writer::printVectorNodes(closeNeightborhoodNodes);
}

void Controller::checkMultigraph(Graph* graph) {
    bool isMultigraph = checkGraphIsMultigraph(graph);

    if (isMultigraph) {
        cout << "The graph is multigraph\n";
        return;
    }

    cout << "The graph is not multigraph\n";
}

void Controller::checkCompleteGraph(Graph* graph) {
    bool isComplete = checkGraphIsComplete(graph);

    if (isComplete) {
        cout << "The graph is complete\n";
        return;
    }

    cout << "The graph is not complete\n";
}

void Controller::checkBipartiteGraph(Graph* graph) {
    bool isBipartite = checkGraphIsBipartite(graph);

    if (isBipartite) {
        cout << "The graph is bipartite\n";
        return;
    }

    cout << "The graph is not bipartite\n";
}

// Exercício C e D
void Controller::getMinimumPathAndCost(Graph* graph) {
    cout << "Select the start node\n";
    Node* startNode = Manager::selectNode(graph);

    cout << "Select the end node\n";
    Node* endNode = Manager::selectNode(graph);

    char option;

    string options[] = {"Dijkstra's algorithm", "Floyd's algorithm", "Exit", ""};

    Graph* minimumPathGraph;

    Writer::printMenu(options);
    option = Reader::readChar();

    switch (option) {
        case 'a':
            cout << "Dijkstra's algorithm\n";
            minimumPathGraph = getMinimumPathAndCostByDijkstra(graph, startNode, endNode);
            cout << "End of Dijkstra's algorithm\n";
            break;

        case 'b':
            cout << "Floyd's algorithm\n";
            minimumPathGraph = getMinimumPathAndCostByFloyd(graph, startNode, endNode);
            cout << "End of Floyd's algorithm\n";
            break;

        case 'c':
            return;

        default:
            cout << "Invalid option\n";
            break;
    }

    Writer::printGraphOptions(minimumPathGraph, "minimumPathGraph");
}

void Controller::getGraphDegree(Graph* graph) {
    cout << "The graph degree is " << graph->getDegree() << "\n";
}

// Exercício A
void Controller::getDirectTransitiveClosure(Graph* graph) {
    Node* node = Manager::selectNode(graph);

    vector<Node*> directTransitiveClosureGraph = getDirectTransitiveClosureByNode(node);

    cout << "The direct transitive closure of node " << node->getId() << " is ";

    if (directTransitiveClosureGraph.size() == 0) {
        cout << "empty\n";
        return;
    }

    Writer::printVectorNodes(directTransitiveClosureGraph);
}

// Exercício B
void Controller::getIndirectTransitiveClosure(Graph* graph) {
    Node* node = Manager::selectNode(graph);

    vector<Node*> indirectTransitiveClosureGraph = getIndirectTransitiveClosureByNode(graph, node);

    cout << "The indirect transitive closure of node " << node->getId() << " is ";

    if (indirectTransitiveClosureGraph.size() == 0) {
        cout << "empty\n";
        return;
    }

    Writer::printVectorNodes(indirectTransitiveClosureGraph);
}

void Controller::getDegreeSequence(Graph* graph) {
    if (graph == nullptr || graph->getNumNodes() == 0) {
        cout << "Grafo inválido ou vazio." << endl;
        return;
    }

    vector<int> degrees;

    Node* currentNode = graph->getFirstNode();
    while (currentNode != nullptr) {
        int degree = currentNode->getDegree(graph->isDirected());
        degrees.push_back(degree);
        currentNode = currentNode->getNextNode();
    }

    sort(degrees.begin(), degrees.end(), greater<int>());

    cout << "Sequência de Graus: ";
    for (int degree : degrees) {
        cout << degree << " ";
    }
    cout << endl;
}

void Controller::getInducedSubgraph(Graph* graph) {
    cout << "Select the nodes to create the induced subgraph\n";
    vector<Node*> selectedNodes = Manager::selectNodes(graph);

    Graph* subgraph = createInducedSubgraph(graph, selectedNodes);

    if (subgraph == nullptr) {
        cout << "Failed to create the induced subgraph.\n";
        return;
    }

    cout << "Subgraph induced by the selected nodes has been created.\n";

    Writer::printGraphOptions(subgraph, "subgraph");
}

void Controller::getGraphComplement(Graph* graph) {
    Graph* complementGraph = getComplementGraph(graph);
    Writer::printGraphOptions(complementGraph, "complementGraph");
}

void Controller::getStronglyConnectedComponents(Graph* graph) {
    printStronglyConnectedComponents(graph);
}

void Controller::checkEulerianGraph(Graph* graph) {
    bool isEulerian = checkGraphIsEulerian(graph);

    if (isEulerian) {
        cout << "The graph is eulerian\n";
        return;
    }

    cout << "The graph is not eulerian\n";
}

// Exercício I
void Controller::getArticulationNodes(Graph* graph) {
    vector<Node*> articulationNodes = getArticulationNodesInGraph(graph);

    if (articulationNodes.size() == 0) {
        cout << "There are no articulation nodes in the graph\n";
        return;
    }

    cout << "The articulation nodes in the graph are:";

    Writer::printVectorNodes(articulationNodes);
}

void Controller::getBridgeEdges(Graph* graph) {
    vector<Edge*> bridgeEdges = getBridgeEdgesInGraph(graph);

    if (bridgeEdges.size() == 0) {
        cout << "There are no bridge edges in the graph\n";
        return;
    }

    cout << "The bridge edges in the graph are:";

    Writer::printVectorEdges(bridgeEdges);
}

// Exercício H
void Controller::getGraphMetrics(Graph* graph) {
    GraphMetrics metrics = getGraphMetricsInGraph(graph);

    cout << "Diâmetro do grafo: " << metrics.diameter << endl;
    cout << "Raio do grafo: " << metrics.radius << endl;

    cout << "Centro do grafo: ";
    for (int node : metrics.center) {
        cout << node << " ";
    }
    cout << endl;

    cout << "Periferia do grafo: ";
    for (int node : metrics.periphery) {
        cout << node << " ";
    }
    cout << endl;

    delete graph;
    return;
}

void Controller::getMinimumSpanningTreeKruscal(Graph* graph) {
    vector<Node*> selectedNodes = Manager::selectNodes(graph);

    Graph* minimumSpanningTree = getMinimumSpanningTreeByKruskal(graph, selectedNodes);
    Writer::printGraphOptions(minimumSpanningTree, "minimumSpanningTreeKruskal");
}

void Controller::getMinimumSpanningTreePrim(Graph* graph) {
    vector<Node*> selectedNodes = Manager::selectNodes(graph);

    Graph* minimumSpanningTree = getMinimumSpanningTreeByPrim(graph, selectedNodes);
    Writer::printGraphOptions(minimumSpanningTree, "minimumSpanningTreePrim");
}

void Controller::getDepthFirstSearchTree(Graph* graph) {
    cout << "Select the start node\n";
    Node* startNode = Manager::selectNode(graph);

    printDepthFirstSearchTree(graph, startNode);
}

void Controller::runGreedyAlgorithm(Graph* graph, int numClusters) {
    vector<Graph*> MGGPPGraph = getMGGPPByGreedyAlgorithm(graph, numClusters);

    Writer::printGraphInDotFile(mergeGraphs(MGGPPGraph), "MGGPPGraph");

    for (int i = 0; i < MGGPPGraph.size(); i++) {
        Writer::printGraphInDotFile(MGGPPGraph[i], "MGGPPGraph" + to_string(i));
    }
}

void Controller::runGRASPAlgorithm(Graph* graph, int numClusters) {
    Graph* MGGPPGraph = getMGGPPByGRASPAlgorithm(graph, numClusters);
}

void Controller::runReactiveGRASPAlgorithm(Graph* graph, int numClusters) {
    Graph* MGGPPGraph = getMGGPPByReactiveGRASPAlgorithm(graph, numClusters);
}