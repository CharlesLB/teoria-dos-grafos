#include "./controller.hpp"

#include <dirent.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <sstream>

#include "../../helpers/Algorithms/algorithms.hpp"
#include "../../helpers/Validators/validators.hpp"
#include "../../lib/Edge/edge.hpp"
#include "../../lib/Graph/graph.hpp"
#include "../../lib/Node/node.hpp"
#include "../Manager/manager.hpp"
#include "../Reader/reader.hpp"
#include "../Writer/writer.hpp"

using namespace std;

void Controller::processOperation(char* argv[], bool hasWeightedNode, bool hasWeightedEdge, bool isDirected, Graph* graph) {
    char option;

    string options[] = {
        "Impressão do grafo em um arquivo",
        "Inclusão e exclusão de nó e de aresta",
        "Retornar o grau de um dado nó informado pelo usuário (caso seja um dígrafo, informar grau de entrada e de saída)",
        "Verificar a k-regularidade do grafo (k informado pelo usuário)",
        "Informar a ordem do grafo",
        "Informar se o grafo é trivial",
        "Informar se o grafo é nulo",
        "Mostrar a vizinhança aberta de um dado nó informado pelo usuário",
        "Mostrar a vizinhança fechada de um dado nó informado pelo usuário",
        "Verificar se o grafo é um multigrafo",
        "Verificar se o grafo é completo",
        "Verificar se o grafo é bipartido",
        "Dados dois vértices informados pelo usuário, mostrar o caminho mínimo e seu custo entre esses vértices (utilizar o algoritmo de Dijkstra ou de Floyd, conforme o usuário solicite)",
        "Retornar o grau do grafo",
        "Retornar o fecho transitivo direto de um dado nó informado pelo usuário",
        "Retornar o fecho transitivo indireto de um dado nó informado pelo usuário",
        "Apresentar a sequência de graus",
        "Apresentar o subgrafo induzido por um dado conjunto de vértices informado pelo usuário",
        "Apresentar o complementar do grafo",
        "Para digrafos, apresentar as componentes fortemente conexas",
        "Verificar se o grafo é eulerianos",
        "Apresentar os nós de articulação",
        "Apresentar as arestas ponte",
        "Apresentar o raio, o diâmetro, o centro e a periferia do grafo",
        "Apresentar a AGM do grafo ou, para grafos desconexos, as florestas de custo mínimo com Kruskal",
        "Apresentar a AGM do grafo ou, para grafos desconexos, as florestas de custo mínimo com Prim",
        "Exit", ""};

    while (true) {
        Writer::printMenu(options);
        option = Reader::readChar();
        bool invalidOption = false;

        switch (option) {
            case '0':
                exitSystem();
                break;
            case 'a':
                printGraphToFile(graph, argv[2]);
                break;

            case 'b':
                nodeAndEdgeInsertionDeletion(graph);
                break;

            case 'c':
                getNodeDegree(graph);
                break;

            case 'd':
                checkGraphRegular(graph);
                break;

            case 'e':
                getGraphOrder(graph);
                break;

            case 'f':
                isGraphTrivial(graph);
                break;

            case 'g':
                isGraphNull(graph);
                break;

            case 'h':
                showOpenNeighborhood(graph);
                break;

            case 'i':
                showClosedNeighborhood(graph);
                break;

            case 'j':
                checkMultigraph(graph);
                break;

            case 'k':
                checkCompleteGraph(graph);
                break;

            case 'l':
                checkBipartiteGraph(graph);
                break;

            case 'm':
                getMinimumPathAndCost(graph);
                break;

            case 'n':
                getGraphDegree(graph);
                break;

            case 'o':
                getDirectTransitiveClosure(graph);
                break;

            case 'p':
                getIndirectTransitiveClosure(graph);
                break;

            case 'q':
                getDegreeSequence(graph);
                break;

            case 'r':
                getInducedSubgraph(graph);
                break;

            case 's':
                getGraphComplement(graph);
                break;

            case 't':
                getStronglyConnectedComponents(graph);
                break;

            case 'u':
                checkEulerianGraph(graph);
                break;

            case 'v':
                getArticulationNodes(graph);
                break;

            case 'w':
                getBridgeEdges(graph);
                break;

            case 'x':
                getGraphMetrics(graph);
                break;

            case 'y':
                getMinimumSpanningTreeKruscal(graph);
                break;

            case 'z':
                getMinimumSpanningTreePrim(graph);
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

    std::cout << "Diâmetro do grafo: " << metrics.diameter << std::endl;
    std::cout << "Raio do grafo: " << metrics.radius << std::endl;

    std::cout << "Centro do grafo: ";
    for (int node : metrics.center) {
        std::cout << node << " ";
    }
    std::cout << std::endl;

    std::cout << "Periferia do grafo: ";
    for (int node : metrics.periphery) {
        std::cout << node << " ";
    }
    std::cout << std::endl;

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
