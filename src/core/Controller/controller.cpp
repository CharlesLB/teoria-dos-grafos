#include "./controller.hpp"

#include <dirent.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
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
        "Apresentar a AGM do grafo ou, para grafos desconexos, as florestas de custo mínimo",
        "Apresentar o caminho mínimo entre dois vértices usando o algoritmo de Dijkstra ou de Floyd (escolha do usuário)", "Exit", ""};

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
                getMinimumSpanningTree(graph);
                break;

            case 'z':
                getMinimumPath(graph);
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
    char option;

    string options[] = {"Imprimir em arquivo .dot", "Imprimir em arquivo .txt", "Exit", ""};

    while (true) {
        Writer::printMenu(options);
        option = Reader::readChar();

        switch (option) {
            case 'a':
                Writer::printGraphInDotFile(graph, outputPath);
                break;

            case 'b':
                Writer::printGraphInTxtFile(graph, outputPath);
                break;

            case 'c':
                return;

            default:
                cout << "Invalid option\n";
                break;
        }
    }
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

    cout << "The open neighborhood of node " << node->getId() << " is: {";
    for (int i = 0; i < openNeightborhoodNodes.size(); i++) {
        if (i == openNeightborhoodNodes.size() - 1) {
            cout << openNeightborhoodNodes[i]->getId() << "}\n";
            break;
        }

        cout << openNeightborhoodNodes[i]->getId() << ", ";
    }
}

void Controller::showClosedNeighborhood(Graph* graph) {
    Node* node = Manager::selectNode(graph);
    vector<Node*> closeNeightborhoodNodes = getClosedNeighborhoodNodesByNode(node);

    cout << "The open neighborhood of node " << node->getId() << " is: {";
    for (int i = 0; i < closeNeightborhoodNodes.size(); i++) {
        if (i == closeNeightborhoodNodes.size() - 1) {
            cout << closeNeightborhoodNodes[i]->getId() << "}\n";
            break;
        }

        cout << closeNeightborhoodNodes[i]->getId() << ", ";
    }
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

    Writer::printGraphInDotFile(minimumPathGraph, "minimumPathGraph.dot");
}

void Controller::getGraphDegree(Graph* graph) {
    cout << "The graph degree is " << graph->getDegree() << "\n";
}

// Exercício A
void Controller::getDirectTransitiveClosure(Graph* graph) {
    Node* node = Manager::selectNode(graph);

    vector<Node*> directTransitiveClosureGraph = getDirectTransitiveClosureByNode(node);

    if (directTransitiveClosureGraph.size() == 0) {
        cout << "The direct transitive closure of node " << node->getId() << " is empty\n";
        return;
    }

    cout << "The direct transitive closure of node " << node->getId() << " is: {";

    for (int i = 0; i < directTransitiveClosureGraph.size(); i++) {
        cout << "(" << node->getId() << ", " << directTransitiveClosureGraph[i]->getId() << ")";

        if (i == directTransitiveClosureGraph.size() - 1) {
            cout << "}\n";
            break;
        }

        cout << ", ";
    }
}

// Exercício B
void Controller::getIndirectTransitiveClosure(Graph* graph) {
    Node* node = Manager::selectNode(graph);

    vector<Node*> indirectTransitiveClosureGraph = getIndirectTransitiveClosureByNode(graph, node);

    if (indirectTransitiveClosureGraph.size() == 0) {
        cout << "The indirect transitive closure of node " << node->getId() << " is empty\n";
        return;
    }

    cout << "The indirect transitive closure of node " << node->getId() << " is: \n {";

    for (int i = 0; i < indirectTransitiveClosureGraph.size(); i++) {
        cout << "(" << indirectTransitiveClosureGraph[i]->getId() << ", " << node->getId() << ")";

        if (i == indirectTransitiveClosureGraph.size() - 1) {
            cout << "}\n";
            break;
        }

        cout << ", ";
    }
}

void Controller::getDegreeSequence(Graph* graph) {
    // TODO
}

void Controller::getInducedSubgraph(Graph* graph) {
    // TODO
}

void Controller::getGraphComplement(Graph* graph) {
    // TODO
}

void Controller::getStronglyConnectedComponents(Graph* graph) {
    // TODO
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
    // TODO
}

void Controller::getBridgeEdges(Graph* graph) {
}

void Controller::getGraphMetrics(Graph* graph) {
    // TODO
}

// Exercício E e F
void Controller::getMinimumSpanningTree(Graph* graph) {
    // TODO
}

void Controller::getMinimumPath(Graph* graph) {
    // TODO
}

// Exercício G

// Exercício H