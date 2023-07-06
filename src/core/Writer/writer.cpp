#include "./writer.hpp"

#include <time.h>

#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "../../lib/Edge/edge.hpp"
#include "../../lib/Graph/graph.hpp"
#include "../../lib/Node/node.hpp"

using namespace std;

void Writer::printMenu() {
    std::string options[] = {
        "leitura de arquivo (conforme entrada) e escrita em arquivo no mesmo modelo da entrada",
        "inclusão e exclusão de nó e de aresta",
        "retornar o grau de um dado nó informado pelo usuário (caso seja um dígrafo, informar grau de entrada e de saída)",
        "verificar a k-regularidade do grafo (k informado pelo usuário)",
        "informar a ordem do grafo",
        "informar se o grafo é trivial",
        "informar se o grafo é nulo",
        "mostrar a vizinhança aberta de um dado nó informado pelo usuário",
        "mostrar a vizinhança fechada de um dado nó informado pelo usuário",
        "verificar se o grafo é um multigrafo",
        "verificar se o grafo é completo",
        "verificar se o grafo é bipartido",
        "dados dois vértices informados pelo usuário, mostrar o caminho mínimo e seu custo entre esses vértices (utilizar o algoritmo de Dijkstra ou de Floyd, conforme o usuário solicite)",
        "retornar o grau do grafo",
        "retornar o fecho transitivo direto de um dado nó informado pelo usuário",
        "retornar o fecho transitivo indireto de um dado nó informado pelo usuário",
        "apresentar a sequência de graus",
        "apresentar o subgrafo induzido por um dado conjunto de vértices informado pelo usuário",
        "apresentar o complementar do grafo",
        "para digrafos, apresentar as componentes fortemente conexas",
        "verificar se o grafo é eulerianos",
        "apresentar os nós de articulação",
        "apresentar as arestas ponte",
        "apresentar o raio, o diâmetro, o centro e a periferia do grafo",
        "Apresentar a AGM do grafo ou, para grafos desconexos, as florestas de custo mínimo",
        "Apresentar o caminho mínimo entre dois vértices usando o algoritmo de Dijkstra ou de Floyd (escolha do usuário)"};

    std::cout << "\nChoose an option:\n"
              << std::endl;

    char index = 'a';
    for (const std::string& option : options) {
        std::cout << "(" << index << ") " << option << std::endl;
        index++;
    }

    std::cout << "(0) Exit" << std::endl;
}

void Writer::printNodes(Graph* graph) {
    Node* currentNode = graph->getFirstNode();
    while (currentNode != nullptr) {
        std::cout << "Node ID: " << currentNode->getId() << std::endl;
        std::cout << "Weight: " << currentNode->getWeight() << std::endl;
        std::cout << "Degree In: " << currentNode->getDegreeIn() << std::endl;
        std::cout << "Degree Out: " << currentNode->getDegreeOut() << std::endl;
        std::cout << std::endl;

        currentNode = currentNode->getNextNode();
    }
}

void Writer::printEdges(Graph* graph) {
    std::vector<Edge*> edges = graph->getEdges();

    for (Edge* edge : edges) {
        std::cout << "Weight: " << edge->getWeight() << std::endl;
        std::cout << "Head Node ID: " << edge->getHead()->getId() << std::endl;
        std::cout << "Tail Node ID: " << edge->getTail()->getId() << std::endl;
        std::cout << std::endl;
    }
}

void Writer::printGraph(Graph* graph) {
    std::cout << "Directed: " << graph->isDirected() << std::endl;
    std::cout << "Weighted Nodes: " << graph->isWeightedNodes() << std::endl;
    std::cout << "Weighted Edges: " << graph->isWeightedEdges() << std::endl;
    std::cout << std::endl;

    std::cout << "Nodes:" << std::endl;
    printNodes(graph);

    std::cout << "\n Edges:" << std::endl;
    printEdges(graph);
}