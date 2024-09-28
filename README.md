# Teoria dos grafos

Este trabalho foi desenvolvido como parte da disciplina Teoria dos Grafos - UFJF, ministrada pelo professor Luciana Brugiolo.

## Como rodar

```bash
g++ *.c* -o execGrupoX
```

Usage:

```bash
./execGrupoX <input_file> <output_file> <directed[0,1]> <weightedEdge[0,1]> <weightedNode[0,1]>
```

Example:

```bash
g++ *.c* -o execGrupoX && ./execGrupoX ./input/weighted_graphs/\_test.txt output.txt 1 1 1
```

## Tags

- `2023-2`: 2023-2, com Stênio Sã
- `2024-1:`, 2024-1. com Luciana Brugiolo Gonçalves

## Funcionalidades

### a) Fecho Transitivo Direto

- **Parâmetro:** ID de um vértice em um grafo direcionado.
- **Saída:** O fecho transitivo direto deste vértice, ou seja, o conjunto de vértices que podem ser alcançados a partir do vértice dado.

### b) Fecho Transitivo Indireto

- **Parâmetro:** ID de um vértice em um grafo direcionado.
- **Saída:** O fecho transitivo indireto deste vértice, ou seja, o conjunto de vértices que podem alcançar o vértice dado.

### c) Caminho Mínimo com Dijkstra

- **Parâmetro:** Dois IDs de vértices do grafo.
- **Saída:** O caminho mínimo entre estes dois vértices utilizando o algoritmo de Dijkstra.

### d) Caminho Mínimo com Floyd-Warshall

- **Parâmetro:** Dois IDs de vértices do grafo.
- **Saída:** O caminho mínimo entre estes dois vértices utilizando o algoritmo de Floyd-Warshall.

### e) Árvore Geradora Mínima (Prim)

- **Parâmetro:** Um subconjunto de vértices do grafo.
- **Saída:** A Árvore Geradora Mínima sobre o subgrafo vértice-induzido pelo subconjunto utilizando o algoritmo de Prim.

### f) Árvore Geradora Mínima (Kruskal)

convertMultigraphToGraph

- **Parâmetro:** Um subconjunto de vértices do grafo.
- **Saída:** A Árvore Geradora Mínima sobre o subgrafo vértice-induzido pelo subconjunto utilizando o algoritmo de Kruskal.

### g) Caminhamento em Profundidade

- **Parâmetro:** ID de um vértice do grafo.
- **Saída:** A árvore resultante do caminhamento em profundidade a partir do vértice dado, destacando as arestas de retorno.

### h) Raio, Diâmetro, Centro e Periferia do Grafo

- **Parâmetro:** Um grafo ponderado (direcionado ou não).
- **Saída:** O raio, diâmetro, centro e periferia do grafo.

### i) Conjunto de Vértices de Articulação

- **Parâmetro:** Um grafo não direcionado.
- **Saída:** O conjunto de vértices de articulação do grafo.

## Commands

### Create SVG from DOT

Install Graphviz:

```bash
sudo apt install graphviz
```

Run:

```bash
 sfdp -x -Goverlap=scale  -Tsvg x.dot > x.svg
```
