# To-do

- [x] a) Read graph from a file (according to the given input) and write to a file in the same format.
- [x] b) Add and remove nodes and edges.
- [x] c) Return the degree of a given node provided by the user (in case of a directed graph, provide both the in-degree and out-degree).
- [x] d) Check the k-regularity of the graph (where k is provided by the user).
- [x] e) Provide the order of the graph.
- [x] f) Check if the graph is trivial.
- [x] g) Check if the graph is null.
- [ ] h) Display the open neighborhood of a given node provided by the user.
- [ ] i) Display the closed neighborhood of a given node provided by the user.
- [x] j) Check if the graph is a multigraph.
- [x] k) Check if the graph is complete.
- [x] l) Check if the graph is bipartite.
- [ ] m) Given two vertices provided by the user, display the shortest path and its cost between those vertices (using either Dijkstra's or Floyd's algorithm, as requested by the user).
- [x] n) Return the degree of the graph.
- [ ] o) Return the direct transitive closure of a given node provided by the user.
- [ ] p) Return the indirect transitive closure of a given node provided by the user.
- [ ] q) Present the degree sequence.
- [ ] r) Present the subgraph induced by a given set of vertices provided by the user.
- [ ] s) Present the complement of the graph.
- [ ] t) For directed graphs, present the strongly connected components.
- [x] u) Check if the graph is Eulerian.
- [ ] v) Present the articulation points in the graph.
- [ ] w) Present the bridge edges in the graph.
- [ ] x) Present the radius, diameter, center, and periphery of the graph.
- [ ] y) Present the Minimum Spanning Tree (MST) of the graph, or for disconnected graphs, the minimum-cost forests.
- [ ] z) Present the shortest path between two vertices using either Dijkstra's or Floyd's algorithm (user's choice).

# Commands

## Create SVG from DOT

```bash
 sfdp -x -Goverlap=scale  -Tsvg x.dot > x.svg
```
