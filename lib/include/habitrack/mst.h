// C++ program for Kruskal's algorithm to find Minimum
// Spanning Tree of a given connected, undirected and
// weighted graph
// from https://www.geeksforgeeks.org/kruskals-minimum-spanning-tree-using-stl-in-c/
#include <bits/stdc++.h>

namespace ht
{

// Creating shortcut for an integer pair
typedef std::pair<int, int> iPair;

// Structure to represent a graph
class Graph
{
public:
    Graph(int V, int E);
    void addEdge(int u, int v, int w) { edges.push_back({w, {u, v}}); }
    int kruskalMST();

private:
    int V;
    int E;
    std::vector<std::pair<int, iPair>> edges;
};

// To represent Disjoint Sets
class DisjointSets
{
public:
    DisjointSets(int n);

    // Find the parent of a node 'u'
    // Path Compression
    int find(int u);

    // Union by rank
    void merge(int x, int y);

private:
    int* parent;
    int* rnk;
    int n;
};

} // namespace ht
