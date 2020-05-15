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
    void addEdge(int u, int v, int w) { mEdges.push_back({w, {u, v}}); }
    std::vector<std::pair<int, int>> kruskalMST();

private:
    int mV;
    int mE;
    std::vector<std::pair<int, iPair>> mEdges;
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
    int mN;
    std::vector<int> mRnk;
    std::vector<int> mParent;
};

} // namespace ht
