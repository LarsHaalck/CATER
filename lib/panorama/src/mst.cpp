#include "mst.h"

namespace ht
{
Graph::Graph(int V, int E)
    : mV(V)
    , mE(E)
{
}

bool Graph::isConnected() const
{
    if (mV == 0)
        return true;

    // no node was visited
    std::vector<bool> vis(mV, false);

    // traverse from root node
    traverseVis(0, vis);

    for (int i = 0; i < mV; i++)
    {
        // single non visisted note? graph is not connected
        if (!vis[i])
            return false;
    }
    return true;
}

void Graph::traverseVis(int u, std::vector<bool>& vis) const
{
    vis[u] = true;
    for (const auto& edgeData : mEdges)
    {
        auto e = edgeData.second;
        // for every edge with one sid ebeeing u, and the other beeing unvisisted, visit it
        if (e.first == u && !vis[e.second])
            traverseVis(e.second, vis);
        if (e.second == u && !vis[e.first])
            traverseVis(e.first, vis);
    }
}

/* Functions returns weight of the MST*/
std::vector<std::pair<int, int>> Graph::kruskalMST()
{
    int mst_wt = 0; // Initialize result

    // Sort edges in increasing order on basis of cost
    std::sort(std::begin(mEdges), std::end(mEdges));

    // Create disjoint sets
    DisjointSets ds(mV);

    // Iterate through all sorted edges
    std::vector<std::pair<int, int>> mstEdges;
    for (auto it = std::begin(mEdges); it != mEdges.end(); ++it)
    {
        auto [u, v] = it->second;

        int set_u = ds.find(u);
        int set_v = ds.find(v);

        // Check if the selected edge is creating
        // a cycle or not (Cycle is created if u
        // and v belong to same set)
        if (set_u != set_v)
        {
            // Current edge will be in the MST
            // so print it
            /* std::cout << u << " - " << v << std::endl; */
            mstEdges.push_back(std::make_pair(u, v));

            // Update MST weight
            mst_wt += it->first;

            // Merge two sets
            ds.merge(set_u, set_v);
        }
    }

    return mstEdges;
}

DisjointSets::DisjointSets(int n)
    : mN(n)
    , mRnk(n + 1, 0)
    , mParent(n + 1)
{

    // Initially, all vertices are in
    // different sets and have rank 0.
    std::iota(std::begin(mParent), std::end(mParent), 0);
}

int DisjointSets::find(int u)
{
    /* Make the parent of the nodes in the path
    from u--> parent[u] point to parent[u] */
    if (u != mParent[u])
        mParent[u] = find(mParent[u]);
    return mParent[u];
}

void DisjointSets::merge(int x, int y)
{
    x = find(x), y = find(y);

    /* Make tree with smaller height
    a subtree of the other tree */
    if (mRnk[x] > mRnk[y])
        mParent[y] = x;
    else // If mRnk[x] <= mRnk[y]
        mParent[x] = y;

    if (mRnk[x] == mRnk[y])
        mRnk[y]++;
}
} // namespace ht
