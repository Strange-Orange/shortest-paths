#include <limits.h>
#include <iostream>
#include <vector>
#include <unordered_map>

struct DistAndPrev
{
    std::vector<int> m_dist;
    std::vector<int> m_prev;
};

// The Adjacency list is a vertex as a key and then reachable vertices on the even indices and weights of the previous edge on the odd indices
void init_adj_list(std::unordered_map<int, std::vector<int>>& p_adjList);
DistAndPrev bellman_ford(std::unordered_map<int, std::vector<int>>& p_adjList);

void init_adj_list(std::unordered_map<int, std::vector<int>>& p_adjList)
{
    p_adjList[0] = {1, 1, 2, 2};
    p_adjList[1] = {3, 3};
    p_adjList[2] = {5, 1};
    p_adjList[3] = {2, 2};
    p_adjList[4] = {3, 1};
    p_adjList[5] = {4, -6};
}

DistAndPrev bellman_ford(std::unordered_map<int, std::vector<int>>& p_adjList)
{
    std::vector<int> l_dist(p_adjList.size());
    std::vector<int> l_compareDist(p_adjList.size());
    std::vector<int> l_prev(p_adjList.size());
    for (size_t i = 0; i < p_adjList.size(); i++)
    {
        l_dist.at(i) = INT_MAX;
        l_compareDist.at(i) = INT_MAX;
        l_prev.at(i) = -1;
    }
    // Start from index 0
    l_dist[0] = 0;
    
    for (size_t i = 0; i < p_adjList.size() - 1; i++)
    {
        for (size_t vertex = 0; vertex < p_adjList.size(); vertex++)
        {
            for (size_t edgeIndex = 0; edgeIndex < p_adjList.at(vertex).size(); edgeIndex+=2)
            {
                int l_touchingVertex = p_adjList.at(vertex).at(edgeIndex);
                int l_weight = p_adjList.at(vertex).at(edgeIndex + 1);
                // Relex the edge
                if (l_dist.at(vertex) != INT_MAX && (l_dist.at(l_touchingVertex) > l_dist.at(vertex) + l_weight))
                {
                    l_dist.at(l_touchingVertex) = l_dist.at(vertex) + l_weight;
                    l_prev.at(l_touchingVertex) = vertex;
                }
            }
        }
    }
    // Check for negative cycles run the algorithm again
    for (size_t i = 0; i < p_adjList.size() - 1; i++)
    {
        for (size_t vertex = 0; vertex < p_adjList.size(); vertex++)
        {
            for (size_t edgeIndex = 0; edgeIndex < p_adjList.at(vertex).size(); edgeIndex+=2)
            {
                int l_touchingVertex = p_adjList.at(vertex).at(edgeIndex);
                int l_weight = p_adjList.at(vertex).at(edgeIndex + 1);
                // If the current vertex has a distance of INT_MIN then it is part of a negative cycle of the vertext being touch is affected by the negative cycle
                if (l_dist.at(vertex) == INT_MIN || l_dist.at(l_touchingVertex) > l_dist.at(vertex) + l_weight)
                {
                    // Found a negative cycle
                    l_dist.at(l_touchingVertex) = INT_MIN;
                    l_prev.at(l_touchingVertex) = INT_MIN;
                }
            }
        }
    }

    return {l_dist, l_prev};

}

int main(int argc, char* args[])
{
    std::unordered_map<int, std::vector<int>> adjList;
    init_adj_list(adjList);
    DistAndPrev l_dp = bellman_ford(adjList);

    for (size_t i = 0; i < adjList.size(); i++)
    {
        std::cout << "Vertex: " << i << " Shortest distance: " << l_dp.m_dist.at(i) << " Previous vertex: " << l_dp.m_prev.at(i) << "\n";
    }

    return 0;
}