// Bi-directional Dijkstra
#include "indexedPriorityQueue.h"

#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <utility>
#include <functional>
#include <limits.h>

std::hash<std::string> g_stringHash;

struct Edge
{
    std::string m_dest;
    int m_weight;

    Edge()
        : m_dest("-1"), m_weight(0) {};

    Edge(std::string dest, int weight)
        : m_dest(dest), m_weight(weight) {}
};

struct Vertex
{
    std::string m_name;
    int m_cost;

    Vertex()
        : m_name("*"), m_cost(INT_MAX) {};

    Vertex(std::string name, int cost)
        : m_name(name), m_cost(cost) {};

    bool operator<(const Vertex& rhs) const
    {
        return this->m_cost < rhs.m_cost;
    }

    bool operator>(const Vertex& rhs) const
    {
        return this->m_cost > rhs.m_cost;
    }

    bool operator==(const Vertex& rhs) const
    {
        return this->m_name == rhs.m_name;
    }
};

struct VertexHash
{
    size_t operator()(const Vertex& v) const
    {
        return g_stringHash(v.m_name);
    }
};

typedef std::unordered_map<std::string, std::vector<Edge>> adjacencyList;
typedef std::unordered_map<std::string, std::pair<bool, bool>> processedMap;

// All data that needs to be tracked per direction
struct SearchData
{
    std::unordered_map<std::string, bool> m_visited;
    std::unordered_map<std::string, int> m_distance;
    // No previous vertex is represented as"-1"
    std::unordered_map<std::string, std::string> m_previous;
    IndexedPriorityQueue<Vertex, VertexHash> m_pq;

    SearchData() {};
    // Initialize all values for a Dijkstra using a vector of all vertices
    SearchData(const std::vector<std::string>& p_vertices)
    {
        for (const std::string v: p_vertices)
        {
            m_visited[v] = false;
            m_distance[v] = INT_MAX;
            m_previous[v] = "-1";
        }
    }
};

adjacencyList create_adj_list();
adjacencyList create_backward_adj_list(const adjacencyList& p_forwardAdj);
std::vector<std::string> all_vertices(const adjacencyList& p_adj);
std::unordered_set<std::string> create_frontier(const std::string& p_start, const adjacencyList& p_adj);
std::vector<std::string> construct_shortest_path(const SearchData& p_fsd, const SearchData& p_bsd, const std::string& p_meetingPoint, const adjacencyList& p_forwardAdj, const adjacencyList& p_backwardAdj);
void search(SearchData& p_sd, const Vertex& p_start, const std::vector<Edge>& p_neighbours, processedMap& p_processed, bool p_forward=true);
std::vector<std::string> bi_directional_dijkstra(const adjacencyList& p_forward, const adjacencyList& p_backward, const std::vector<std::string>& p_vertices, const std::string& p_start, const std::string& p_end);

adjacencyList create_adj_list()
{
    adjacencyList l_adj;
    // Simple graph
    // l_adj["s"] = {{"u", 3}, {"w", 5}};
    // l_adj["u"] = {{"u`", 3}};
    // l_adj["w"] = {{"t", 5}};
    // l_adj["u`"] = {{"t", 3}}; 

    l_adj["s"] = {{"a", 2}, {"b", 1}, {"c", 2}};
    l_adj["a"] = {{"d", 3}};
    l_adj["b"] = {{"e", 1}, {"f", 4}};
    l_adj["c"] = {{"g", 3}};
    l_adj["d"] = {{"h", 3}};
    l_adj["e"] = {{"i", 3}, {"f", 1}};
    l_adj["f"] = {{"i", 1}};
    l_adj["g"] = {{"i", 2}, {"j", 3}};
    l_adj["h"] = {{"k", 4}};
    l_adj["i"] = {{"k", 1}};
    l_adj["j"] = {{"k", 1}};
    return l_adj;
}

adjacencyList create_backward_adj_list(const adjacencyList& p_forwardAdj)
{
    adjacencyList l_backward;

    for (const std::pair<std::string, std::vector<Edge>>& item: p_forwardAdj)
    { 
        if (!item.second.empty())
        {
            for (const Edge& e: item.second)
            {
                std::string l_vertex = e.m_dest;
                // Create an entry in the backward adj list if this vertex has not been seen.
                if (l_backward.find(l_vertex) == l_backward.end())
                {
                    l_backward[l_vertex];
                }
                // Add 'parent' vertex to child vertices adj list to in effect creating reverse version of each edge
                l_backward[l_vertex].push_back({item.first, e.m_weight});
            }
        } 
    }
    return l_backward;
}

std::vector<std::string> all_vertices(const adjacencyList& p_adj)
{
    std::vector<std::string> l_vertices;
    std::unordered_set<std::string> l_set;
    for (const std::pair<std::string, std::vector<Edge>>& item: p_adj)
    {
        if (l_set.find(item.first) == l_set.end())
        {
            l_set.insert(item.first);
            l_vertices.emplace_back(item.first);
        }
        for (const Edge edge: item.second)
        {
            if (l_set.find(edge.m_dest) == l_set.end())
            {
                l_set.insert(edge.m_dest);
                l_vertices.emplace_back(edge.m_dest);
            }
        }
    }
    return l_vertices;
}

// Create a frontier from the p_start vertex in the graph. Will place all reachable vertices in 1 jump in the frontier and return it.
std::unordered_set<std::string> create_frontier(const std::string& p_start, const adjacencyList& p_adj)
{
    std::unordered_set<std::string> l_pf;
    for (const Edge& e: p_adj.at(p_start))
    {
        l_pf.insert(e.m_dest);
    }
    return l_pf; 
}

// Check if the meeting point is in fact on the shortest path, look at the front and backward frontier and make there is not a short path around the meeting point.
// The meeting point will always be the smallest number of edges not necessarily the shorts distance in terms of weights.
std::vector<std::string> construct_shortest_path(const SearchData& p_fsd, const SearchData& p_bsd, const std::string& p_meetingPoint, const adjacencyList& p_forwardAdj, const adjacencyList& p_backwardAdj)
{
    std::unordered_set<std::string> l_forwardFrontier = create_frontier(p_fsd.m_previous.at(p_meetingPoint), p_forwardAdj);
    std::unordered_set<std::string> l_backwardFrontier = create_frontier(p_bsd.m_previous.at(p_meetingPoint), p_backwardAdj);
    // The meeting point is currently the shortest distance
    int l_smallest = p_fsd.m_distance.at(p_meetingPoint) + p_bsd.m_distance.at(p_meetingPoint);
    std::string l_forwardBridgePoint = p_meetingPoint;
    std::string l_backwardBridgePoint = p_bsd.m_previous.at(p_meetingPoint);
    for (const std::string& vertex: l_forwardFrontier)
    {
        // Ignore the meeting point
        if (vertex != p_meetingPoint)
        {
            int fDistance = p_fsd.m_distance.at(vertex);
            for (const Edge edge: p_forwardAdj.at(vertex))
            {
                // If the vertex begin touched is in the backward frontier and the distance is lower than smallest update the smallest weight path
                if ((l_backwardFrontier.find(edge.m_dest) != l_backwardFrontier.end()) && ((fDistance + edge.m_weight) + p_bsd.m_distance.at(edge.m_dest) < l_smallest))
                {
                    l_smallest = (fDistance + edge.m_weight) + p_bsd.m_distance.at(edge.m_dest);
                    // Update the join a the front and backward frontier
                    l_forwardBridgePoint = vertex;
                    l_backwardBridgePoint = edge.m_dest;
                }
            }
        }
    }
    std::vector<std::string> l_path;
    // Forward search to the bridge point, then the backward
    while (l_forwardBridgePoint != "-1")
    {
        l_path.emplace_back(l_forwardBridgePoint);
        l_forwardBridgePoint = p_fsd.m_previous.at(l_forwardBridgePoint);
    }
    std::reverse(l_path.begin(), l_path.end());
    while (p_bsd.m_previous.find(l_backwardBridgePoint) != p_bsd.m_previous.end())
    {
        l_path.emplace_back(l_backwardBridgePoint);
        l_backwardBridgePoint = p_bsd.m_previous.at(l_backwardBridgePoint);
    }
    return l_path;
}

// Look at all neighbours of the p_start vertex and relax needed edges. forward sets the direction that will set when processed because both directions need to see if they have both processed vertex
void search(SearchData& p_sd, const Vertex& p_start, const std::vector<Edge>& p_neighbours, processedMap& p_processed, bool p_forward)
{
    if (p_sd.m_visited.at(p_start.m_name) == false)
    {
        p_sd.m_visited.at(p_start.m_name) = true;
        // Follow each edge
        for (const Edge& edge: p_neighbours)
        {
            // If a vertex has been seen before ignore it
            if (p_sd.m_visited.at(edge.m_dest) == false)
            {
                int l_newDist = p_sd.m_distance.at(p_start.m_name) + edge.m_weight;
                // Relax the edge, update the priority and update the previous
                if (l_newDist < p_sd.m_distance.at(edge.m_dest))
                {
                    // Insert the touched vertex into the priority queue
                    p_sd.m_pq.insert({edge.m_dest, INT_MAX});

                    p_sd.m_pq.change_priority({edge.m_dest, p_sd.m_distance.at(edge.m_dest)}, {edge.m_dest, l_newDist});
                    p_sd.m_distance.at(edge.m_dest) = l_newDist;
                    p_sd.m_previous.at(edge.m_dest) = p_start.m_name;
                }
            }
        }
        // Set the start node as processed
        if (p_forward)
        {
            p_processed[p_start.m_name].first = true;
        }
        else 
        {
            p_processed[p_start.m_name].second = true;
        }
    }
}

std::vector<std::string> bi_directional_dijkstra(const adjacencyList& p_forward, const adjacencyList& p_backward, const std::vector<std::string>& p_vertices, const std::string& p_start, const std::string& p_end)
{
    SearchData l_forwardData(p_vertices);
    SearchData l_backwardData(p_vertices);
    // Tracks vertices that have been processed and by which direction
    processedMap l_processed;

    l_forwardData.m_pq.insert({p_start, 0});
    l_forwardData.m_distance.at(p_start) = 0;
    l_backwardData.m_pq.insert({p_end, 0});
    l_backwardData.m_distance.at(p_end) = 0;

    std::string l_meetPoint;
    while ((!l_forwardData.m_pq.empty()) || (!l_backwardData.m_pq.empty()))
    {
        Vertex l_forwardMin = l_forwardData.m_pq.extract_min();
        Vertex l_backwardMin = l_backwardData.m_pq.extract_min();
        search(l_forwardData, l_forwardMin, p_forward.at(l_forwardMin.m_name), l_processed);
        search(l_backwardData, l_backwardMin, p_backward.at(l_backwardMin.m_name), l_processed, false);

        // Check if a vertex has been processed by both searches, if the vertex processed by the forward has already been procesesd by the second or vice versa
        if (l_processed.at(l_forwardMin.m_name).second)
        {
            std::cout << "Found the 'middle as such' forward at " << l_forwardMin.m_name << std::endl;
            l_meetPoint = l_forwardMin.m_name;
            break;
        }
        if (l_processed.at(l_backwardMin.m_name).first)
        {
            std::cout << "Found the 'middle as such' backward at " << l_backwardMin.m_name << std::endl;
            l_meetPoint = l_forwardMin.m_name;
            break;
        }
    }

    return construct_shortest_path(l_forwardData, l_backwardData, l_meetPoint, p_forward, p_backward);
}

int main(int argc, char* args[])
{
    adjacencyList l_forward = create_adj_list();
    adjacencyList l_backward = create_backward_adj_list(l_forward);
    std::vector<std::string> l_vertices = all_vertices(l_forward);
    std::vector<std::string> l_path = bi_directional_dijkstra(l_forward, l_backward, l_vertices, "s", "k");
    for (const std::string& s: l_path)
    {
        std::cout << s << " ";
    }
    std::cout << std::endl;

    return 0;
}