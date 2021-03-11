// Example of Dijkstra's algorithm
// https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-006-introduction-to-algorithms-fall-2011/lecture-videos/MIT6_006F11_lec16.pdf
// Good video on heaps https://www.youtube.com/watch?v=t0Cq6tVNRBA&t=316s
#include <iostream>
#include <vector>
#include <utility>
#include <unordered_map>
#include <limits.h>

struct Vertex
{
    int name;
    int cost;
    Vertex(int n, int c)
        : name(n), cost(c) {}
};

void fillAdj(std::unordered_map<int, std::vector<int>>& adj);
void swap(std::vector<Vertex>& heap, int i1, int i2, std::unordered_map<int, int>& heapMap);
// The heap will only hold positive values, the first value is the vertex and the second is the current difficulty to get to said node
Vertex getMin(std::vector<Vertex>& heap);
Vertex extractMin(std::vector<Vertex>& heap, std::unordered_map<int, int>& heapMap);
void minHeapify(std::vector<int>&  heap, size_t index, std::unordered_map<int, int>& heapMap);
void buildMinHeap(std::vector<Vertex>& heap, std::unordered_map<int, int>& heapMap);
void bubbleUp(std::vector<Vertex>& heap, std::unordered_map<int, int>& heapMap, int vIndex);
void insert(std::vector<Vertex>& heap, Vertex vertex, std::unordered_map<int, int>& heapMap);
void dijkstra(std::unordered_map<int, std::vector<int>>& adj, size_t n, int s);
void decreasePriority(std::vector<Vertex>& heap, std::unordered_map<int, int>& heapMap, int v, int newValue);

// Even indices are the reachable nodes and the odd indices are the weight of the edge to the node in the previous index
void fillAdj(std::unordered_map<int, std::vector<int>>& adj )
{
    adj[0] = {1, 3, 4, 1};
    adj[1] = {0, 3, 2, 2, 3, 4};
    adj[2] = {1, 2, 3, 6, 5, 3};
    adj[3] = {1, 4, 2, 6, 6, 4};
    adj[4] = {0, 1, 8, 9, 9, 8};
    adj[5] = {2, 3, 10, 8};
    adj[6] = {3, 4, 7, 2};
    adj[7] = {6, 2, 10, 7};
    adj[8] = {4, 9, 10, 2};
    adj[9] = {4, 8, 10, 4};
    adj[10] = {5, 8, 7, 7, 8, 2, 9, 4};
}

// Swap to values in the heap and reflect the changes in the heap map
void swap(std::vector<Vertex>& heap, int i1, int i2, std::unordered_map<int, int>& heapMap)
{
    
    Vertex temp = heap.at(i1);
    int t = heapMap.at(heap.at(i1).name);

    heapMap.at(heap.at(i1).name) = heapMap.at(heap.at(i2).name);
    heap.at(i1) = heap.at(i2);

    heapMap.at(heap.at(i2).name) = t;
    heap.at(i2) = temp;
}

Vertex getMin(std::vector<Vertex>& heap)
{
    if (heap.size() > 0)
    {
        return heap.at(0);
    }
    else
    {
        return {-1, -1};
    }
}

Vertex extractMin(std::vector<Vertex>& heap, std::unordered_map<int, int>& heapMap)
{
    Vertex min = heap.at(0);
    // Move the last element to root
    int lastElement = heap.size() - 1;
    heap.at(0) = heap.at(lastElement);
    heapMap.at(heap.at(lastElement).name) = 0; 
    // Remove the last element in the heap and the corresponding value in the heapMap
    heapMap.erase(min.name);
    heap.erase(heap.begin() + lastElement);
    // Move the new root node to the correct position
    size_t index = 0;
    while (index < heap.size())
    {
        size_t smallest = index;
        size_t left = index * 2 + 1;
        size_t right = index * 2 + 2;
        // 
        if (left < heap.size() && heap.at(left).cost < heap.at(index).cost)
            smallest = left;
        if (right < heap.size() && heap.at(right).cost < heap.at(smallest).cost)
            smallest = right;
        
        if (smallest != index)
        {
            // One of the children is smaller than the parent
            swap(heap, index, smallest, heapMap);
            // Update the index
            index = smallest;
        }
        else
        {
            // Parent is small than its children
            break;
        }
    }
    return min;
}

void minHeapify(std::vector<Vertex>& heap, size_t index, std::unordered_map<int, int>& heapMap)
{
    if (index < heap.size() / 2)
    {
        size_t smallest = index;
        size_t left = index * 2 + 1;
        size_t right = index * 2 + 2;

        if (left < heap.size() && heap.at(left).cost < heap.at(index).cost)
            smallest = left;
        if (right < heap.size() && heap.at(right).cost < heap.at(smallest).cost)
            smallest = right;

        if (index != smallest)
        {
            swap(heap, index, smallest, heapMap);
            // heapify the affect sub tree
            minHeapify(heap, smallest, heapMap);
        }
    }
}

void buildMinHeap(std::vector<Vertex>& heap, std::unordered_map<int, int>& heapMap)
{
    for (int i = int(heap.size()) / 2; i> -1; i--)
    {
        minHeapify(heap, i, heapMap);
    }
}

void bubbleUp(std::vector<Vertex>& heap, std::unordered_map<int, int>& heapMap, int vIndex)
{
    int parent = (vIndex - 1) / 2;
    if (parent > -1 && heap.at(parent).cost > heap.at(vIndex).cost)
    {
        swap(heap, vIndex, parent, heapMap);
        bubbleUp(heap, heapMap, parent);
    }

}

// Inserts new value in to the heap and into heapMap, does not insert dupilcates because this will cause issues with the heap map
void insert(std::vector<Vertex>& heap, Vertex vertex, std::unordered_map<int, int>& heapMap)
{
    if (heapMap.find(vertex.name) == heapMap.end())
    {
        heap.push_back(vertex);
        heapMap[vertex.name] = heap.size() - 1;
    }
}

void decreasePriority(std::vector<Vertex>& heap, std::unordered_map<int, int>& heapMap, int v, int newValue)
{
        int index = heapMap.at(v);
        heap.at(index).cost = newValue;
        bubbleUp(heap, heapMap, index);
}

// At the moment returns a pointer to an array on the heap change this
// n is the number of vertex in the graph and s is the start vertex
// The end is vertex 10, if you just want to find the shortest path to each vertex just remove the || on the while loop
void dijkstra(std::unordered_map<int, std::vector<int>>& adj, size_t n, int s)
{
    bool visited[n];
    int distance[n];
    int prev[n];
    for (size_t i = 0; i < n; i++)
    {
        visited[i] = false;
        distance[i] = INT_MAX;
        prev[i] = -1;
    }
    distance[s] = 0;
    std::vector<Vertex> pq;
    std::unordered_map<int, int> pqMap;
    insert(pq, {s, distance[s]}, pqMap);
    
    while (pq.size() != 0)
    {
        Vertex cVertex = extractMin(pq, pqMap);
        visited[cVertex.name] = true;
        // Loop over all edges of the current vertex
        for (size_t edge = 0; edge < adj.at(cVertex.name).size(); edge+=2)
        {
            // If the Vertex has been visted ignore it.
            if (!visited[adj.at(cVertex.name).at(edge)])
            {
                // The vertex that is currently being viewed
                Vertex touch(adj.at(cVertex.name).at(edge), distance[adj.at(cVertex.name).at(edge)]);

                // The adjacency list is ordered as reachable vertex and in the next index the weight of that edge, + 1 to the current edge with give its weigth
                int newDist = cVertex.cost + adj.at(cVertex.name).at(edge + 1);
                if (newDist < touch.cost)
                {
                    // Add the touched vertex to the priority queue if it is not already in the priority queue
                    insert(pq, touch, pqMap);
                    // Relax the edge
                    decreasePriority(pq, pqMap, touch.name, newDist);
                    distance[touch.name] = newDist;
                    prev[touch.name] = cVertex.name;
                }
            }
        }
    }
    for (size_t i = 0; i < adj.size(); i++)
    {
        std::cout << "Vertex: " << i << " Distance: " << distance[i] << " Previous: " << prev[i] << "\n"; 
    } 
}

int main(int argc, char* args[])
{
    std::unordered_map<int ,std::vector<int>> adj;
    fillAdj(adj);
    // Map to stores the vertex as a key and current position in the heap as the value
    std::unordered_map<int, int> heapMap = {{5, 0}, {4, 1}, {3, 2}, {2, 3}, {1, 4}, {0, 5}};

    std::vector<Vertex> heap = {{5, 7}, {4, 10}, {3, 2}, {2, 1}, {1, 5}, {0, 0}};
    buildMinHeap(heap, heapMap);
    dijkstra(adj, adj.size(), 0);
    return 0;
}

