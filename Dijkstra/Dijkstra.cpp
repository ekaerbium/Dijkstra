// Dijkstra.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <vector>
#include <map>
#include <tuple>
#include <iostream>
#include <time.h>
#include <random>
#define V 200
#define BRF 50

double eucDist(std::tuple<int, int, int> p1, std::tuple<int, int, int> p2)
{
	using namespace std;
	return sqrt(pow(get<0>(p1) - get<0>(p2), 2.0)
		+ pow(get<1>(p1) - get<1>(p2), 2.0)
		+ pow(get<2>(p1) - get<2>(p2), 2.0));

}

int minDistance(std::vector<double> dist, std::vector<bool> sptSet)
{

	int min = INT_MAX, min_index;

	for (int v = 0; v < V; v++)
	{
		if (sptSet[v] == false && dist[v] <= min)
		{
			min = dist[v], min_index = v;
		}
	}
	return min_index;
}

void printSolution(std::vector<int> dist, std::vector<int> prev, int fin)
{
	int finish = fin;
	for (int i = 0; i < V; i++)
		std::cout << "Node:" << i << " Dist:" << dist[i] << " Prev:" << prev[i] << "\n";

	bool atStrt = false;
	std::vector<int> path;
	while (!atStrt)
	{
		path.insert(path.begin(), prev[fin]);
		fin = prev[fin];
		if (fin == 0 || fin == INT_MAX)
		{
			atStrt = true;
		}
	}
	if(fin != INT_MAX)
	{
		for (int i = 0; i < path.size(); i++)
		{
			std::cout << path[i] << "->";
		}
		std::cout << finish << "\n";
	}
	else
	{
		std::cout << "No paths to node " << finish << "\n";
	}
}

/*void dijkstra(std::vector<std::vector<int>> graph, int src)
{
	std::vector<int> dist (V, INT_MAX);     // The output array.  dist[i] will hold the shortest distance from src to i
	
	std::vector<bool> sptSet (V, false); // sptSet[i] will true if vertex i is included in shortest path tree or shortest distance from src to i is finalized
	
	std::vector<int> prev (V, INT_MAX); //Holds previous node on shortest path. Used to reconstruct path from start to finish

	// Distance of source vertex from itself is always 0
	dist[src] = 0;

	// Find shortest path for all vertices
	for (int count = 0; count < V - 1; count++)
	{
		// Pick the minimum distance vertex from the set of vertices not
		// yet processed. u is always equal to src in first iteration.
		int u = minDistance(dist, sptSet);

		// Mark the picked vertex as processed
		sptSet[u] = true;

		// Update dist value of the adjacent vertices of the picked vertex.
		for (int v = 0; v < V; v++)

			// Update dist[v] only if is not in sptSet, there is an edge from 
			// u to v, and total weight of path from src to  v through u is 
			// smaller than current value of dist[v]
			if (!sptSet[v] && graph[u][v] && dist[u] != INT_MAX && dist[u] + graph[u][v] < dist[v])
			{
				dist[v] = dist[u] + graph[u][v];
				prev[v] = u;
			}

	}

	
}*/
//Dijkstra based pathfinding to single node
std::vector<int> dijkstra(std::vector<std::vector<double>> graph, std::vector<std::tuple<int,int,int>> map, int src, int fin)
{
	std::vector<double> dist(graph.size(), INT_MAX);
	std::vector<double> heur(graph.size(), INT_MAX);
	std::vector<bool> sptSet(graph.size(), false);
	std::vector<int> prev(graph.size(), INT_MAX); 
	dist[src] = 0;
	heur[src] = eucDist(map[src], map[fin]);
	bool finished = false;
	while(!finished)
	{
		int u = minDistance(heur, sptSet);
		sptSet[u] = true;
		if (u == fin)
		{
			finished = true;
			continue;
		}
		for (int v = 0; v < V; v++)
			if (!sptSet[v] && graph[u][v] && dist[u] != INT_MAX && dist[u] + graph[u][v] < dist[v])
			{
				dist[v] = dist[u] + graph[u][v];
				heur[v] = dist[v] + eucDist(map[v], map[fin]);
				prev[v] = u;
			}

	}
	bool atStrt = false;
	std::vector<int> path;
	path.push_back(fin);
	while (!atStrt)
	{
		path.insert(path.begin(), prev[fin]);
		fin = prev[fin];
		if (fin == src || fin == INT_MAX)
		{
			atStrt = true;
		}
	}
	if (fin == INT_MAX)
	{
		path.clear();
		path.push_back(INT_MAX);
	}
		return path;
}

std::vector<std::vector<double>> graphGen(std::vector<std::tuple<int,int,int>> nodeMap, int brF)
{
	using namespace std;
	vector<vector<double>> nodes(V, vector<double>(V, 0));
	random_device rand;
	for (int i = 0; i < V; ++i)
	{
		for (int j = 0; j < i; ++j)
		{
			if ((rand() % 1000) < brF)
			{
				nodes[i][j] = nodes[j][i] = 
					sqrt(pow(get<0>(nodeMap[i]) - get<0>(nodeMap[j]), 2.0) 
						+ pow(get<1>(nodeMap[i]) - get<1>(nodeMap[j]), 2.0) 
						+ pow(get<2>(nodeMap[i]) - get<2>(nodeMap[j]), 2.0));
			}
		}
	}
	/*for (int i = 0; i < V; ++i)
	{
		for (int j = 0; j < V; ++j)
		{
			cout << nodes[i][j] << " ";
		}
		cout << "\n";
	}*/
	return nodes;
}

std::vector<std::tuple<int, int, int>> mapGen()
{
	using namespace std;
	random_device rd;
	vector<tuple<int, int, int>> nodeMap;
	for (int i = 0; i < V; ++i)
	{
		int x = rd() % 100;
		int y = rd() % 100;
		int z = rd() % 100;
		nodeMap.push_back(make_tuple(x, y, z));
	}
	return nodeMap;
}

int main()
{
	using namespace std;
	vector<tuple<int, int, int>> nodeMap = mapGen();
	vector<vector<double>> nodes = graphGen(nodeMap, BRF);
	for (int j = 0; j < V; ++j)
	{
		for (int k = 0; k < V; ++k)
		{
			vector<int> solution = dijkstra(nodes, nodeMap, j, k);
			if (solution[0] == INT_MAX)
			{
				cout << "No path between node " << j << " and node " << k << "\n";
				continue;
			}
			for (int i = 0; i < solution.size() - 1; ++i)
			{
				cout << solution[i] << "->";
			}
			cout << solution[solution.size() - 1] << "\n";
		}
	}
	return 0;
}
