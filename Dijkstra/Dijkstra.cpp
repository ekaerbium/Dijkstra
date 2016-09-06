// Dijkstra.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <vector>
#include <map>
#include <tuple>
#include <iostream>
#include <time.h>
#include <random>
#include <fstream>
#include <thread>
#include <atomic>
#include <string>
#include <regex>
#define V 100
#define BRF 50

using namespace std;
double eucDist(tuple<int, int, int> &p1, tuple<int, int, int> &p2)
{
	return sqrt(pow(get<0>(p1) - get<0>(p2), 2.0)
		+ pow(get<1>(p1) - get<1>(p2), 2.0)
		+ pow(get<2>(p1) - get<2>(p2), 2.0));

}

int minDistance(vector<double> &dist, vector<bool> &sptSet)
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

//Dijkstra based pathfinding to single node
vector<int> astar(vector<vector<double>> &graph, vector<tuple<int,int,int>> &map, int src, int fin)
{
	vector<double> dist(graph.size(), INT_MAX);
	vector<double> heur(graph.size(), INT_MAX);
	vector<bool> sptSet(graph.size(), false);
	vector<int> prev(graph.size(), INT_MAX); 
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
	vector<int> path;
	path.push_back(fin);
	while (!atStrt)
	{
		path.insert(path.begin(), prev[fin]);
		fin = prev[fin];
		if (fin == src)
		{
			atStrt = true;
		}
		if (fin == INT_MAX)
		{
			path.clear();
			path.push_back(INT_MAX);
			atStrt = true;
		}
	}
	return path;
}

vector<vector<double>> graphGen(vector<tuple<int,int,int>> nodeMap, int brF)
{
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
	return nodes;
}

vector<tuple<int, int, int>> mapGen()
{
	random_device rd;
	vector<tuple<int, int, int>> nodeMap;
	for (int i = 0; i < V; ++i)
	{
		nodeMap.push_back(make_tuple(rd() % 100, rd() % 100, rd() % 100));
	}
	return nodeMap;
}

void thAStar(vector<vector<double>> &nodes, vector<tuple<int, int, int>> &map, vector<vector<vector<int>>>::iterator &row, vector<vector<vector<int>>>::iterator &rowFin, vector<vector<vector<int>>>::iterator &setBegin)
{
	cout << "Init thread\n";
	vector<vector<int>>::iterator col;
	for (row; row != rowFin; ++row)
	{
		for (col = row->begin(); col != row->end(); ++col)
		{
			*col = astar(nodes, map, row - setBegin, col - (row->begin()));
		}
	}
}

int main()
{
	ifstream in("C:/Users/Tab/Documents/input.txt", ios::in);
	string line;
	int lineNum = 0;
	vector<vector<int>> inNodes;
	regex notspaces("\\S+");
	while (getline(in, line))
	{
		
		lineNum++;
	}
	time_t startCalc = time(NULL);
	vector<tuple<int, int, int>> nodeMap = mapGen();
	vector<vector<double>> nodes = graphGen(nodeMap, BRF);
	vector<vector<vector<int>>> solutions (V, vector<vector<int>> (V, vector<int> ()));
	vector<thread> threads(thread::hardware_concurrency());
	cout << "Running...\n";
	int count = 0;
	int rem = V % threads.size();
	for (int i = 0; i < threads.size(); ++i)
	{
		vector<vector<vector<int>>>::iterator it1 = solutions.begin();
		vector<vector<vector<int>>>::iterator it2 = solutions.begin();
		if (rem != 0)
		{
			advance(it1, count);
			count += V / threads.size() + 1;
			rem--;
			advance(it2, count);
		}
		else
		{
			advance(it1, count);
			count += V / threads.size();
			advance(it2, count);
		}
		threads[i] = thread(thAStar, nodes, nodeMap, it1, it2, solutions.begin());
	}
	for (int i = 0; i < threads.size(); ++i)
	{
		threads[i].join();
	}
	time_t endCalc = time(NULL);
	cout << "Completed calculation in " << endCalc-startCalc << " seconds\nWriting to file\n";
	ofstream out.open("C:/Users/Tab/Documents/output.txt", ios::out);
	out.seekp(0, ios::beg);
	for (int i = 0; i < V; ++i)
	{
		for (int j = 0; j < V; ++j)
		{
			if (solutions[i][j][0] == INT_MAX)
			{
				out << "No path between node " << i << " and node " << j << "\n";
				continue;
			}
			for (int k = 0; k < solutions[i][j].size() - 1; ++k)
			{
				out << solutions[i][j][k] << "->";
			}
			out << solutions[i][j][solutions[i][j].size() - 1] << "\n";
		}
	}
	out.close();
	time_t endWrite = time(NULL);
	cout << "Written to file in " << endWrite-endCalc << " seconds\n";
	return 0;
}




/*void printSolution(std::vector<int> dist, std::vector<int> prev, int fin)
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

void dijkstra(std::vector<std::vector<int>> graph, int src)
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
