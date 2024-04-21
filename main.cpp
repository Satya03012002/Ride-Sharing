#include <iostream>
#include <vector>
#include <bits/stdc++.h>
#include <algorithm>
#include <iomanip>
#include<queue>
#include "json.hpp"

using json = nlohmann::json;
using namespace std;


#include <cmath>

using namespace std;



// double haversine(double lat1, double lon1, double lat2, double lon2) {

//      std:: cout << lat1 << "  to  " << lon1 << "------------->"<< lat2 << "  to  " <<lon2 <<std::endl;;
 
//     const double R = 6371;  // Earth's radius in kilometers

//     // Convert latitude and longitude from degrees to radians
//     const auto toRadians = [](double angle) { return angle * (M_PI / 180.0); };
//     lat1 = toRadians(lat1);
//     lon1 = toRadians(lon1);
//     lat2 = toRadians(lat2);
//     lon2 = toRadians(lon2);

//     // Differences in coordinates
//     double dlat = lat2 - lat1;
//     double dlon = lon2 - lon1;

//     // Haversine formula
//     double a = sin(dlat / 2.0) * sin(dlat / 2.0) + cos(lat1) * cos(lat2) * sin(dlon / 2.0) * sin(dlon / 2.0);
//     double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
//     double distance = R * c;

//     return distance;
// }

long double toRadians(const long double & degree)
{
    // cmath library in C++ 
    // defines the constant
    // M_PI as the value of
    // pi accurate to 1e-30
    long double one_deg = (M_PI) / 180;
    return (one_deg * degree);
}
 
long double haversine(long double lat1, long double long1, 
                     long double lat2, long double long2)
{
    // Convert the latitudes 
    // and longitudes
    // from degree to radians.
    lat1 = toRadians(lat1);
    long1 = toRadians(long1);
    lat2 = toRadians(lat2);
    long2 = toRadians(long2);
     
    // Haversine Formula
    long double dlong = long2 - long1;
    long double dlat = lat2 - lat1;
 
    long double ans = pow(sin(dlat / 2), 2) + 
                          cos(lat1) * cos(lat2) * 
                          pow(sin(dlong / 2), 2);
 
    ans = 2 * asin(sqrt(ans));
 
    // Radius of Earth in 
    // Kilometers, R = 6371
    // Use R = 3956 for miles
    long double R = 6371;
     
    // Calculate the result
    ans = ans * R*1000;
 
    return ans;
}


struct Position {
    double lat;
    double lng;
};

struct Store {
    std::string title;
    Position position;
    std::string hours;
};

 template <typename S>
void with_separator(const vector<S> &vec, string sep = " ")
{
    for (int i=0; i<vec.size()-1; i++) {
       cout << vec[i] << sep;
    }
 	cout<<vec[vec.size()-1];
   //cout << endl;
}


struct Node {
  int vertex;
  int distance;
};

bool operator<(const Node& a, const Node& b) {
  return a.distance > b.distance;
}

vector<int> dijkstra(vector<vector<int>>& adjacency_matrix, int source) {
  int n = adjacency_matrix.size();

  priority_queue<Node> pq;
  pq.push({source, 0});

  vector<int> distances(n, INT_MAX);
  distances[source] = 0;

  while (!pq.empty()) {
    Node current_node = pq.top();
    pq.pop();

    for (int i = 0; i < n; i++) {
      if (adjacency_matrix[current_node.vertex][i] > 0 && distances[i] > distances[current_node.vertex] + adjacency_matrix[current_node.vertex][i]) {
        distances[i] = distances[current_node.vertex] + adjacency_matrix[current_node.vertex][i];
        pq.push({i, distances[i]});
      }
    }
  }

  return distances;
}

class WeightedGraph
{
private:
  
  int numVertices;

public:
  vector<vector<int>> adjacencyMatrix;
  vector<int> Nodes;
  WeightedGraph(int numVertices,json stores)
  {

      //    for(int i = 0; i < numVertices; i++){
      //   // Store point_A = stores[i];

      //   for(int j = i+1; j < numVertices; j++){
            

      // int distance = haversine(stores[i]["position"]["lat"],  stores[i]["position"]["lng"], stores[j]["position"]["lat"],  stores[j]["position"]["lng"]);
      //  std::cout << stores[i]["title"] << stores[j]["title"]   << "-------------->"<< distance << std::endl;


      //   }
      // }








    this->numVertices = numVertices;
    adjacencyMatrix = vector<vector<int>>(numVertices, vector<int>(numVertices, 1e9));
    for (int i = 0; i < numVertices; i++)
    {
      int node_value = 6;
      // cout << "Enter the Value of Node"
      //      << " " << i << " : ";
      // cin >> node_value;

      Nodes.push_back(node_value);
    }

    int edge;
    //cout << "Enter No of edge : ";

   // cin >> edge;

   edge = (numVertices*(numVertices-1))/2;
   //std:: cout <<  "edge---------------" << edge <<std::endl;;

    // for (int i = 0; i < edge; i++)
    // {
    //   int s, e, w;
    //   cin >> s >> e >> w;
    //   this->addEdge(s, e, w);
    // }


       for(int i = 0; i < numVertices; i++){
        // Store point_A = stores[i];

        for(int j = i+1; j < numVertices; j++){

            

      int distance = haversine(stores[i]["position"]["lat"],  stores[i]["position"]["lng"], stores[j]["position"]["lat"],  stores[j]["position"]["lng"]);
      // std::cout << stores[i]["title"] << stores[j]["title"]   << "-------------->"<< distance << std::endl;

          int s = stores[i]["no"], e = stores[j]["no"], w = distance;
               //  std::cout << " stores[i]['no']" << s << " stores[j]['no']" << e  << "distance"<<"------->" <<w << std::endl;

      //cin >> s >> e >> w;
      this->addEdge(s, e, w);


        }
      }

      //std::cout << std::endl << std::endl;;
  }

  void addEdge(int sourceVertex, int destinationVertex, int weight)
  {
    adjacencyMatrix[sourceVertex][destinationVertex] = weight;
    adjacencyMatrix[destinationVertex][sourceVertex] = weight;
  }
		
  void printAdjacencyMatrix()
  {
    for (
        int i = 0; i < numVertices; i++)
    {
      for (int j = 0; j < numVertices; j++)
      {
        //cout << adjacencyMatrix[i][j] << " ";
      }
      //cout << endl;
    }
  }
  
  
  vector<vector<int>> get_matrix()
  {
  	return adjacencyMatrix;
  }
};


  vector<bool> visited;


	// void findAllPathsUtil(int src, int dest, int low_limit, int up_limit, vector<vector<int>> adjMatrix, vector<int>& path, vector<vector<int>> &path_list, int& pathWeight, int &pathCount) 
  // {
  	
  //       visited[src] = true;
  //       path.push_back(src);

  //       if (src == dest) {
  //           if (pathWeight <= up_limit && pathWeight>low_limit) {
  //               pathCount++;
  //              // cout << "Path " << pathCount << ": ";
  //               for (int i = 0; i < path.size(); ++i) {
  //                  // cout << path[i];
  //                   if (i != path.size() - 1){
  //                                             //cout << " -> ";

  //                   }
  //               }
  //              // cout << " (Total Weight: " << pathWeight << ")" << endl;
  //               path_list.push_back(path);
  //           }
  //       } else {
  //           for (int i = 0; i < adjMatrix[src].size(); ++i) {
  //               if (adjMatrix[src][i] != 0 && !visited[i]) {
  //                   int weight = adjMatrix[src][i];
  //                   if (pathWeight + weight <= up_limit) {
  //                       findAllPathsUtil(i, dest, low_limit, up_limit, adjMatrix, path, path_list, pathWeight += weight, pathCount);
  //                       pathWeight -= weight;
  //                   }
  //               }
  //           }
  //       }

  //       path.pop_back();
  //       visited[src] = false;
  //   }


	void findAllPathsUtil(int src, int dest, int low_limit, int up_limit, vector<vector<int>> adjMatrix, vector<int>& path, vector<vector<int>> &path_list, int& pathWeight, int &pathCount) 
  {

        visited[src] = true;
        path.push_back(src);
         
         if(path.size() >= 5){
               path.pop_back();
        visited[src] = false;
        return;
         }


        if (src == dest) {
                     //cout << "src -- " << src << "--"<< "dest --" << dest << endl;

            if (pathWeight <= up_limit && pathWeight>low_limit) {
                pathCount++;
            //    cout << "Path " << pathCount << ": ";
            //     for (int i = 0; i < path.size(); ++i) {
            //         cout << path[i];
            //         if (i != path.size() - 1){
            //                                   cout << " -> ";
            //         }
            //     }
            //     cout << " (Total Weight: " << pathWeight << ")" << endl;
                path_list.push_back(path);

                 path.pop_back();
                   visited[src] = false;
                            return;

            }
        } else {
            for (int i = 0; i < adjMatrix[src].size(); ++i) {
                if (adjMatrix[src][i] != 0 && !visited[i]) {
                    int weight = adjMatrix[src][i];
                    if (pathWeight + weight <= up_limit) {
                        findAllPathsUtil(i, dest, low_limit, up_limit, adjMatrix, path, path_list, pathWeight += weight, pathCount);
                        pathWeight -= weight;
                    }
                }
            }
        }

        path.pop_back();
        visited[src] = false;
    }

    
vector<int> find_node_sum(vector<vector<int>> &path_list, vector<int> &Nodes)
{
	vector<int> node_sum;
	for(int i=0; i<path_list.size(); i++)
	{
		int sum=0;
		for(int j=1; j<path_list[i].size()-1; j++)
		{
			sum += Nodes[ path_list[i][j] ];
		}
		node_sum.push_back(sum);
	}
	return node_sum;
}

vector<int> find_path_cost(vector<vector<int>> path_list, vector<vector<int>> Matrix)
{
	vector<int> path_cost;
	
	for(int i=0; i<path_list.size(); i++)
	{
		int cost = 0;
		
		for(int j=0; j<path_list[i].size()-1; j++)
		{
			cost+= Matrix[ path_list[i][j] ] [ path_list[i][j+1] ];
		}
		
		path_cost.push_back(cost);
	}
	
	return path_cost;
}



vector<int> optimal_path(vector<vector<int>> &Matrix, vector<int> &Nodes, vector<int> &cost, float alpha1, float alpha2, float tolerance)
{
	
	int n = Matrix.size();
	
	int max_cost1 = cost[n-1] * alpha1;
	
	vector<int> temp_path1;
	int pathWeight1 = 0;
	int pathCount1 = 0;
	visited.resize(n, false);
	//Phase 1 - alpha 1
	
	vector<vector<int>> path_list1;
    findAllPathsUtil(0, n-1, 0,  max_cost1, Matrix  , temp_path1, path_list1, pathWeight1, pathCount1);
    
    vector<int> node_sum1;
    vector<int> path_cost1;
    
    node_sum1 = find_node_sum(path_list1, Nodes);
    path_cost1 = find_path_cost(path_list1, Matrix);
    
    int max=INT_MIN;
    int max_index=0;
    
    
    for(int i=0; i<node_sum1.size(); i++)
    {	
    	if(node_sum1[i]==max)
    	{
    		if(path_cost1[i] < path_cost1[max_index])
    		{
    			max_index = i;
    			continue;
			}
		}
		
    	if(node_sum1[i] > max)
    	{
    		max=node_sum1[i];
    		max_index = i;
		}	
	}
	
	vector<int> final_path1 = path_list1[max_index];
	int final_path_cost1 = path_cost1[max_index];
	int final_node_sum1 = node_sum1[max_index];
	
	//phase 2 - alpha 2
	
//	for(auto it: final_path1)
//	{
//		cout<<it<<"--";
//	}
//

	int max_cost2 = cost[n-1] * alpha2;
	
	vector<int> temp_path2;
	vector<vector<int>> path_list2;
	int pathWeight2 = 0;
	int pathCount2 = 0;
	
	findAllPathsUtil(0, n-1, max_cost1,  max_cost2, Matrix  , temp_path2, path_list2, pathWeight2, pathCount2);
	
	vector<int> node_sum2;
    vector<int> path_cost2;
    
    node_sum2 = find_node_sum(path_list2, Nodes);
    path_cost2 = find_path_cost(path_list2, Matrix);
	
	// cost/node_sum
	
	vector<vector<int>> shortlisted_path2;
	
	for(int i=0; i<node_sum2.size(); i++)
    {	
    	float cost_ratio;
    	float node_ratio;
    	
    	cost_ratio = path_cost2[i] / final_path_cost1;
    	
    	node_ratio = node_sum2[i] / final_node_sum1;
    	
    	if(cost_ratio/node_ratio < tolerance)
    	shortlisted_path2.push_back(path_list2[i]);	
	}
	if(shortlisted_path2.size()==0)
	{
		return final_path1;
	}
	
	
	vector<int> shortlisted_node_sum = find_node_sum(shortlisted_path2, Nodes);
	vector<int> shortlisted_path_cost = find_path_cost(shortlisted_path2, Matrix);
	
	max = INT_MIN;
	max_index = 0;
	for(int i=0; i<shortlisted_node_sum.size(); i++)
    {	
    	if(shortlisted_node_sum[i]==max)
    	{
    		if(shortlisted_path_cost[i] < shortlisted_path_cost[max_index])
    		{
    			max_index = i;
    			continue;
			}
		}
		
    	if(shortlisted_node_sum[i] > max)
    	{
    		max=shortlisted_node_sum[i];
    		max_index = i;
		}	
	}
	
	return shortlisted_path2[max_index];
	
	
}




int main()
{
     json stores;
    std::cin >> stores;
     // std::cout << stores.dump(2) << std::endl;

      

    int len = 0;
    vector<Store> node;
    for (const auto& store : stores) {
        Store currentStore;
        currentStore.title = store["title"];
        currentStore.position.lat = store["position"]["lat"];
        currentStore.position.lng = store["position"]["lng"];
        currentStore.hours = store["hours"];

        // Do something with the current store...
        // std::cout << "Store Title: " << currentStore.title << std::endl;
        // std::cout << "Position (Lat, Lng): " << currentStore.position.lat << ", " << currentStore.position.lng << std::endl;
        // std::cout << "Operating Hours: " << currentStore.hours << std::endl;
        // std::cout << "-----------------------------" << std::endl;
        len++;
         node.push_back(currentStore);
    }

          //std::cout << len << std::endl;



       
      // for(int i = 0; i < len; i++){
      //   // Store point_A = stores[i];

      //   for(int j = i+1; j < len; j++){
            

      // int distance = haversine(stores[i]["position"]["lat"],  stores[i]["position"]["lng"], stores[j]["position"]["lat"],  stores[j]["position"]["lng"]);
      //  std::cout << stores[i]["title"] << stores[j]["title"]   << "-------------->"<< distance << std::endl;


      //   }
      // }










  int n;
  // cout<<"Enter number of Nodes\n";
  // cin >> n;
  n = len;
  WeightedGraph graph(n,stores);
  
  /* Edges Value 5 0 4 3 2 0 9
  				 0 4 10
				 0 2 7 
				 0 1 5 
				 0 3 6
				 1 2 3
				 1 4 8
				 1 3 2 
				 2 4 5 
				 3 4 7*/

 //graph.printAdjacencyMatrix();
  vector<int> cost;
  
  cost = dijkstra(graph.adjacencyMatrix, 0);
 // with_separator(cost);
  
  int min_cost = cost[n-1];
  
  vector<int> path;
  float alpha1 = 1.2;
  float alpha2 = 1.4;
  float tolerance = 1.2;
  
  path = optimal_path(graph.adjacencyMatrix, graph.Nodes, cost, alpha1, alpha2, tolerance);
  
  //cout<<endl<<endl<<"Recommended Path =>> ";
  
  with_separator(path, "--");
  
  return 0;
}

