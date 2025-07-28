#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
using namespace std;
#include "IOHandler.h"
#include "algorithm.h"
int main() {   
    //////////////////////////////////////// DO NOT Change this code ///////////////////////////////////////////////////
    vector<vector<float>> obstacles = readCSV("./CSV/obstacles.csv"); // obstacles.csv
    vector<vector<float>> nodes = readCSV("./CSV/nodes.csv"); // nodes.csv
    vector<vector<float>> treeNodes;
    vector<vector<int>> edges;
    vector<int> path = YourChosenAlgorithm(obstacles, nodes, treeNodes, edges);
    cout << "Path: "<< path.size() << endl;
    for (int node : path) {
        std::cout << node << "->";
    }
    writePath(path, "./CSV/path.csv");
    appendNodes("./CSV/nodes.csv", treeNodes);
    writeEdges("./CSV/edges.csv", edges);
    
    //////////////////////////////////////// DO NOT Change this code ///////////////////////////////////////////////////


    return 0;
}