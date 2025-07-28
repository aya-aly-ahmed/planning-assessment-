#include <vector>
#include <cmath>
#include <limits>
#include <utility>
#include <cstdlib>  // For rand()
#include <ctime>    // For time()
#include <chrono>
#include <thread>

// Use standard namespace for convenience
using namespace std;

// Represents a node in the pathfinding graph
class Node {
public:
    int id;         // Unique identifier for the node
    float x, y;     // Coordinates of the node
    Node* parent;   // Pointer to parent node for path reconstruction
    // Default constructor
    Node() : id(0), x(0), y(0), parent(nullptr) {}
    // Constructor with id and coordinates
    Node(int id, float x, float y) : id(id), x(x), y(y), parent(nullptr) {}
};

// Global variables for storing the final path and main line
vector<Node> finalPath;
vector<pair<float, float>> mainLine;
int x = 0; // Counter for debugging (commented out in functions)

// Checks if a point (x, y) is inside any obstacle, returns radius/5 or 0.015 if inside
__float128 isInObstacle(float x, float y, const vector<vector<float>>& obstacles) {
    for (const auto& obs : obstacles) {
        if (obs.size() < 3) continue;  // Skip invalid obstacles
        float cx = obs[0], cy = obs[1], r = obs[2]/2.0; // Center and radius of obstacle
        float dist = hypot(x - cx, y - cy); // Distance from point to obstacle center
        if (dist < r) { // Point is inside obstacle
            return (r/5.0 > 0.015) ? r/5.0 : 0.015; // Return safe radius
        }
    }
    return 0; // Point is not inside any obstacle
}

// Finds the index of the closest point on the main line to (x, y)
int findClosestPointIndex(const vector<pair<float, float>>& line, float x, float y) {
    int closestIndex = -1;
    float minDist = numeric_limits<float>::max();
    for (int i = 0; i < line.size(); ++i) {
        float dx = x - line[i].first;
        float dy = y - line[i].second;
        float dist = sqrt(dx * dx + dy * dy);
        if (dist < minDist) {
            minDist = dist;
            closestIndex = i;
        }
    }
    return closestIndex;
}

// Checks if a point is close to the line from start to goal and not in an obstacle
bool isCloseToLine(float x, float y, float radius,
                   const pair<float, float>& start,
                   const pair<float, float>& goal,
                   const vector<vector<float>>& obstacles) {
    // Direction vector of the line
    float dx = goal.first - start.first;
    float dy = goal.second - start.second;
    // Line length squared (avoiding sqrt for projection)
    float lenSq = dx * dx + dy * dy;
    // Project point onto line and clamp to segment [0, 1]
    float t = ((x - start.first) * dx + (y - start.second) * dy) / lenSq;
    t = max(0.0f, min(1.0f, t));
    // Closest point on the line segment to (x, y)
    float closestX = start.first + t * dx;
    float closestY = start.second + t * dy;
    // Distance from (x, y) to that closest point
    float distToLine = hypot(x - closestX, y - closestY);
    if (distToLine > radius*2) return false; // Too far from line
    // Check if closest point is inside any obstacle
    for (const auto& obs : obstacles) {
        float cx = obs[0], cy = obs[1], r = obs[2] / 2.0f;
        float d = hypot(closestX - cx, closestY - cy);
        if (d < r) return false; // Line point is inside obstacle
    }

    // Check if point is progressing towards goal
    if (!finalPath.empty()) {
        Node last_Node = finalPath.back();
        pair<float, float> wrong_point = mainLine[findClosestPointIndex(mainLine, x, y) - 1];
        // cout <<endl<<endl<< "Last Node in main line: (" << last_Node.x << ", " << last_Node.y << ")" << endl;
        // cout << "Wrong Point: (" << wrong_point.first << ", " << wrong_point.second << ")" << endl;
        if (std::hypot(mainLine[0].first - last_Node.x, mainLine[0].second - last_Node.y) > 
            std::hypot(mainLine[0].first - wrong_point.first, mainLine[0].second - wrong_point.second)) {
            // cout<<"false is returned back"<<endl<<endl;
            return false; // Close to line but not progressing
        }
    }
    return true; // Close to line and valid
}


// Generates points on a circle around (cx, cy) with given radius
vector<pair<float, float>> generateCirclePoints(float cx, float cy, float radius, int num = 20) {
    vector<pair<float, float>> points;
    for (int i = 0; i < num; ++i) {
        float theta = 2 * M_PI * i / num; // Evenly spaced angles
        float x = cx + radius * cos(theta);
        float y = cy + radius * sin(theta);
        points.emplace_back(x, y);
    }
    return points;
}

// Finds the next point on an obstacle's surface for path continuation
pair<float, float> nextSurfacePoint(float prevX, float prevY, float curX, float curY, 
                                   const vector<vector<float>>& obstacles, float radius) {
    // cout<< endl << "radius = " << radius << endl;
    auto circle = generateCirclePoints(curX, curY, radius);
    // cout<< endl << endl << "-------------------------\n The New Node circular points: " <<endl<<endl; 
    // for (const auto& p : circle) {
    //     cout << "Circle Point: (" << p.first << ", " << p.second << ")" << endl;
    // }
    pair<float, float> edge1, edge2;
    // cout<< endl;
    // Find first transition from inside to outside
    for (int i = 0; i < circle.size(); ++i) {
        const auto& p1 = circle[i];
        const auto& p2 = circle[(i+1)%circle.size()];
        bool in1 = isInObstacle(p1.first, p1.second, obstacles);
        bool in2 = isInObstacle(p2.first, p2.second, obstacles);
        // cout << "Checking segment " << i << ": "
        //      << "(" << p1.first << ", " << p1.second << ") inObstacle = " << in1
        //      << " --> "
        //      << "(" << p2.first << ", " << p2.second << ") inObstacle = " << in2 << endl;
        if (in1 && !in2) {
            edge1 = p2; // Pick outside point
            break;
        } else if (!in1 && in2) {
            edge1 = p1; // Outside to inside
            break;
        }
    }
    // Find last transition from inside to outside (reverse)
    for (int i = circle.size() - 1; i >= 0; --i) {
        const auto& p1 = circle[i];
        const auto& p2 = circle[(i+1)%circle.size()];
        bool in1 = isInObstacle(p1.first, p1.second, obstacles);
        bool in2 = isInObstacle(p2.first, p2.second, obstacles);
        // cout << "Checking segment " << i << ": "
        //      << "(" << p1.first << ", " << p1.second << ") inObstacle = " << in1
        //      << " --> "
        //      << "(" << p2.first << ", " << p2.second << ") inObstacle = " << in2 << endl;
        if (in1 && !in2) {
            edge2 = p2; // Pick outside point
            break;
        } else if (!in1 && in2) {
            edge2 = p1;
            break;
        }
    }
    float d1 = hypot(edge1.first - prevX, edge1.second - prevY);
    float d2 = hypot(edge2.first - prevX, edge2.second - prevY);
    // cout << "Boundry1: (" << edge1.first << ", " << edge1.second << ") "
    //      << "Boundry2: (" << edge2.first << ", " << edge2.second << ")" << endl;
    return (d1 > d2) ? edge1 : edge2; // Return closer boundary point
}

// Follows the edge of an obstacle until close to the main line
vector<Node> followEdge(pair<float, float> start, float prevX, float prevY,
                        const vector<vector<float>>& obstacles,
                        const vector<pair<float, float>>& mainLine,
                        int& nodeId, float radius) {
    vector<Node> path;
    float cx = prevX;
    float cy = prevY;
    float nx = start.first;
    float ny = start.second;
    path.emplace_back(nodeId++, cx, cy); // Start at previous safe point
    while (!isCloseToLine(nx, ny, radius, mainLine.front(), mainLine.back(), obstacles) || path.size() <= 3) {
        auto next = nextSurfacePoint(cx, cy, nx, ny, obstacles, radius);
        // cout<< endl << "X choosed : " << next.first << " Y choosed : " << next.second << endl;
        cx = nx;
        cy = ny;
        nx = next.first;
        ny = next.second;
        path.emplace_back(nodeId++, cx, cy);
    }
    path.emplace_back(nodeId++, nx, ny); // Add final point
    // cout << "near to line ? " << (isCloseToLine(nx, ny, radius, mainLine.front(), mainLine.back(), obstacles) ? "true" : "false") << endl;
    // for (const auto& n : path) {
    //     cout << "Node ID: " << n.id << " X: " << n.x << " Y: " << n.y << endl << endl;
    // }
    return path;
}

// Filters a path by skipping nodes based on frequency
vector<Node> filterPath(const vector<Node>& path, int skipFrequency = 0) {
    if (path.empty()) return {};
    vector<Node> filtered;
    filtered.push_back(path[0]); // Always keep first node
    int currentId = path[0].id + 1;
    int counter = 0;
    for (size_t i = 1; i < path.size(); ++i) {
        if (skipFrequency == 0 || counter == skipFrequency) {
            Node newNode = path[i];
            newNode.id = currentId++;
            filtered.push_back(newNode);
            counter = 0; // Reset counter
        } else {
            ++counter; // Skip node
        }
    }
    return filtered;
}

// Traces a path around an obstacle from a safe point
vector<Node> traceAroundObstacle(float safeX, float safeY,
                                 const vector<vector<float>>& obstacles,
                                 const vector<pair<float, float>>& mainLine,
                                 int& nodeId, float radius) {
    auto circle = generateCirclePoints(safeX, safeY, radius);
    // for (const auto& p : circle) {
    //     cout << "Circle Point: (" << p.first << ", " << p.second << ")" << endl;
    // }
    pair<float, float> edge1, edge2;
    // Find first boundary point
    for (int i = 0; i < circle.size(); ++i) {
        // cout<< "Current Circle Point: " << circle[i].first << ", " << circle[i].second << endl;
        // cout<< "Next Circle Point: " << circle[(i+1)%circle.size()].first << ", " << circle[(i+1)%circle.size()].second << endl;
        bool in = isInObstacle(circle[i].first, circle[i].second, obstacles);
        bool nextIn = isInObstacle(circle[(i+1)%circle.size()].first, circle[(i+1)%circle.size()].second, obstacles);
        if (in != nextIn) {
            pair<float, float> p1 = circle[i];
            pair<float, float> p2 = circle[(i+1)%circle.size()];
            bool p1In = isInObstacle(p1.first, p1.second, obstacles);
            edge1 = p1In ? p2 : p1; // Choose outside point
            break;
        }
    }
    // cout << endl << endl<< "we finished the first edge and starting the second one " << endl << endl;
    // Find second boundary point (reverse direction)
    for (int i = circle.size() - 1; i >= 0; --i) {
        // cout<< "Current Circle Point: " << circle[i].first << ", " << circle[i].second << endl;
        // cout<< "previous Circle Point: " << circle[(i+1+circle.size())%circle.size()].first << ", " << circle[(i+1+circle.size())%circle.size()].second << endl;
        bool in = isInObstacle(circle[i].first, circle[i].second, obstacles);
        bool prevIn = isInObstacle(circle[(i+1+circle.size())%circle.size()].first, circle[(i+1+circle.size())%circle.size()].second, obstacles);
        if (in != prevIn) {
            pair<float, float> p1 = circle[i];
            pair<float, float> p2 = circle[(i+1+circle.size())%circle.size()];
            bool p1In = isInObstacle(p1.first, p1.second, obstacles);
            edge2 = p1In ? p2 : p1; // Choose outside point
            break;
        }
    }
    // cout << "Edge1: (" << edge1.first << ", " << edge1.second << ") "
    //      << "Edge2: (" << edge2.first << ", " << edge2.second << ")" << endl;
    auto path1 = followEdge(edge1, safeX, safeY, obstacles, mainLine, nodeId, radius);
    auto path2 = followEdge(edge2, safeX, safeY, obstacles, mainLine, nodeId, radius);
    return filterPath((path1.size() < path2.size()) ? path1 : path2); // Return shorter path
}

// Main pathfinding algorithm
vector<int> YourChosenAlgorithm(vector<vector<float>> obstacles,
                               vector<vector<float>> nodes,
                               vector<vector<float>> &pathNodes,
                               vector<vector<int>> &edges) {
    vector<int> pathIds;
    int nodeId = 3; // Start from 3 since 1 and 2 are taken
    float startX = nodes[0][1], startY = nodes[0][2];
    float goalX = nodes[1][1], goalY = nodes[1][2];
    // Calculate steps along straight line from start to goal
    float dx = goalX - startX;
    float dy = goalY - startY;
    float dist = hypot(dx, dy);
    int steps = ceil(dist / 0.01); // Step size of 0.01
    float stepX = dx / steps;
    float stepY = dy / steps;
    // cout << "dist: " << dist << ", steps: " << steps << endl;
    // cout << "stepx: " << stepX << ", stepy: " << stepY  << endl;
    // cout << "startX: " << startX << ", startY: " << startY << endl;
    // Generate main line points
    for (int i = 0; i <= steps; ++i) {
        mainLine.emplace_back(startX + i * stepX, startY + i * stepY);
    }
    // for (auto& p : mainLine) {
    //     cout <<" x = "<<p.first <<" y ="<<p.second << endl;
    // }
    pathIds.push_back(1); // Start node
    finalPath.emplace_back(1, startX, startY);
    // Iterate along the main line
    for (int i = 1; i <= steps; ++i) {
        float x = startX + i * stepX;
        float y = startY + i * stepY;
        // cout << endl << "X-Value:" << x << " Y-Value:" << y << endl;
        if (!isInObstacle(x, y, obstacles)) {
            finalPath.emplace_back(nodeId++, x, y); // Add safe point
            // cout << "Added node: " << nodeId - 1 << " X: " << x << " Y: " << y << endl;
        } else {
            float radius = isInObstacle(x, y, obstacles); // Get obstacle radius
            float safeX = startX + (i - 1) * stepX;
            float safeY = startY + (i - 1) * stepY;
            auto detour = traceAroundObstacle(safeX, safeY, obstacles, mainLine, nodeId, radius);
            finalPath.insert(finalPath.end(), detour.begin(), detour.end());
            if (!detour.empty()) {
                i = findClosestPointIndex(mainLine, detour[detour.size() - 1].x, detour[detour.size() - 1].y);
            }
        }
    }
    finalPath.emplace_back(2, goalX, goalY); // Add goal node
    // Populate output vectors
    for (const auto& n : finalPath) {
        pathIds.push_back(n.id);
        pathNodes.push_back({(float)n.id, n.x, n.y});
    }
    for (size_t i = 0; i + 1 < pathIds.size(); ++i) {
        edges.push_back({pathIds[i], pathIds[i + 1]});
    }
    return pathIds;
}