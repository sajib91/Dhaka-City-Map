/*
  Author: Md. Khayrul Islam Sajib
  University of Dhaka,
  Institute of Information Technology,
  Email: bsse1552@iit.du.ac.bd
  date: 13 february 2025
*/
// Here is the test Map link via google
// https://www.google.com/maps/d/u/0/edit?mid=1me02FQCrdm8od7H_x0Kw1ztVx1ek-Ec&usp=sharing
// https://www.google.com/maps/d/u/0/edit?mid=1me02FQCrdm8od7H_x0Kw1ztVx1ek-Ec&usp=sharing


#include <bits/stdc++.h>
using namespace std;

#define INT unsigned int
#define INF (INT)1e8

struct Node {
    double lat;
    double lng;
};

struct Edge {
    int source;
    int destination;
    double distance;
    double altitude;
    string type;
    string sourceName;
    string destinationName;
};

struct DelimitedString {
    string value;
    char delimiter;
    DelimitedString(char delim = ',') : delimiter(delim) {}
    friend istream &operator>>(istream &is, DelimitedString &output) {
        getline(is, output.value, output.delimiter);
        return is;
    }
};

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    double R = 6371;
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
               sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
}

pair<vector<int>, double> dijkstra(int srcNode, int destNode, const vector<Node> &nodes, const vector<Edge> &edges, const map<int, vector<int>> &adjList) {
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
    vector<double> dist(nodes.size(), numeric_limits<double>::infinity());
    vector<int> prev(nodes.size(), -1);

    dist[srcNode] = 0;
    pq.push({0, srcNode});

    while (!pq.empty()) {
        double d = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (d > dist[u]) continue;

        if (adjList.find(u) != adjList.end()) {
            for (int v : adjList.at(u)) {
                double cost = numeric_limits<double>::infinity();
                for (const auto &edge : edges) {
                    if (edge.source == u && edge.destination == v) {
                        cost = edge.distance;
                        break;
                    }
                }

                if (dist[v] > dist[u] + cost) {
                    dist[v] = dist[u] + cost;
                    prev[v] = u;
                    pq.push({dist[v], v});
                }
            }
        }
    }

    vector<int> path;
    if (dist[destNode] != numeric_limits<double>::infinity()) {
        int curNode = destNode;
        while (curNode != -1) {
            path.push_back(curNode);
            curNode = prev[curNode];
        }
        reverse(path.begin(), path.end());
    }
    return {path, dist[destNode]};
}

void readDhakaRoute(const string &filename, string edgeType, vector<Node> &nodes, vector<Edge> &edges, map<pair<double, double>, int> &nodeIndex, map<int, vector<int>> &adjacencyList) {
    ifstream metrofile(filename);
    string metroline;

    if (metrofile.is_open()) {
        while (getline(metrofile, metroline)) {
            if (metroline.empty() || metroline[0] == '#') continue;

            stringstream ss(metroline);
            DelimitedString transportTypeStr(','), latStr(','), lonStr(','), startNameStr(','), endNameStr(',');
            vector<pair<double, double>> coordinates;

            ss >> transportTypeStr >> latStr;

            while (ss.good()) {
                DelimitedString lon(',');
                ss >> lon;

                if (!ss.fail()) {
                    double lat = 0;
                    double lonValue = 0;
                    try {
                        lat = stod(latStr.value);
                        lonValue = stod(lon.value);
                    } catch (...) {
                        break;
                    }

                    coordinates.push_back({lat, lonValue});
                    ss >> latStr;

                    if (ss.fail()) break;
                } else {
                    break;
                }
            }

            try {
                ss >> startNameStr >> endNameStr;
            } catch (...) {}

            auto addNode = [&](double latitude, double longitude) {
                pair<double, double> coords = {latitude, longitude};
                if (nodeIndex.find(coords) == nodeIndex.end()) {
                    Node newNode = {latitude, longitude};
                    nodes.push_back(newNode);
                    int index = nodes.size() - 1;
                    nodeIndex[coords] = index;
                    return index;
                } else {
                    return nodeIndex[coords];
                }
            };

            double altitude = 0;

            if (coordinates.size() >= 2) {
                int sourceNodeIndex = addNode(coordinates[0].first, coordinates[0].second);
                int destNodeIndex = addNode(coordinates.back().first, coordinates.back().second);
                double total_distance = 0;

                for (size_t i = 0; i < coordinates.size() - 1; ++i) {
                    total_distance += calculateDistance(coordinates[i].first, coordinates[i].second, coordinates[i + 1].first, coordinates[i + 1].second);
                }

                Edge edge = {sourceNodeIndex, destNodeIndex, total_distance, altitude, edgeType, startNameStr.value, endNameStr.value};
                edges.push_back(edge);
                if (adjacencyList.find(sourceNodeIndex) == adjacencyList.end()) {
                    adjacencyList[sourceNodeIndex] = vector<int>();
                }
                adjacencyList[sourceNodeIndex].push_back(destNodeIndex);
            }
        }
        metrofile.close();
    } else {
        cerr << "Unable to open file " << filename << endl;
    }
}

void readDhakaStreet(const string &filename, vector<Node> &nodes, vector<Edge> &edges, map<pair<double, double>, int> &nodeIndex, map<int, vector<int>> &adjacencyList) {
    ifstream file(filename);
    string line;

    if (file.is_open()) {
        while (getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            stringstream ss(line);

            DelimitedString dataType(','), nextValue(','), altitudeStr(','), distanceStr(',');
            vector<pair<double, double>> coordinates;
            double altitude = 0;
            double distance = 0;

            ss >> dataType;

            while (ss.good()) {
                DelimitedString latStr(','), lonStr(',');

                ss >> latStr;

                if (ss.fail()) {
                    try {
                        altitude = stod(latStr.value);
                        ss >> distanceStr;
                        distance = stod(distanceStr.value);
                    } catch (...) {
                        break;
                    }
                    break;
                }

                ss >> lonStr;
                if (ss.fail()) break;

                try {
                    double lat = stod(latStr.value);
                    double lonValue = stod(lonStr.value);
                    coordinates.push_back({lat, lonValue});
                } catch (...) {
                    break;
                }
            }

            auto addNode = [&](double latitude, double longitude) {
                pair<double, double> coords = {latitude, longitude};
                if (nodeIndex.find(coords) == nodeIndex.end()) {
                    Node newNode = {latitude, longitude};
                    nodes.push_back(newNode);
                    int index = nodes.size() - 1;
                    nodeIndex[coords] = index;
                    return index;
                } else {
                    return nodeIndex[coords];
                }
            };

            for (size_t i = 0; i < coordinates.size() - 1; ++i) {
                int srcNodeId = addNode(coordinates[i].first, coordinates[i].second);
                int destNodeId = addNode(coordinates[i + 1].first, coordinates[i + 1].second);

                Edge edge = {srcNodeId, destNodeId, distance, altitude, "road", "", ""};
                edges.push_back(edge);
                adjacencyList[srcNodeId].push_back(destNodeId);
            }
        }
        file.close();
    } else {
        cerr << "Unable to open file " << filename << endl;
    }
}

void createKMLFile(const vector<Node> &nodes, const vector<Edge> &edges, const vector<int> &path) {
    ofstream kmlFile("route.kml");
    if (kmlFile.is_open()) {
        kmlFile << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        kmlFile << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n";
        kmlFile << "<Document>\n";
        kmlFile << "<name>Route</name>\n";
        kmlFile << "<Placemark>\n";
        kmlFile << "<name>BSSE-1552</name>\n";
        kmlFile << "<Style>\n";
        kmlFile << "<LineStyle>\n";
        kmlFile << "<color>ffff00ff</color>\n";
        kmlFile << "<width>4</width>\n";
        kmlFile << "</LineStyle>\n";
        kmlFile << "</Style>\n";
        kmlFile << "<LineString>\n";
        kmlFile << "<tessellate>1</tessellate>\n";
        kmlFile << "<coordinates>\n";

        for (int i = 0; i < path.size(); ++i) {
            kmlFile << fixed << setprecision(6) << nodes[path[i]].lat << "," << nodes[path[i]].lng;
            kmlFile << fixed << setprecision(0);
            bool isAltitude = false;
            if (i < path.size() - 1) {
                for (const auto &edge : edges) {
                    if (edge.source == path[i] && edge.destination == path[i + 1]) {
                        kmlFile << "," << edge.altitude << endl;
                        isAltitude = true;
                        break;
                    }
                }
            }
            if (!isAltitude) {
                kmlFile << ",0" << endl;
            }
        }

        kmlFile << "</coordinates>\n";
        kmlFile << "</LineString>\n";
        kmlFile << "</Placemark>\n";
        kmlFile << "</Document>\n";
        kmlFile << "</kml>\n";
        kmlFile.close();
    } else {
        cerr << "Unable to open file output.kml" << endl;
    }
}

int main() {
   
    cout << fixed << setprecision(6);

    vector<Node> nodes;
    vector<Edge> edges;
    map<pair<double, double>, int> nodeId;
    map<int, vector<int>> adjList;

    readDhakaStreet("Roadmap-Dhaka.csv", nodes, edges, nodeId, adjList);
    readDhakaRoute("Routemap-DhakaMetroRail.csv", "metro", nodes, edges, nodeId, adjList);
    readDhakaRoute("Routemap-BikolpoBus.csv", "bus", nodes, edges, nodeId, adjList);
    readDhakaRoute("Routemap-UttaraBus.csv", "bus", nodes, edges, nodeId, adjList);

    cout << "Number of nodes: " << nodes.size() << endl;
    cout << "Number of edges: " << edges.size() << endl;

    double sourceLat, sourceLon, destLat, destLon;
    cout << "Enter the Source Latitude, Longitude, Destination Latitude, Longitude" << endl;
    cin >> sourceLat >> sourceLon >> destLat >> destLon;

    auto addNode = [&](double latitude, double longitude) {
        pair<double, double> coords = {latitude, longitude};
        if (nodeId.find(coords) == nodeId.end()) {
            Node newNode = {latitude, longitude};
            nodes.push_back(newNode);
            int index = nodes.size() - 1;
            nodeId[coords] = index;
            return index;
        } else {
            return nodeId[coords];
        }
    };

    int srcNode = addNode(sourceLat, sourceLon);
    int destNode = addNode(destLat, destLon);

    auto [path, totalDistance] = dijkstra(srcNode, destNode, nodes, edges, adjList);

    printf("Source: (%lf, %lf)\n", nodes[srcNode].lat, nodes[srcNode].lng);
    printf("Destination: (%lf, %lf)\n", nodes[destNode].lat, nodes[destNode].lng);

    cout << "\nShortest Path from (" << nodes[srcNode].lat << ", " << nodes[srcNode].lng << ") to ("
         << nodes[destNode].lat << ", " << nodes[destNode].lng << "):" << endl;
    bool hasPath = true;
    if (path.empty()) {
        cout << "No path exists." << endl;
        hasPath = false;
    } else {
        cout << "Path: " << endl;
        for (size_t i = 0; i < path.size(); ++i) {
            if (i < path.size() - 1) {
                string edgeType = "None";
                string sourceName = "";
                string destinationName = "";
                double cost = 0.0;

                for (const auto &edge : edges) {
                    if (edge.source == path[i] && edge.destination == path[i + 1]) {
                        edgeType = edge.type;
                        sourceName = edge.sourceName;
                        destinationName = edge.destinationName;
                        cost = edge.distance * 2;
                        break;
                    }
                }

                if (edgeType == "metro") {
                    cout << "Cost: " << fixed << setprecision(2) << cost << ": Ride Metro from "
                         << fixed << setprecision(6)
                         << sourceName << " (" << nodes[path[i]].lat << ", " << nodes[path[i]].lng << ") to "
                         << destinationName << " (" << nodes[path[i + 1]].lat << ", " << nodes[path[i + 1]].lng << ")" << endl;
                } else if (edgeType == "road" || edgeType == "bus") {
                    cout << "Cost: " << fixed << setprecision(2) << cost << ": Ride Bus from ("
                         << fixed << setprecision(6)
                         << nodes[path[i]].lat << ", " << nodes[path[i]].lng << ") to ("
                         << nodes[path[i + 1]].lat << ", " << nodes[path[i + 1]].lng << ")" << endl;
                } else {
                    cout << "Cost: 0.00: Walk from ("
                         << nodes[path[i]].lat << ", " << nodes[path[i]].lng << ") to ("
                         << nodes[path[i + 1]].lat << ", " << nodes[path[i + 1]].lng << ")" << endl;
                }
            }
        }
        cout << endl;

        cout << "Total Distance: " << totalDistance << " km" << endl;
    }

    if (hasPath) {
        cout << "Generating KML file..." << endl;
        createKMLFile(nodes, edges, path);
        cout << "KML file generated successfully." << endl;
    }

    return 0;
}