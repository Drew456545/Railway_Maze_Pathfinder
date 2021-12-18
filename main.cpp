#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <vector>
#include <algorithm>
#include <queue>
#include <map>
#include <stdio.h>
#include <math.h>
using namespace std;

int GOODPIXELVAL, minStep, maxStep, maxDegrees, minPickup, width, height;
string file;
//pixel struct, stores RGB values
struct Pixel {
    unsigned char r, g, b;
};
Pixel *Image;
Pixel &pix(int x, int y) {
    return Image[width*y + x];
}
/**
 * x,y are coordinate positions
 * prev stores past node location so path can be printed
 */
struct Node {
    int x, y;
    pair<int, int> prev;
    int pass;
    Node(){};
    Node (int x1, int y1) {x = x1; y = y1;}
};
int inf = 999999;

Node start, finish, Dean, Lin, John1, John2;
vector < Node > all_nodes;
map < Node, double > dist;
map < Node, Node > pred;
typedef pair < double, Node > pin; // (distance to node, node)
Pixel orange = {245,102,0};

//overriding some operators
bool operator== (Node a, Node b){
    return a.x==b.x && a.y==b.y && a.prev==b.prev && a.pass==b.pass;
}
bool operator!= (Node a, Node b){
    return !(a==b);
}
//first compares x&y then prev locations, then passengers, need for DIJKSTRA
bool operator< (Node a, Node b){
    if (a.x==b.x && a.y==b.y)
        return a.prev != b.prev ? a.prev < b.prev : a.pass < b.pass;
    return a.x==b.x ? a.y < b.y : a.x < b.x;
}
//reads in config file and PPM file
void read(string filename){
    ifstream input(filename);
    string ppmfile;
    input >> ppmfile
          >> start.x >> start.y
          >> finish.x >> finish.y
          >> Dean.x >> Dean.y
          >> Lin.x >> Lin.y
          >> John1.x >> John1.y >> John2.x  >> John2.y
          >> GOODPIXELVAL >> minStep
          >> maxStep >> maxDegrees >> minPickup;
    FILE *img = fopen (ppmfile.c_str(), "r");
    fscanf (img, "P6\n%d %d\n255%*c", &width, &height);
    Image = new Pixel[width * height];
    fread (Image, width * height, sizeof(Pixel), img);
    fclose (img);
}

//Returns Euclidian distance between 2 nodes
double get_distance(Node a, Node b){
    return sqrt((a.x - b.x) * (a.x-b.x) + (a.y - b.y) * (a.y-b.y));
}
/**
 * The next three functions are calculating the angle between two nodes,
 * this is needed in order to ensure only gradual turns are  made in our path
 * @param uv coordinate pair
 * @return unit vector of the coordinate pair
 */
//Makes unit vector
pair<double, double> unit(pair<int, int> uv){
    double length = sqrt((uv.first * uv.first) + (uv.second * uv.second));
    return make_pair(uv.first/length, uv.second/length);
}
//Calculates dot product
double dot(pair<double, double> a, pair<double, double> b){
    return a.first*b.first + a.second*b.second;
}
//returns true if angle is valid, angle given in config file as a constraint
bool angle(Node prev, Node curr, Node next){
    pair<int, int> v1 = make_pair(curr.x - prev.x, curr.y - prev.y);
    pair<int, int> v2 = make_pair(next.x - curr.x, next.y - curr.y);
    double dotp = dot(unit(v1), unit(v2));
    //convert to radians
    double angle = cos(maxDegrees*M_PI/180.0);
    return dotp >= angle;
}
//prints the path
void print_path(Node start,Node goal, FILE *pathout){
    for (int i = -3; i < 3; i++)
        for (int j = -3; j < 3; j++){
            Image[(width * goal.y+i) + goal.x+j] = orange;
            Image[(width * goal.y+i) + goal.x-j] = orange;
            Image[(width * goal.y-i) + goal.x+j] = orange;
            Image[(width * goal.y-i) + goal.x-j] = orange;
        }
    if (pred[goal] != goal) print_path(start, pred[goal], pathout);
    if (&pred[goal] !=  &pred[start]) fprintf (pathout, "%d %d\n", pred[goal].x, pred[goal].y);
}
//writes to PPM file
void write(void){
    FILE *pathout = fopen ("path.txt","w");
    print_path(start, finish, pathout);
    FILE *answer = fopen ("path.ppm", "w");
    fprintf (answer, "P6\n%d %d\n255\n", width, height);
    fwrite (Image, width * height, sizeof(Pixel), answer);
    fclose(answer);
}
//When looking for node neighbors, checks that the pixel is acutally part
//of the maze
bool isValid(Pixel p){
    if(p.r + p.b + p.g <= GOODPIXELVAL) return true;
    return false;
}
/**
 * This function takes in a Node a looks for valid neighbors around the Node
 * and returns a vector of all valid neighbor Nodes
 * @param c current Node we're on in DIJKSTAS
 * @return vector of valid neighbor nodes
 */
vector<Node> get_neighbors(Node c){
    vector<Node> nbrs;
    int x = c.x;
    int y = c.y;
    //searches a square around the current Node
    for (int k = max(0, y-maxStep); k <= y+maxStep && k < height; ++k)
        for (int l = max(0, x-maxStep); l <= x+maxStep && l < width; ++l){
            //if it tries to look at invalid pixel continue
            if(!isValid(pix(l,k))) continue;
            Node nbr = Node(l, k);
            nbr.pass = c.pass;
            double distance = get_distance(c, nbr);
            //if node is too far away continue
            if (distance > maxStep || distance < minStep) continue;
            Node previous = Node(c.prev.first, c.prev.second);
            //if node does not meet angle requirements continue
            if (c != start && !angle(previous, c, nbr)) continue;
            //bitshifts if in range of passenger to pickup
            if (get_distance(nbr, Dean) <= minPickup) nbr.pass |= 1;
            if (get_distance(nbr, Lin) <= minPickup) nbr.pass |= 2;
            if (get_distance(nbr, John1) <= minPickup || get_distance(nbr, John2) <= minPickup) nbr.pass |= 4;
            nbr.prev = make_pair(x, y);
            nbrs.push_back(nbr);
        }
    return nbrs;
}

/**
 * Dijkstras shortest path algorithm that has been modified to call the get neighbors
 * function instead of some sort of adjacency list, to speed it up
 */
void dijkstra(void) {
    dist[start] = 0;
    priority_queue <pin, vector <pin>, greater <pin>> to_visit;
    to_visit.push(make_pair(0, start));
    pred[start] = start;
    while (!to_visit.empty()) {
        Node x = to_visit.top().second;
        to_visit.pop();
        if (get_distance(x, finish) <= minPickup && x.pass == 7) {
            cout << "Shortest path found\n";
            pred[finish] = x;
            return;
        }
        //instead of adjacency list, uses individual nodes to find neighbors
        for (Node n: get_neighbors(x)) {
            if (dist.find(n) == dist.end()) dist[n] = inf;
            double weight = get_distance(x, n);
            if (dist[x] + weight < dist[n]) {
                dist[n] = dist[x] + weight;
                pred[n] = x;
                to_visit.push(make_pair(dist[n], n));
            }
        }
    }
}

int main(int argc, char *argv[])
{
    read(argv[1]);
    dijkstra();
    cout << pred[finish].x << " " << pred[finish].y << "\n";
    write();
}