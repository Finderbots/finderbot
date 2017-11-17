#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

using namespace std;

int main() {
    return 0;
}

// take in nav_messages_occupancy_grid as a 1-D vector (graph)
// take in an x,y destination
// output command velocities... angular and linear velocities
//      geometry_msgs/Twist.h
//      at any given time you can only rotate or go forward or backward
void a_star() {
    // where N is the number of nodes
    for (int i = 0; i < N; ++i)
    {
        nodes[i].dist = infinity;
        nodes[i].parent = None;
        nodes[i].visited = false;
    }
    nodes[0].dist = infinity;
    nodes[0].parent = None;
    nodes[0].visited = false;

    visit_queue.append(nodes[0]);

    j = 0;
    while ((visit_queue.isEmpty() == false) && (current_node != goal)) {
        current_node = nodes.pop(); // calculate f_score()
        current_node.visited = true;
        for (/*each neighbor in nodes*/) {
            visit_queue.push(neighbor);
            if (neighbor.dist > current_node.dist + distance(neighbor, current_node)) {
                neighbor.parent = current_node;
                neighbor.dist = current_node.distance + distance(neighbor, current_node);
                f_score = g_score(neighbor) + h_score();
            }
        }
    }
    // output parent, distance? OR PBR the nodes and update their elements
}

int distance(node1, node2) {
    // Euclidean distance
}

int f_score(node) {

}