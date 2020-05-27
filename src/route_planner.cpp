#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    this->start_node = &(m_Model.FindClosestNode(start_x, start_y));
    this->end_node = &(m_Model.FindClosestNode(end_x, end_y));

}


// Use distance member function of the node struct to calculate the distance to the end node
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// Add the neighbors of the current node to the open list.
// Calculate their g and h values.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    current_node->FindNeighbors();

    // Calculate g and h values for all the neighbors
    for(auto neighbor : current_node->neighbors)
    {
        // Add the distance between the two nodes to the current g to calculate the new g.
        float g_neighbor = current_node->g_value + current_node->distance(*neighbor);
        // Calculate h value for the neighbor.
        float h_neighbor = this->CalculateHValue(neighbor);

        neighbor->g_value = g_neighbor;
        neighbor->h_value = h_neighbor;

        // Set the neighbor's parent
        neighbor->parent = current_node;

        // Add neighbor to the open list
        neighbor->visited = true;
        open_list.push_back(neighbor);
    }
}

bool RoutePlanner::CompareNodes(RouteModel::Node* n1, RouteModel::Node* n2)
{
    float f1 = n1->g_value + n1->h_value;
    float f2 = n2->g_value + n2->h_value;
    return f1 > f2;
}


// Sort the open list using the static private member function CompareNodes
// Remove the element with the least F value
// Returns the element with the least F value
RouteModel::Node *RoutePlanner::NextNode() {

    RouteModel::Node* nextNode = nullptr;

    if(open_list.size() > 0)
    {
    std::sort(open_list.begin(), open_list.end(), CompareNodes);
        nextNode = open_list.back();
        open_list.pop_back();
    }

    return nextNode;
}


// Construct path by checking if the parent is not null
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // Iterate until there is no parent, logically it means we reached the start node
    while (current_node->parent != nullptr)
    {
        path_found.push_back(*current_node);
        distance += current_node->distance(*(current_node->parent));
        current_node = current_node->parent;
    }

    // Add start node to path
    path_found.push_back(*current_node);

    // Invert vector to have the start node as the first element
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// A* Search algorithm.
// Add start node to open list
// While loop : sort, compare then expand
void RoutePlanner::AStarSearch() {

    RouteModel::Node *current_node = nullptr;

    // Add start node to the open list
    this->start_node->visited = true;
    open_list.push_back(this->start_node);

    while (open_list.size() > 0)
    {
        // Get the next node to be expanded
        current_node = this->NextNode();

        // Check if the current node is the end node
        if((current_node->x == this->end_node->x) && (current_node->y == this->end_node->y))
        {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }

        // Expand current and add neighbors
        this->AddNeighbors(current_node);
    }
    

}