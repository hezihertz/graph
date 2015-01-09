#include <vector>
#include <fstream>
#include <limits>

#include "CS207/SDLViewer.hpp"
#include "CS207/Util.hpp"
#include "CS207/Color.hpp"

#include "Graph.hpp"

/** Find the bridge between 2 subgraphs inside a graph, given 1 node in each
 *subgraph, denoted n1 and n2
 *
 * Steps:
 * 1. Find the shortest path from n1 to n2, using Dijkstra's algorithm, the
 *		bridge must be in the path;
 * 2. For each edge along the shortest path (say, {n1,n3}), try to find another
 *		path between node n1 and n3 without using the edge {n1, n3};
 * 3. Check if the neighbor nodes (say nodes n1 and n3) can still find each
 *		other;
 * 4. If yes, keep going along the path, select next edge, and try 2;
 * 5. If no, then we've found the bridge.
 */


/** Find if neighbor nodes @a n1, and @a n2 have a path between them without
 * using the edge they share, i.e. {n1, n2}
 *
 * @pre graph.has_edge(n1, n2) == true
 * @pre for every node in graph, node.value().visited == false
 * @return true, if there exists a set of node in graph, {Node_1, Node_2, ...
 *Node_t}, t>0 and @a n1 and @a n2 not included, so that graph.has_edge(n1,
 *Node_1) && ... && graph.has_edge(Node_t-1, Node_t) && graph.has_edge(Node_t,
 *n2) == true.
 * @return false otherwise
 */
template <typename Node>
bool is_path_not_direct(Node n1, Node n2) {
  auto incident_iterator_n1 = n1.edge_begin();

  while (incident_iterator_n1 != n1.edge_end() &&
         (*incident_iterator_n1).node2() == n2)
    ++incident_iterator_n1;
  if (incident_iterator_n1 == n1.edge_end())
    return false;  // meaning n1 is only connected to n2

  // Found a neighbor node of n1 that is not n2
  auto neighbor_node = (*incident_iterator_n1).node2();
  // Start BFS for a path between neighbor_node and n2
  n1.value().visited = true;
  neighbor_node.value().visited = true;
  std::vector<Node> node_queue{n1, neighbor_node};
  unsigned queue_ptr = 1;

  while (queue_ptr < node_queue.size()) {
    auto curr_node = node_queue[queue_ptr];
    for (auto inc_iter = curr_node.edge_begin();
         inc_iter != curr_node.edge_end(); ++inc_iter) {
      // node hasn't been visited
      if ((*inc_iter).node2().value().visited == false) {
        auto next_node = (*inc_iter).node2();
        if (next_node == n2) {
          // found an alternative path and reset visited node indicator before
          // return
          for (auto n : node_queue)
            n.value().visited = false;
          return true;
        }
        next_node.value().visited = true;
        node_queue.push_back(next_node);
      }
    }
    ++queue_ptr;
  }
  for (auto n : node_queue)
    n.value().visited = false;
  return false;
}

/** Find the shortest path from node @a n1 to @a n2, stored in @a path,
 *Dijkstra's algorithm
 * @pre path.size()=0
 * @pre for all node in graph, node.value() == default value, i.e.
 * 	node.value().visited == false
 * 	node.value().queued == false
 * 	node.value().prev_node.is_valid() == false
 * 	node.value().distance = std::numeric_limits<int>::max()
 *
 * @post path.front() == n2, path.back() == n1
 * @post 2 consecutive nodes in @a path share an edge, i.e., for
 *  0<=i<path.size()-1, graph.has_edge(path[i], path[i+1])
 * @post for all node in graph, node.value().distance is updated
 *
 * @return true if the shortest path between node @a n1 and @a n2 is found,
 * @return false otherwise
 * */
template <typename Node,
          template <typename, typename = std::allocator<Node>> class Container>
bool shortest_path(Node n1, Node n2, Container<Node>& path) {
  n1.value().distance = 0;
  std::vector<Node> unvisited_node_queue{n1};
  unsigned queue_ptr = 0;

  while (queue_ptr < unvisited_node_queue.size()) {
    auto curr_node = unvisited_node_queue[queue_ptr];
    curr_node.value().visited = true;
    if (curr_node == n2)
      break;
    for (auto inc_iter = curr_node.edge_begin();
         inc_iter != curr_node.edge_end(); ++inc_iter) {
      if ((*inc_iter).node2().value().visited == false) {
        auto temp_node = (*inc_iter).node2();
        if (temp_node.value().distance > curr_node.value().distance + 1) {
          temp_node.value().distance = curr_node.value().distance + 1;
          temp_node.value().prev_node = curr_node;
        }
        // queued flag used to prevent pushing same node into queue multiple
        // times
        if (temp_node.value().queued == false) {
          temp_node.value().queued = true;
          unvisited_node_queue.push_back(temp_node);
        }
      }
    }
    ++queue_ptr;
  }

  if (n2.value().visited == false) {
    // clean up the flags
    for (auto node : unvisited_node_queue) {
      node.value().visited = false;
      node.value().queued = false;
    }
    return false;  // meaning n1 and n2 are not connected
  }
  while (n2.is_valid()) {
    path.push_back(n2);
    n2 = n2.value().prev_node;
  }
  for (auto node : unvisited_node_queue) {
    node.value().visited = false;
    node.value().queued = false;
  }
  return true;
}

/** Find bridge between node @a n1 and @a n2, in graph @a g, given n1 and n2 in
 * different subgraphs which are connected through a bridge
 * @return the bridge edge if found, an invalid edge otherwise
 */
template <typename Graph, typename Node>
typename Graph::edge_type find_bridge(const Graph& g, Node n1, Node n2) {
  std::vector<Node> path;
  if (shortest_path(n1, n2, path)) {
    for (auto n_it = path.rbegin(); n_it != path.rend() - 1; ++n_it)
      if (!is_path_not_direct(*n_it, *(n_it + 1)))
        return g.edge(*n_it, *(n_it + 1));
  }
  return g.nulledge();
}

int main(int argc, char** argv) {
  // Check arguments
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " NODES_FILE TETS_FILE\n";
    exit(1);
  }

  // Construct a Graph
  struct NodeData;
  typedef Graph<NodeData, int> GraphType;
  typedef typename GraphType::node_type NodeType;
  typedef typename GraphType::edge_type EdgeType;

  struct NodeData {
    bool visited{false};
    bool queued{false};
    int distance{std::numeric_limits<int>::max()};
    NodeType prev_node;
  };
  GraphType graph;
  std::vector<NodeType> nodes;

  // Create a nodes_file from the first input argument
  std::ifstream nodes_file(argv[1]);
  // Interpret each line of the nodes_file as a 3D Point and add to the Graph
  Point p;
  while (CS207::getline_parsed(nodes_file, p))
    nodes.push_back(graph.add_node(p));

  // Create a tets_file from the second input argument
  std::ifstream tets_file(argv[2]);
  // Interpret each line of the tets_file as four ints which refer to nodes
  std::array<int, 4> t;
  while (CS207::getline_parsed(tets_file, t))
    for (unsigned i = 1; i < t.size(); ++i)
      for (unsigned j = 0; j < i; ++j)
        graph.add_edge(nodes[t[i]], nodes[t[j]]);

  // Print out the stats
  std::cout << graph.num_nodes() << " " << graph.num_edges() << std::endl;

  auto node1 = graph.node(0);
  auto node2 = graph.node(graph.num_nodes() / 2);

  auto bridge = find_bridge(graph, node1, node2);
  if (bridge.is_valid()) {
    std::cout << "Bridge Found:\n";
    std::cout << "Node 1 index : " << bridge.node1().index()
              << "\tposition: " << bridge.node1().position() << "\n";
    std::cout << "Node 2 index : " << bridge.node2().index()
              << "\tposition: " << bridge.node2().position() << "\n";
  } else
    std::cout << "Bridge Not Found\n";

  // Launch the SDLViewer
  CS207::SDLViewer viewer;
  auto node_map = viewer.empty_node_map(graph);
  viewer.launch();

  viewer.add_nodes(graph.node_begin(), graph.node_end(), node_map);
  viewer.add_edges(graph.edge_begin(), graph.edge_end(), node_map);

  viewer.center_view();

  return 0;
}
