/**
 * @file mass_spring.cpp
 * Implementation of mass-spring system using Graph
 *
 * @brief Reads in two files specified on the command line.
 * First file: 3D Points (one per line) defined by three doubles
 * Second file: Tetrahedra (one per line) defined by 4 indices into the point
 * list
 */

#include <fstream>

#include "CS207/SDLViewer.hpp"
#include "CS207/Util.hpp"
#include "CS207/Color.hpp"

#include "Graph.hpp"
#include "Point.hpp"

// Gravity in meters/sec^2
static constexpr double grav = 9.81;

/** Custom structure of data to store with Nodes */
struct NodeData {
  Point velocity;  //< Node velocity
  double mass;     //< Node mass
};

// Spring constant K = 100;
static constexpr double spring_const = 100.0;

// Edge value stores the edge length
typedef Graph<NodeData, double> GraphType;
typedef typename GraphType::node_type Node;
typedef typename GraphType::edge_type Edge;

/** Change a graph's nodes according to a step of the symplectic Euler
 *    method with the given node force.
 * @param[in,out] g      Graph
 * @param[in]     t      The current time (useful for time-dependent forces)
 * @param[in]     dt     The time step
 * @param[in]     force  Function object defining the force per node
 * @param[in]     constraint  Constraint object defining the constraint on the
 *graph nodes
 * @return the next time step (usually @a t + @a dt)
 *
 * @tparam G::node_value_type supports ???????? YOU CHOOSE
 * @tparam F is a function object called as @a force(n, @a t),
 *           where n is a node of the graph and @a t is the current time.
 *           @a force must return a Point representing the force vector on Node
 *           at time @a t.
 * @tparam C is a function object called as @a constraint(g, @a t),
 *           where g is the graph by reference and @a t is the current time.
 *           @a constraint doesn't have a return value, but will make
 *corresponding changes to the graph nodes and edges,
 *			 including the value() at time @a t.
 */
template <typename G, typename F, typename C>
double symp_euler_step(G& g, double t, double dt, F force, C constraint) {
  // Compute the {n+1} node positions
  for (auto it = g.node_begin(); it != g.node_end(); ++it) {
    auto n = *it;
    if (n.position() != Point(0, 0, 0) && n.position() != Point(1, 0, 0)) {
      // Update the position of the node according to its velocity
      // x^{n+1} = x^{n} + v^{n} * dt
      n.position() += n.value().velocity * dt;
    }
  }

  // Compute the {n+1} node velocities
  for (auto it = g.node_begin(); it != g.node_end(); ++it) {
    auto n = *it;
    if (n.position() != Point(0, 0, 0) && n.position() != Point(1, 0, 0)) {
      // v^{n+1} = v^{n} + F(x^{n+1},t) * dt / m
      n.value().velocity += force(n, t) * (dt / n.value().mass);
    }
  }

  // Act on the graph nodes and edges defined by the @a constraint functor,
  // before the next loop
  constraint(g, t);
  return t + dt;
}

struct GravityForce {
  /** Return the gravity force being applied to @a n at time @a t.
   * Calculated using the mass of the current node and gravity constant
   */
  Point operator()(Node n, double) { return Point(0, 0, -grav * n.value().mass); }
};

struct MassSpringForce {
  /** Return the spring force being applied to @a n at time @a t.
   * Use the rest length of all the edges connected to the current node, and the
   * current positions of all the connected nodes
   */
  Point operator()(Node n, double) {
    Point force = Point(0, 0, 0);
    for (auto it = n.edge_begin(); it != n.edge_end(); ++it) {
      force -= spring_const * (n.position() - (*it).node2().position()) *
               ((*it).length() - (*it).value()) / (*it).length();
    }
    return force;
  }
};

struct DampingForce {
  /** Return the damping force being applied to @a n at time @a t.
   *
   * initialized with the number of nodes @a num_n
   * to calculate the spring force, also with the current node's velocity
   */
 private:
  unsigned num_nodes;

 public:
  DampingForce(int num_n) : num_nodes(num_n) {}
  Point operator()(Node n, double) {
    Point force = -1.0 / double(num_nodes) * n.value().velocity;
    return force;
  }
};

/**Partial template specialization
 * Primary template with 3 template input types
 *
 * MetaForce takes in 3 template types
 * @return combined forces on the same node @a n, at time @a t
 */
template <typename F1, typename F2, typename F3 = void>
struct MetaForce {
 private:
  F1 f1_;
  F2 f2_;
  F3 f3_;

 public:
  MetaForce(F1 f1, F2 f2, F3 f3) : f1_(f1), f2_(f2), f3_(f3) {}
  Point operator()(Node n, double t) { return f1_(n, t) + f2_(n, t) + f3_(n, t); }
};

/**Partial template specialization
 * Primary template with 2 template input types
 */
template <typename F1, typename F2>
struct MetaForce<F1, F2, void> {
 private:
  F1 f1_;
  F2 f2_;

 public:
  MetaForce(F1 f1, F2 f2) : f1_(f1), f2_(f2) {}
  Point operator()(Node n, double t) { return f1_(n, t) + f2_(n, t); }
};

template <typename F1, typename F2, typename F3>
MetaForce<F1, F2, F3> make_combined_force(F1 f1, F2 f2, F3 f3) {
  return MetaForce<F1, F2, F3>(f1, f2, f3);
}

template <typename F1, typename F2>
MetaForce<F1, F2> make_combined_force(F1 f1, F2 f2) {
  return MetaForce<F1, F2>(f1, f2);
}

struct Problem1Force {
  /** Return the force being applied to @a n at time @a t.
   *
   * For HW2 #1, this is a combination of mass-spring force and gravity,
   * except that points at (0, 0, 0) and (1, 0, 0) never move. We can
   * model that by returning a zero-valued force. */
  Point operator()(Node n, double) {
    Point force = Point(0, 0, 0);
    // static double rest_length = (*(n.edge_begin())).length();
    if (n.position() == Point(0, 0, 0) || n.position() == Point(1, 0, 0)) return force;
    for (auto it = n.edge_begin(); it != n.edge_end(); ++it) {
      force -= spring_const * (n.position() - (*it).node2().position()) *
               ((*it).length() - (*it).value()) / (*it).length();
      /*
      std::cout << "rest_length : " << (*it).value() << "\n";
      std::cout << "Node1 : " << (*it).node1().index() << "\tPosition : " <<
      (*it).node1().position() << "\n";
      std::cout << "Node2 : " << (*it).node2().index() << "\tPosition : " <<
      (*it).node2().position() << "\n";
        CS207::sleep(1);
        */
    }
    force += Point(0, 0, -grav * n.value().mass);  // gravity force
    return force;
  }
};

struct PlaneConstraint {
  /** Take the current graph by reference and a unused time variable
   * Reset the positions and velocity of the nodes that fall under the plane
   * z==-0.75
   */
  void operator()(GraphType& g, double) {
    for (auto ni = g.node_begin(); ni != g.node_end(); ++ni) {
      if (inner_prod((*ni).position(), Point(0, 0, 1)) < -0.75) {
        (*ni).position().z = -0.75;
        (*ni).value().velocity.z = 0;
      }
    }
  }
};

struct SphereConstraint {
  /** Take the current graph by reference and a unused time variable
   * Reset the positions and velocity of the nodes that fall into the sphere
   * sphere center is at (0.5, 0.5, -0.5), radius is 0.15
   */
  void operator()(GraphType& g, double) {
    Point c = Point(0.5, 0.5, -0.5);
    for (auto ni = g.node_begin(); ni != g.node_end(); ++ni) {
      if (norm((*ni).position() - c) < 0.15) {
        Point R = ((*ni).position() - c) / norm((*ni).position() - c);
        Point v = (*ni).value().velocity;
        (*ni).position() = 0.15 * R + c;
        (*ni).value().velocity = v - inner_prod(v, R) * R;
      }
    }
  }
};

struct RemoveNodeSphereC {
  /** Takes the current graph by reference and an unused time variable
   * Doesn't have a return value
   * Use node_iterator to find the nodes that violate the sphere position
   *constraint
   *
   * @post Remove the nodes and connected edges that violate the constraint
   */
  void operator()(GraphType& g, double) {
    Point c = Point(0.5, 0.5, -0.5);
    auto ni = g.node_begin();
    while (ni != g.node_end()) {
      if (norm((*ni).position() - c) < 0.15)
        ni = g.remove_node(ni);
      else
        ++ni;
    }
  }
};

/** MetaConstraint takes 2 Constraint types, and act on the graph in succession
 */
template <typename C1, typename C2>
struct MetaConstraint {
  C1 c1;
  C2 c2;
  void operator()(GraphType& g, double t) {
    c1(g, t);
    c2(g, t);
  }
};

template <typename C1, typename C2>
MetaConstraint<C1, C2> make_combined_constraint(C1, C2) {
  return MetaConstraint<C1, C2>();
}

int main(int argc, char** argv) {
  // Check arguments
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " NODES_FILE TETS_FILE\n";
    exit(1);
  }

  // Construct a graph
  GraphType graph;

  // Create a nodes_file from the first input argument
  std::ifstream nodes_file(argv[1]);
  // Interpret each line of the nodes_file as a 3D Point and add to the Graph
  std::vector<Node> nodes;
  Point p;
  while (CS207::getline_parsed(nodes_file, p)) nodes.push_back(graph.add_node(p));

  // Create a tets_file from the second input argument
  std::ifstream tets_file(argv[2]);
  // Interpret each line of the tets_file as four ints which refer to nodes
  std::array<int, 4> t;
  while (CS207::getline_parsed(tets_file, t)) {
    for (unsigned i = 1; i < t.size(); ++i) {
      graph.add_edge(nodes[t[0]], nodes[t[1]]);
      graph.add_edge(nodes[t[0]], nodes[t[2]]);

      // Diagonal edges: include as of HW2 #2
      graph.add_edge(nodes[t[0]], nodes[t[3]]);
      graph.add_edge(nodes[t[1]], nodes[t[2]]);

      graph.add_edge(nodes[t[1]], nodes[t[3]]);
      graph.add_edge(nodes[t[2]], nodes[t[3]]);
    }
  }

  // Set initial conditions for the edge length
  for (auto it = graph.edge_begin(); it != graph.edge_end(); ++it) (*it).value() = (*it).length();

  double node_mass = 1.0 / graph.num_nodes();
  for (auto it = graph.node_begin(); it != graph.node_end(); ++it) {
    (*it).value().velocity = Point(0, 0, 0);
    (*it).value().mass = node_mass;
  }

  // Print out the stats
  std::cout << graph.num_nodes() << " " << graph.num_edges() << std::endl;

  // Launch the SDLViewer
  CS207::SDLViewer viewer;
  auto node_map = viewer.empty_node_map(graph);
  viewer.launch();

  viewer.add_nodes(graph.node_begin(), graph.node_end(), node_map);
  viewer.add_edges(graph.edge_begin(), graph.edge_end(), node_map);

  viewer.center_view();

  // Begin the mass-spring simulation
  double dt = 0.0005;
  double t_start = 0.0;
  double t_end = 5.0;

  for (double t = t_start; t < t_end; t += dt) {
    // std::cout << "t = " << t << std::endl;
    // symp_euler_step(graph, t, dt, Problem1Force());
    DampingForce f_damp(graph.num_nodes());

    // Two arguments test
    //	auto combined_force = make_combined_force(GravityForce(),
    // MassSpringForce());
    // Three arguments test
    auto combined_force = make_combined_force(GravityForce(), MassSpringForce(), f_damp);
    // auto combined_constraint =
    //   make_combined_constraint(SphereConstraint(), PlaneConstraint());
    auto combined_constraint = make_combined_constraint(RemoveNodeSphereC(), PlaneConstraint());
    symp_euler_step(graph, t, dt, combined_force, combined_constraint);

    // Update viewer with nodes' new positions
    viewer.clear();
    node_map.clear();
    viewer.add_nodes(graph.node_begin(), graph.node_end(), node_map);
    viewer.add_edges(graph.edge_begin(), graph.edge_end(), node_map);
    viewer.set_label(t);

    // These lines slow down the animation for small graphs, like grid0_*.
    // Feel free to remove them or tweak the constants.
    if (graph.size() < 100) CS207::sleep(0.01);
  }

  return 0;
}
