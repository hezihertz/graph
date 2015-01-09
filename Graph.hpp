#ifndef CS207_GRAPH_HPP
#define CS207_GRAPH_HPP

/** @file Graph.hpp
 * @brief An undirected graph type
 */

#include <algorithm>
#include <vector>
#include <cassert>
#include <array>
#include <iostream>

#include "CS207/Util.hpp"
#include "Point.hpp"

/** @class Graph
 * @brief A template for 3D undirected graphs.
 *
 * Users can add and retrieve nodes and edges. Edges are unique (there is at
 * most one edge between any pair of distinct nodes).
 */
template <typename V, typename E>
class Graph {
 private:
  struct node_element;
  struct edge_element;

 public:
  /////////////////////////////
  // PUBLIC TYPE DEFINITIONS //
  /////////////////////////////

  /** Type of this graph. */
  typedef Graph graph_type;
  typedef V node_value_type;
  typedef E edge_value_type;

  /** Predeclaration of Node type. */
  class Node;
  /** Synonym for Node (following STL conventions). */
  typedef Node node_type;

  /** Predeclaration of Edge type. */
  class Edge;
  /** Synonym for Edge (following STL conventions). */
  typedef Edge edge_type;

  /** Type of indexes and sizes.
    Return type of Graph::Node::index(), Graph::num_nodes(),
    Graph::num_edges(), and argument type of Graph::node(size_type) */
  typedef unsigned size_type;

  /** Type of node iterators, which iterate over all graph nodes. */
  class NodeIterator;
  /** Synonym for NodeIterator */
  typedef NodeIterator node_iterator;

  /** Type of edge iterators, which iterate over all graph edges. */
  class EdgeIterator;
  /** Synonym for EdgeIterator */
  typedef EdgeIterator edge_iterator;

  /** Type of incident iterators, which iterate incident edges to a node. */
  class IncidentIterator;
  /** Synonym for IncidentIterator */
  typedef IncidentIterator incident_iterator;

  ////////////////////////////////
  // CONSTRUCTOR AND DESTRUCTOR //
  ////////////////////////////////

  /** Construct an empty graph. */
  Graph() : nodes_(), i2u_(), edges_() {}
  /** Default destructor */
  ~Graph() = default;

  /////////////
  // General //
  /////////////

  /** Return the number of nodes in the graph.
  *
  * Complexity: O(1).
  */
  size_type size() const { return i2u_.size(); }

  /** Remove all nodes and edges from this graph.
  * @post num_nodes() == 0 && num_edges() == 0
  *
  * Invalidates all outstanding Node and Edge objects, plus the index to node
  *relationship.
  */
  void clear() {
    nodes_.clear();
    i2u_.clear();
    edges_.clear();
  }

  /////////////////
  // GRAPH NODES //
  /////////////////

  /** @class Graph::Node
  * @brief Class representing the graph's nodes.
  *
  * Node objects are used to access information about the Graph's nodes.
  */
  class Node : private totally_ordered<Node> {
   public:
    /** Construct an invalid node.
     *
     * Valid nodes are obtained from the Graph class, but it
     * is occasionally useful to declare an @i invalid node, and assign a
     * valid node to it later. For example:
     *
     * @code
     * Graph::node_type x;
     * if (...should pick the first node...)
     *   x = graph.node(0);
     * else
     *   x = some other node using a complicated calculation
     * do_something(x);
     * @endcode
     */
    Node() {}

    /** Return this node's position. */
    Point& position() { return g_->nodes_[uid_].p_; }

    const Point& position() const {
      return const_cast<const Point&>(g_->nodes_[uid_].p_);
    }

    /** Return this node's index, a number in the range [0, graph_size). */
    size_type index() const { return g_->nodes_[uid_].idx_; }

    /** Test whether this node and @a x are equal.
    *
    * Equal nodes have the same graph and the same index.
    */
    bool operator==(const Node& x) const {
      if (g_ != x.g_ || uid_ != x.uid_) return false;
      return true;
    }

    /** Test whether this node is less than @a x in the global order.
     *
     * This ordering function is useful for STL containers such as
     * std::map<>. It need not have any geometric meaning.
     *
     * The node ordering relation must obey trichotomy: For any two nodes x
     * and y, exactly one of x == y, x < y, and y < x is true.
     */
    bool operator<(const Node& x) const {
      if (g_ < x.g_) return true;
      if (g_ > x.g_) return false;
      if (uid_ < x.uid_) return true;
      return false;
    }

    /** Return the value() of the current node, using @a uid_
     *
     * Complexity: O(1)
     */
    node_value_type& value() { return g_->nodes_[uid_].v_; }

    const node_value_type& value() const {
      return const_cast<const node_value_type&>(g_->nodes_[uid_].v_);
    }

    // returns the number of nodes, or the number of edges, the current node is
    // connected to
    size_type degree() const { return g_->edges_[uid_].size(); }

    /** incident_iterator starts with current node being node1, and the second
     * index starting from 0 in the representation of edges_ 2nd dimension
     */
    incident_iterator edge_begin() const {
      return g_->make_incident_iterator(uid_, 0);
    }

    /** edge_end() is represented by the construction of an invalid
     * incident_iterator, where the second node uid is just out of range
     * Because in the representation of @a edges_, the 2nd dimension of vector
     * is continuously indexed
     * therefore, for all i, when i>=0 and i<g_->edges_[uid_].size(), @a
     * g_->edges_[uid_][i] is valid edge
     */
    incident_iterator edge_end() const {
      return g_->make_incident_iterator(uid_, g_->edges_[uid_].size());
    }

		bool is_valid() const {
			return (g_!=nullptr && uid_<g_->nodes_.size());
		}

   private:
    friend class Graph;
    Graph* g_;
    size_type uid_;  // Stores the node uid

    Node(const Graph* g, size_type uid)
        : g_(const_cast<Graph*>(g)), uid_(uid) {}
  };

  /** Synonym for size(). */
  size_type num_nodes() const { return size(); }

  /** Add a node to the graph, returning the added node.
  * @param[in] position The new node's position
  * @post new size() == old size() + 1
  * @post result_node.index() == old size()
  * @post new @a edges_.size() == old @a edges_.size() + 1, including a new
  *empty vector for the edges the new node will be connected to
  *
  * Complexity: O(1) amortized operations.
  */
  Node add_node(const Point& position,
                const node_value_type& node_value = node_value_type()) {
    size_type new_idx = i2u_.size();
    size_type new_uid = nodes_.size();
    nodes_.push_back(node_element(new_idx, position, node_value));
    i2u_.push_back(new_uid);
    return Node(this, new_uid);
  }

  /** Remove node from graph, return success indicator.
  * @param[in] @a n, node to be removed
  * @post when removal operation is successful, return value == 1 && new @a
  *size() == old @a size() - 1
  * @post when current graph doesn't have the node @a n, return value == 0 &&
  *new @a size() == old size()
  *
  * @post @a nodes_[uid_].deleted_ == true, meaning this node has been deleted
  * @post @a for all i, 0<=i<nodes_[n.uid_].idx_, new @a i2u_[i] == old @a
  *i2u_[i]
  * 		and for all i, nodes_[n.uid_].idx_ + 1 < i < old @a i2u_.size(),
  *new @a i2u_[i-1] == old @a i2u_[i]
  * @post new @a i2u_.size() == old @a i2u_.size() - 1
  * node_iterator will skip this node
  *
  * Complexity: O(num_nodes()^2)
  */
  size_type remove_node(const Node& n) {
    if (!has_node(n)) return 0;
    size_type curr_n1id = n.uid_;
    for (size_type i = 0; i < i2u_.size(); ++i) {
      // meaning this node still exists, remove the node
      if (i2u_[i] == curr_n1id) {
        // Remove all related edges first
        auto ii = n.edge_begin();
        while (ii != n.edge_end()) {
          ii = remove_edge(ii);
        }

        // Remove this node
        nodes_[i2u_[i]].deleted_ = true;
        for (size_type j = i + 1; j < i2u_.size(); ++j) {
          --(nodes_[i2u_[j]].idx_);
          i2u_[j - 1] = i2u_[j];
        }
        i2u_.pop_back();
      }
    }
    return 1;
  }

  /** Remove node from graph, using node_iterator.
   * @param[in] @a n_it, node to be removed, indicated by the node iterator @a
   *n_it
   * return node_iterator == old @a ++n_it
   *
   * @post With the removal of the node, @a (*n_it)
   * every node @a n.index() > old @a (*n_it).index()
   * new @a n.index() = old @a n.index() - 1
   *
   * And since node_iterator @a n_it is represented by the node index, @a n_it
   *automatically points to the next node
   * If the last node is removed, the returned @a n_it is invalid, and @a n_it
   *== node_end()
   *
   * Complexity: O(num_nodes()^2)
   */
  node_iterator remove_node(node_iterator n_it) {
    Node n = (*n_it);
    remove_node(n);
    return n_it;
  }

  /** Determine if this Node @n belongs to this Graph
   * @return True if @a n is currently a Node of this Graph
   *
   * Complexity: O(n).
   */
  bool has_node(const Node& n) const {
		if (this != n.g_) return false;
    for (size_type i = 0; i < i2u_.size(); ++i)
      if (n.uid_ == i2u_[i]) return true;
    return false;
  }

  /** Return the node with index @a i.
   * @pre 0 <= @a i < num_nodes()
   * @post result_node.index() == i
   *
   * Complexity: O(1).
   */
  Node node(size_type i) const { return Node(this, i2u_[i]); }

	Node nullnode() const {
		return Node(nullptr, 0);
	}
  /////////////////
  // GRAPH EDGES //
  /////////////////

  /** @class Graph::Edge
  * @brief Class representing the graph's edges.
  *
  * Edges are order-insensitive pairs of nodes. Two Edges with the same nodes
  * are considered equal if they connect the same nodes, in either order.
  */
  class Edge : private totally_ordered<Edge> {
   public:
    Edge() {}

    /** Return a node of this Edge */
    Node node1() const { return g_->node(g_->nodes_[n1_uid_].idx_); }

    /** Return the other node of this Edge */
    Node node2() const {
      return g_->node(
          g_->nodes_[g_->edges_[n1_uid_][node2_edge_idx_].nid_].idx_);
    }

    /** Test whether this edge and @a x are equal.
     *
     * Equal edges are from the same graph and have the same nodes.
     */
    bool operator==(const Edge& x) const {
      if ((g_ == x.g_) &&
          ((n1_uid_ == x.n1_uid_) && (node2_edge_idx_ == x.node2_edge_idx_)))
        return true;
      return false;
    }

    /** Test whether this edge is less than @a x in the global order.
     *
     * This ordering function is useful for STL containers such as
     * std::map<>. It need not have any geometric meaning.
     *
     * The edge ordering relation must obey trichotomy: For any two edges x
     * and y, exactly one of x == y, x < y, and y < x is true.
     */
    bool operator<(const Edge& x) const {
      if (g_ < x.g_) return true;
      if (g_ > x.g_) return false;

      if (n1_uid_ < x.n1_uid_) return true;
      if ((n1_uid_ == x.n1_uid_) && (node2_edge_idx_ < x.node2_edge_idx_))
        return true;
      return false;
    }

    /** Return the current length of the edge, with the @a norm() function from
    * Point class
    * Get the 2 nodes' positions with the help of n1_uid_ and node2_edge_idx_
    */
    double length() const {
      Point p1 = (g_->node(g_->nodes_[n1_uid_].idx_)).position();
      size_type n2id = g_->edges_[n1_uid_][node2_edge_idx_].nid_;
      //			std::cout << "n2id : " << n2id << std::endl;
      Point p2 = (g_->node(g_->nodes_[n2id].idx_)).position();
      return norm(p1 - p2);
    }

    /** Return the @a value() of the current edge
    * Need to loop through all the edges connected to @a node1(), to get the
    *location of @a node2() inside @a edges_[n1_uid_]
    *
    * Complexity: O(num_edges())
    */
    edge_value_type& value() {
      return g_->edge_values_[g_->edges_[n1_uid_][node2_edge_idx_].ev_idx_];
    }

    const edge_value_type& value() const {
      return const_cast<const edge_value_type&>(value());
    }

		bool is_valid() const {
			return (g_!=nullptr && (n1_uid_<g_->nodes_.size() && node2_edge_idx_<g_->edges_[n1_uid_].size()));
		}

   private:
    friend class Graph;
    /** RI: while the edge_iterator is valid, g_ != nullptr,
    *n1id_<g_->edges_.size(), n2id_<g_->edges_[n1id_].size()
    * while it's invalid, i.e., reaches the end g_ != nullptr, n1id_ ==
    *g_->edges_.size(), n2id_ < g_->edges_[n1id_].size()
    *
    * Edge representation: graph pointer, node1().uid_ and node2().uid_
    */
    Graph* g_;
    size_type n1_uid_;
    size_type node2_edge_idx_;

    Edge(const Graph* g, size_type n1_uid, size_type node2_edge_idx)
        : g_(const_cast<Graph*>(g)),
          n1_uid_(n1_uid),
          node2_edge_idx_(node2_edge_idx) {}
  };

  /** Return the total number of edges in the graph.
  * Loop through the first dimension of @a edges_ vector, i.e., all the nodes,
  *to get every 2nd dimension's size
  * Because every edge is duplicated in this representation, the returned number
  *of edges should be divided by 2 at the end
  *
  * Complexity: O(num_nodes())
  */
  size_type num_edges() const {
    size_type total_edges = 0;
    for (size_type i = 0; i < edges_.size(); ++i)
      total_edges += edges_[i].size();
    return total_edges / 2;
  }

  /** Add an edge to the graph, or return the current edge if it already exists.
  * @pre @a a and @a b are distinct valid nodes of this graph
  * @return an Edge object e with e.node1() == @a a and e.node2() == @a b
  * @post has_edge(@a a, @a b) == true
  * @post If old has_edge(@a a, @a b), new num_edges() == old num_edges().
  *       Else,                        new num_edges() == old num_edges() + 1.
  *
  * Can invalidate edge indexes -- in other words, old edge(@a i) might not
  * equal new edge(@a i). Must not invalidate outstanding Edge objects.
  *
  * Complexity: O(num_nodes() + num_edges()), same as @a has_edge()
  */
  Edge add_edge(const Node& a, const Node& b) {
    if (a == b) return nulledge();
    if (edges_.size() < nodes_.size()) edges_.resize(nodes_.size());
    int node2_edge_idx = has_edge_return_index(a, b);
    if (node2_edge_idx != -1) return Edge(this, a.uid_, node2_edge_idx);
    // Current edge doesn't exist, push back a new edge and return it
    size_type ev_idx = edge_values_.size();
    auto new_edge1 = edge_element(b.uid_, ev_idx);
    auto new_edge2 = edge_element(a.uid_, ev_idx);
    node2_edge_idx = edges_[a.uid_].size();
    edges_[a.uid_].push_back(new_edge1);
    edges_[b.uid_].push_back(
        new_edge2);  // store duplicates incident iterator and node.degree()
    edge_values_.push_back(edge_value_type());
    return Edge(this, a.uid_, node2_edge_idx);
  }

  /** Test whether two nodes are connected by an edge.
  * @pre @a a and @a b are valid nodes of this graph
  * @return true if, for some @a i, edge(@a i) connects @a a and @a b.
  *
  * Complexity: O(num_nodes() + num_edges())
  */
  bool has_edge(const Node& a, const Node& b) const {
    if (!has_node(a) || !has_node(b)) return false;
    for (auto it = edges_[a.uid_].begin(); it != edges_[a.uid_].end(); ++it)
      if ((*it).nid_ == b.uid_) return true;
    return false;
  }

  int has_edge_return_index(const Node& a, const Node& b) const {
    if (!has_node(a) || !has_node(b)) return -1;
    for (unsigned i = 0; i < edges_[a.uid_].size(); ++i)
      if (edges_[a.uid_][i].nid_ == b.uid_) return i;
    return -1;
  }

  /** Return the edge with index @a i.
  * @pre 0 <= @a i < num_edges()
  *
  * Complexity: O(num_nodes() + num_edges())
  */

  Edge edge(size_type i) const {
    size_type curr_i = i;
    for (size_type j = 0; j < edges_.size(); ++j) {
      for (size_type k = 0; k < edges_[j].size(); ++k) {
        if (edges_[j][k].nid_ > j) {
          if (curr_i == 0)
            return Edge(this, j, edges_[j][k].nid_);
          else
            --curr_i;
        }
      }
    }
    return Edge();
  }

	Edge edge(const Node& a, const Node& b) const {
    int node2_edge_idx = has_edge_return_index(a, b);
    if (node2_edge_idx == -1) return nulledge();
		return Edge(this, a.uid_, node2_edge_idx);
	}

	Edge nulledge() const {
		return Edge(nullptr, 0, 0);
	}
  /** Remove edge from graph from an iterator_type, return an edge_iterator.
  * @param[in] EdgeIterator @a e_it, points to the edge to be removed
  * @post if Edge exists, old @a has_edge((*e_it).node1(), (*e_it).node2()) ==
  * true, removal operation is successful, return value == 1 && new @a
  * has_edge((*e_it).node1(), (*e_it).node2()) == false
  * @post if Edge doesn't exist, removal operation failed, return value == 0
  */
  edge_iterator remove_edge(edge_iterator& e_it) {
    Node n1 = (*e_it).node1();
    Node n2 = (*e_it).node2();
    remove_edge(n1, n2);
    e_it.fix();
    return e_it;
  }

  incident_iterator remove_edge(incident_iterator& e_it) {
    Node n1 = (*e_it).node1();
    Node n2 = (*e_it).node2();
    remove_edge(n1, n2);
    e_it.fix();
    return e_it;
  }

  /** Remove edge from graph from 2 input nodes, return success indicator.
  * @param[in] nodes @a n1, @a n2 represent the edge to be removed
  * @post if edge exists, old @a has_edge(n1, n2) == true, removal operation is
  *successful, return value == 1 && new @a has_edge(n1, n2) == false
  * @post if edge doesn't exist, old @a has_edge(n1, n2) == false, removal
  *operation failed, return value == 0
  *
  * Complexity: O(n1.degree() + n2.degree())
  */
  size_type remove_edge(const Node& n1, const Node& n2) {
    // edge_iterator constructor takes in graph pointer @a g_, uid of node1, and
    // index of node2
    if (!has_node(n1) || !has_node(n2) || !has_edge(n1, n2)) return 0;
    for (size_type i = 0; i < edges_[n1.uid_].size(); ++i) {
      if (n2.uid_ == edges_[n1.uid_][i].nid_) {
        for (size_type j = i + 1; j < edges_[n1.uid_].size(); ++j) {
          edges_[n1.uid_][j - 1] = edges_[n1.uid_][j];
        }
        edges_[n1.uid_].pop_back();
      }
    }
    for (size_type i = 0; i < edges_[n2.uid_].size(); ++i) {
      if (n1.uid_ == edges_[n2.uid_][i].nid_) {
        for (size_type j = i + 1; j < edges_[n2.uid_].size(); ++j)
          edges_[n2.uid_][j - 1] = edges_[n2.uid_][j];
        edges_[n2.uid_].pop_back();
      }
    }
    return 1;
  }

  /** Remove edge from graph from an input edge_type, return success indicator.
  * @param[in] Edge @a e, represents the edge to be removed
  * @post if Edge exists, old @a has_edge(e.node1(), e.node2()) == true, removal
  * operation is successful, return value == 1 && new @a has_edge(e.node1(),
  * e.node2()) == false
  * @post if Edge doesn't exist, removal operation failed, return value == 0
  */
  size_type remove_edge(const Edge& e) {
    return remove_edge(e.node1(), e.node2());
  }

  ///////////////
  // Iterators //
  ///////////////

  /** @class Graph::NodeIterator
  * @brief Iterator class for nodes. A forward iterator. */
  class NodeIterator : private totally_ordered<NodeIterator> {
   public:
    // These type definitions help us use STL's iterator_traits.
    /** Element type. */
    typedef Node value_type;
    /** Type of pointers to elements. */
    typedef Node* pointer;
    /** Type of references to elements. */
    typedef Node& reference;
    /** Iterator category. */
    typedef std::input_iterator_tag iterator_category;
    /** Difference between iterators */
    typedef std::ptrdiff_t difference_type;

    /** Construct an invalid NodeIterator. */
    NodeIterator() {}

    /** Operator* dereferences the iterator type and returns the value it points
     * to, cannot modify this value
     * Use Node constructor, constructed with node uid
     */
    Node operator*() const { return Node(g_, g_->i2u_[idx_]); }

    /** incrementing operator
     * increase the @a idx_ to the next node's index
     * Need to check on its validity with @a fix()
     */
    NodeIterator& operator++() {
      ++idx_;
      fix();
      return *this;
    }

    bool operator==(const NodeIterator& other) const {
      return ((g_ == other.g_) && (idx_ == other.idx_));
    }

   private:
    friend class Graph;
    Graph* g_;
    size_type idx_;  // Use node index for the node_iterator, to iterator
                     // through all currently existing nodes

    // fix() function to help skip the node that's already been removed, or the
    // invalid @a idx_
    void fix() {
      if (idx_ > g_->i2u_.size()) {
        idx_ = g_->i2u_.size();
        return;
      }
      while (idx_ < g_->i2u_.size()) {
        if (g_->nodes_[g_->i2u_[idx_]].deleted_ == false)
          return;
        else
          ++idx_;
      }
    }

    /** Valid NodeIterator constructor
     * Has to make sure the constructed NodeIterator is valid, with the help of
     * @a fix() function
     */

    NodeIterator(const Graph* g, size_type idx) {
      g_ = const_cast<Graph*>(g);
      idx_ = idx;
      fix();
    }
  };

  /** node_iterator starts with @a index() == 0
  */
  node_iterator node_begin() const { return node_iterator(this, 0); }

  /** node_iterator ends with @a index() == i2u_.size() == num_nodes()
  */
  node_iterator node_end() const { return node_iterator(this, i2u_.size()); }

  /** @class Graph::EdgeIterator
  * @brief Iterator class for edges. A forward iterator. */
  class EdgeIterator : private totally_ordered<EdgeIterator> {
   public:
    // These type definitions help us use STL's iterator_traits.
    /** Element type. */
    typedef Edge value_type;
    /** Type of pointers to elements. */
    typedef Edge* pointer;
    /** Type of references to elements. */
    typedef Edge& reference;
    /** Iterator category. */
    typedef std::input_iterator_tag iterator_category;
    /** Difference between iterators */
    typedef std::ptrdiff_t difference_type;

    /** Construct an invalid EdgeIterator. */
    EdgeIterator() {}

    /** Dereference operator
     * @return an Edge type
     * construct the Edge with n1_uid_ and node2_edge_idx_ with the help of @a
     * n2_idx_ and @a edges_
     */
    Edge operator*() const { return Edge(g_, n1_uid_, n2_idx_); }

    /** Incrementing operator
     * Make sure it's valid with @a fix() function
     */
    EdgeIterator& operator++() {
      ++n2_idx_;
      fix();
      return *this;
    }

    bool operator==(const EdgeIterator& other) const {
      return ((g_ == other.g_) &&
              ((n1_uid_ == other.n1_uid_) && (n2_idx_ == other.n2_idx_)));
    }

   private:
    friend class Graph;
    /** Representation of EdgeIterator:
     * graph pointer
     * node1 uid
     * node2's represented index in the edges_[n1_uid] vector, NOTE: NOT NODE
     * idx_ IN ITS OWN REPRESENTATION
     */
    Graph* g_;
    size_type n1_uid_;
    size_type n2_idx_;  // this node2 index is the index inside the @a
                        // edges_[n1_uid_], different from the @a idx_ element
                        // in the @a node_element

    // fix an invalid EdgeIterator (where node1().index()>node2().index(), by
    // moving the edge_iterator pointer forward
    void fix() {
      if (n1_uid_ >= g_->edges_.size()) {
        n1_uid_ = g_->edges_.size();
        n2_idx_ = 0;
        return;
      }
      while (n1_uid_ < g_->edges_.size()) {
        if (n2_idx_ >= (g_->edges_[n1_uid_]).size()) {
          n2_idx_ = 0;
          ++n1_uid_;
        }
        while ((n1_uid_ < g_->edges_.size()) &&
               n2_idx_ < (g_->edges_[n1_uid_]).size()) {
          if ((g_->edges_[n1_uid_][n2_idx_].nid_ > n1_uid_)) return;
          ++n2_idx_;
        }
      }
    }

    // Valid EdgeIterator constructor, make sure it's valid with @a fix()
    // function
    EdgeIterator(const Graph* g, size_type n1_uid, size_type n2_idx) {
      g_ = const_cast<Graph*>(g);
      n1_uid_ = n1_uid;
      n2_idx_ = n2_idx;
      fix();
    }
  };

  /** edge_iterator starts with n1_uid_ == 0, and n2_idx_ == 0, NOTE: THIS IS @a
   * edges_ REPRESENTATION OF INDEX, NOT @a nodes_
   */
  edge_iterator edge_begin() const { return edge_iterator(this, 0, 0); }

  /** edge_iterator ends with n1_uid_ == edges_size(), n2_idx == 0, where the
   * n1_uid_ just passes the last valid n1_uid_
   */
  edge_iterator edge_end() const {
    return edge_iterator(this, edges_.size(), 0);
  }

  /** @class Graph::IncidentIterator
  * @brief Iterator class for edges incident to a node. A forward iterator. */
  class IncidentIterator : private totally_ordered<IncidentIterator> {
   public:
    // These type definitions help us use STL's iterator_traits.
    /** Element type. */
    typedef Edge value_type;
    /** Type of pointers to elements. */
    typedef Edge* pointer;
    /** Type of references to elements. */
    typedef Edge& reference;
    /** Iterator category. */
    typedef std::input_iterator_tag iterator_category;
    /** Difference between iterators */
    typedef std::ptrdiff_t difference_type;

    /** Construct an invalid IncidentIterator. */
    IncidentIterator() {}

    Edge operator*() const { return Edge(g_, n1_uid_, n2_idx_); }

    IncidentIterator& operator++() {
      ++n2_idx_;
      fix();
      return *this;
    }

    bool operator==(const IncidentIterator& other) const {
      return ((g_ == other.g_) &&
              ((n1_uid_ == other.n1_uid_) && (n2_idx_ == other.n2_idx_)));
    }

   private:
    /** RI: while the incident_iterator is valid, g_ != nullptr,
    *n1_uid_<g_->edges_.size(), n2_idx_<g_->edges_[n1_uid_].size()
    * while it's invalid, i.e., reaches the end, g_ != nullptr,
    *n1_uid_<g_->edges_.size(), n2_idx_ == g_->edges_[n1_uid_].size()
    *
    * Same representation as EdgeIterator
    */
    friend class Graph;
    Graph* g_;
    size_type n1_uid_;
    size_type n2_idx_;

    void fix() {
      if (n2_idx_ > g_->edges_[n1_uid_].size())
        n2_idx_ = g_->edges_[n1_uid_].size();
    }

    IncidentIterator(const Graph* g, size_type n1_uid, size_type n2_idx) {
      g_ = const_cast<Graph*>(g);
      n1_uid_ = n1_uid;
      n2_idx_ = n2_idx;  // this node2 index is the index inside the @a
                         // edges_[n1_uid_], different from the @a idx_ element
                         // in the @a node_element
      fix();
    }
  };

  // Helper function to construct an incident_iterator for Node class
  incident_iterator make_incident_iterator(size_type id1, size_type id2) const {
    return incident_iterator(this, id1, id2);
  }

 private:
  // Use this space for your Graph class's internals:
  //   helper functions, data members, and so forth.
  struct node_element {
    size_type idx_;  // idx_ stores the index of the node element, not uid
    Point p_;
    node_value_type v_;
    bool deleted_;

    node_element(size_type idx, Point p, const node_value_type& v,
                 bool deleted = false)
        : idx_(idx), p_(p), v_(v), deleted_(deleted) {}
    node_element& operator=(const node_element& other) {
      idx_ = other.idx_;
      p_ = other.p_;
      v_ = other.v_;
      deleted_ = other.deleted_;
      return *this;
    }
  };

  struct edge_element {
    size_type nid_;  //@a nid_ represents the node id of @a node2()
    size_type ev_idx_;  // edge value index

    edge_element(size_type nid, size_type ev_idx)
        : nid_(nid), ev_idx_(ev_idx) {}
    edge_element& operator=(const edge_element& other) {
      nid_ = other.nid_;
      ev_idx_ = other.ev_idx_;
      return *this;
    }
  };

  // @a nodes_ is indexed by node uid, not node index
  std::vector<node_element> nodes_;
  // @a i2u_ stores the node index to node uid relationship, the value in i2u_
  // is ordered
  std::vector<size_type> i2u_;

  /** Adjancency list for edges, stores all node uid's
   * First dimension is indexed by @a node1().uid_, after nodes are removed,
   * certain vector might become empty
   * Second dimension is continuous, storing @a node2().uid_
   */
  std::vector<std::vector<edge_element>> edges_;
  std::vector<edge_value_type> edge_values_;
};

#endif
