
#ifndef ASTAR_H
#define ASTAR_H

// a-star implemenation
// based on tute at http://web.mit.edu/eranki/www/tutorials/search/

#include <queue>
#include <functional>
#include <stdexcept>
#include <set>
#include <cassert>


#include <unordered_set>

#ifdef DEBUG
#include <iostream>
#endif

namespace astar {


template<typename T>
class node;

// cost of a node, f, is given by:
//    f = g + h
// where g is the cost it took to _get_ to the node
// (i.e., 1 + the predecessor cost)
// and h is our estimated distance to the goal.
//    Unhelpful names, ikr, but they seem standard
// for a-star implementations.

// we use a simple model where incremental cost 
// (i.e. cost of a single move to a neighbouring state)
// is always 1.
template<typename T>
class node {
public:
  static const size_t move_cost = 1;
  T data;  
  size_t f;
  size_t g;
  const node * pred;
 
  node(T data, size_t f, size_t g, const node * pred) :
    data(data),
    f(f),
    g(g),
    pred(pred)
    { ; } 
};


// the top of the priority queue is the greatest element by default,
// but we want the smallest, so flip the sign
template<typename T>
bool operator<(const node<T> &a, const node<T> &b) {
  return a.f > b.f;
}

// types used in astar_search -
// these are basically just aliases for function types,
// to make the signature a bit easier to read.

// C++11 alias declaration
// an /heuristic/ returns estimated dist to sol
template <typename T>
using heuristic = std::function<size_t(const T&)>;

// generates neighbouring positions/states
template <typename T>
using get_neighbours = std::function<std::vector<T>(const T&)>;

// compare states/positions for equality - have we found goal?
template <typename T>
using eq = std::function<bool(const T&, const T&)>;

/* used for a previous /set/ implementation -
 * current versions uses /unordered-set/ (a hash-set),
 * so doesn't need '<'
 */
// // compare states/positions with "<" - used for storing 
// // visited and to-visit states.
// template <typename T>
// using lt = std::function<bool(const T&, const T&)>;


// a hash function - used for storing 
// visited and to-visit states.
template <typename T>
using hash = std::function<size_t(const T&)>;

// a-star search.
//  
// templated over /T/, the type of a state, position, or node in a
// graph being searched. 
//
// arguments: 
//   start: a datum being the _start_ position or state;
//   goal: the goal position or state;
//   ngen: get_neighbours<T> : a function of type (const T&) -> vector<T>,
//    used to generate next-states;
//   heur: heuristic<T> : a function of type (const T&) -> size_t,
//     which gives an estimate of distance to the goal.
//
//   (optional) safety_lim, max no of states/positions to visit.
//
// returns: vector containing path (in reverse, from goal)
//  back to start state. If no solution (...and we finish searching),
//  returns an empty vector.
//
// prints some debugging info if DEBUG is defined; you'll need to
// overload the stream output operator << for your type T
// if so, and #include <iostream> before #include'ing "astar.h".
//
// sample usage:
//
//    #include "astar.h"
//    #include <functional>
//    // ^(for std::equal_to etc)
//
//    using astar::heuristic;
//    using astar::get_neighbours;
//    using astar::astar_search;
//
//    int goal = 10;
//    
//    heuristic<int> myHeuristic = [&goal](const int &n) {
//      if (n > 10) {
//        return (size_t) (n - goal);
//      } else {
//        return (size_t) (goal - n);
//      }
//    };
//    
//    get_neighbours<int> myGen = [](const int &n) {
//      vector<int> res;
//      res.push_back(n-1);
//      res.push_back(n+1);
//      //cout << "    generated " << (n-1) << ", " << (n+1) << endl;
//      return res;
//    };
//    
//    // if we've defined an appropriate operator< and operator==
//    // for our type, we can pass the standard operators.
//    // BUT: if we include trace-back data, our operators must not
//    // use that. (else the trace-back will get included in the state,
//    // and we'll generate infinite new states and never end.)
//    vector<int> res = 
//        astar_search<int>( 
//            5, goal, myGen, myHeuristic, 
//            std::equal_to<int>(), std::less<int>() );
//    
//    cout << "back in main" << endl;
//    
//    for( auto && chain_el : res ) {
//      cout << chain_el << " ";
//    }
//    cout << endl;

template<typename T>
std::vector<T> astar_search(
    const T & start,
    const T & goal,
    get_neighbours<T> ngen,
    heuristic<T> heur,
    eq<T> eq,
    hash<T> hash,
    int safety_lim = 100000
  ) {

  // auto node_lt = [&lt](const node<T> & a, const node<T> & b) {
  //     return lt(a.data, b.data);
  //   };

  // we want to be able to retrieve nodes by their _data_,
  // rather than anything else.
  auto node_hash = [&hash](const node<T> & a) {
      return hash(a.data);
    };
  auto node_eq = [&eq](const node<T> & a, const node<T> & b) {
      return eq(a.data, b.data);
    };


#ifdef DEBUG
  std::cout << "starting astar" << std::endl;
#endif

  node<T> start_node{ start, 0, 0, NULL } ;

  // on success, will contain ptr to (something in visited)
  // with a back-chain
  // on failure, it won't
  const node<T> * success = NULL;

  std::priority_queue<node<T> > nodes_to_visit {};
  // we also, irritatingly, need to be able to retrieve items
  // from the "open set" (queue) by _data_/position,
  // so we keep a parallel set, called "open".
  //std::set<node<T>, decltype(node_lt) > open {node_lt } ;
  std::unordered_set<node<T>, decltype(node_hash), decltype(node_eq) > open { 10000, node_hash, node_eq };

  // /visited/ is the 'standard location' for nodes we want to keep
  // a pointer to. So, since states contain a link back to their
  // predecessor state, we rely on things in /visited/ not moving, lol.
  //  This seems to be the case for the g++ standard C++11 libraries,
  // but I don't think it's at all guaranteed.
  //  So a safer approach would be to store them in a vector, say,
  //  and just hand out pointers/indices to the objects in the vector.
  //  Or to store a /map/ from states to predecessor-states.
  //std::set<node<T>, decltype(node_lt) > visited {node_lt };
  std::unordered_set<node<T>, decltype(node_hash), decltype(node_eq) > visited { 10000, node_hash, node_eq };

  nodes_to_visit.push( start_node) ;
  open.insert( start_node );

  // insert a node into "visited", returning address of inserted
  // node
  auto visit = [&visited](const node<T> &some_node) {
    visited.insert(some_node);
    auto success_iter = visited.find(some_node);
    assert (success_iter != visited.end() );
    return &(*success_iter);
  };

  // insert found node into visited, set "success" ptr
  // equal to the inserted copy. 
  auto onSuccess = [&visit,&success,&visited](const node<T> &some_node) {
    success = visit(some_node);
  };

  int safety_ticker = 0;
  while (!nodes_to_visit.empty()) {

    node<T> cur { nodes_to_visit.top() };
#ifdef DEBUG
    std::cout << "  got front: " << cur.data << std::endl;
#endif

    if ( eq(cur.data, goal) ) {
      onSuccess(cur);
      break;
    }

    // add to visited, remove from _to_visit
    visited.insert( cur );
    const node<T> * curPtr = visit(cur);

    nodes_to_visit.pop();
    open.erase( cur );

    // find neighbours as a vec
    auto neighbours = ngen(cur.data);

    // for each successor - if this is the goal, we're done
    // set the back_chain
    // push to back of q
    bool tmp_success = false;

    for( auto && succ : neighbours) {
      // f = g + h
      // g is the distance travelled
      // successor_g = cur.g + cost of move
      size_t succ_g = cur.g + node<T>::move_cost;
      // successor.f = successor.g + (dist from goal to successor)
      size_t succ_f = cur.g + heur( succ );
      node<T> succ_node(succ, succ_f, succ_g, curPtr);
      if ( eq(succ,goal) ) {
        //cout << "hit success in gen stage" << endl;
        onSuccess(succ_node);
        tmp_success = true;
        break;
      }
      // only add this to the "open" list if needed.
      //
      // if there's any node in the "open" list with the same data,
      //    and lower f, then don't bother
      // if there's any node in the "visited" list with the same data,
      //    and lower f, then don't bother.  

      //    Easy to check, because those lists retrieve elements by
      //    _data_/position, so we just compare cost of retrieved
      //    with current.

      auto visited_iter = visited.find(succ_node);
      if ( visited_iter != visited.end() && visited_iter->f < succ_f)
            { continue; }

      auto open_iter = open.find(succ_node);
      if ( open_iter != open.end() && open_iter->f < succ_f)
            { continue; }

      // if still here, we've found a better route
      nodes_to_visit.push( succ_node );
      open.erase( succ_node );
      open.insert( succ_node );
    }

    // break again, 'cos C++ doesn't have labelled loops
    if (tmp_success) {
      break;
    }

    safety_ticker++;
    if (safety_ticker > safety_lim) {
      throw std::logic_error( "safety_lim exceeded" );
    }
  }

#ifdef DEBUG
  std::cout << "finished" << std::endl;
  std::cout << "num ticks: " << safety_ticker << std::endl;
#endif

  // construct the reverse chain leading from goal,
  // back to our start state.
  std::vector<T> res { };
  if (success) {
    const node<T> * cur = success;
    while( cur != NULL ) {        
      res.push_back( cur->data );
      cur = cur->pred;
    }
  }
  return res; 
}

} // end namespace astar

#endif
// ASTAR_H

