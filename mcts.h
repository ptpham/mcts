// Written by Peter Pham, Nov. 2013
// Free for anything; good for nothing.

#include <functional>
#include <algorithm>
#include <vector>
#include <list>
#include <map>

#include <math.h>
#include <float.h>

// This namespace defines a collection of interfaces used to represent games.
// Functions prefixed with as_ will wrap a given object in the specified
// interface. Players are represented as integers; evaluator.goals returns a
// vector of goal values indexed by player index and advancer.actions returns a
// vector of vectors where inner vector i lists the actions for player i. Games
// are assumed to be simultaneous WLOG.
namespace machine {

  // An evaluator provides information about the end of a game.
  template <class S>
  struct evaluator {
    std::function<std::vector<double> (S)> goals;
    std::function<bool (S)> terminal;
  };

  // An advancer describes the rules of a game.
  template <class S, class A>
  struct advancer {
    std::function<std::vector<std::vector<A>> (S)> actions;
    std::function<S (S, const std::vector<A>&)> next;
  };

  // A standard machine has an advancer, an evaluator, and an initial state.
  template <class S, class A>
  struct standard {
    std::function<S (void)> initial;
    evaluator<S> evaluator;
    advancer<S,A> advancer;
  };

  template <class S, class C>
  evaluator<S> as_evaluator(C& machine) {
    evaluator<S> result;
    result.terminal = [&] (S state) { return machine.terminal(state); };
    result.goals = [&] (S state) { return machine.goals(state); };
    return result;
  }

  template <class S, class A, class C>
  advancer<S,A> as_advancer(C& machine) {
    advancer<S,A> result;
    result.actions = [&] (S state) { return machine.actions(state); };
    result.next = [&] (S state, const std::vector<A>& actions) {
      return machine.next(state, actions);
    };
    return result;
  }

  template <class S, class A, class C>
  standard<S,A> as_machine(C& machine) {
    machine::standard<S,A> result;
    result.initial = [&] { return machine.initial(); };
    result.evaluator = as_evaluator(machine);
    result.advancer = as_advancer(machine);
    return result;
  }
}

namespace colut {
  template <class U, class V, 
    class S = std::map<U,V>, class T = std::unordered_set<U>>
  S retain_keys(const S& s, const T& t) {
    S result;
    for (auto& elem : s) {
      if (t.find(elem.first) == t.end()) continue;
      result.insert(elem);
    }
    return result;
  }
}

namespace graph {

  // Represents a set of virtual edges
  template <class T>
  using v_edges = std::function<std::vector<T> (const T&)>;

  // Uses breadth-first traversal to find the set of elements reachable from
  // the given node using the given set of virtual edges.
  template <class T>
  std::unordered_set<T> reachable(const T& root, const v_edges<T>& edges) {
    std::vector<T> explore, next = { root };
    std::unordered_set<T> result;

    while (next.size() > 0) {
      explore = next;
      next.clear();

      while (explore.size() > 0) {
        auto top = explore.back();
        explore.pop_back();

        if (result.find(top) != result.end()) continue;
        result.insert(top);

        auto children = edges(top);
        for (auto& child : children) next.push_back(child);
      }
    }
    
    return result;
  }

}

// This namespace contains the core logic of MCTS. It uses the game
// representation defined in the machine namespace. A suggestor is an object
// that "plays for" a player at a specific node in the game graph.
namespace mcts {

  template <class A>
  struct suggestor {
    std::function<A (void)> explore, play;
    std::function<void (A, double)> inform;
  };

  template <class A, class C>
  suggestor<A> as_suggestor(C& suggestor) {
    mcts::suggestor<A> result;
    result.play = [&] { return suggestor.play(); };
    result.explore = [&] { return suggestor.explore(); };
    result.inform = [&] (A action, double value) {
      return suggestor.inform(action, value);
    };
    return result;
  }

  template <class S, class A>
  struct node {
    std::map<std::vector<A>,node<S,A>*> children;
    std::vector<suggestor<A>> suggestors;
    S state;

  };

  template <class A>
  void inform_joint(const std::vector<suggestor<A>>& suggestors,
    const std::vector<A>& actions, const std::vector<double>& goals) {
    for (unsigned i = 0; i < suggestors.size(); i++) {
      suggestors[i].inform(actions[i], goals[i]);
    }
  }

  template <class T> static std::vector<T> 
  random_joint(const std::vector<std::vector<T>>& actions) {
    std::vector<T> result;
    for (auto& slice : actions) {
      result.push_back(slice[rand() % slice.size()]);
    }
    return result;
  }

  // Continues to advance with random actions from the given state until the
  // game has terminated. Returns the goal vector at the terminal node.
  template <class S, class A>
  std::vector<S> depth_charge(S state, machine::advancer<S,A>& advancer, 
    machine::evaluator<S>& evaluator) {
    std::vector<S> result = { state };
    while (!evaluator.terminal(state)) {
      auto actions = advancer.actions(state);
      auto joint = random_joint(actions);
      state = advancer.next(state, joint);
      result.push_back(state);
    }
    return result;
  }

  template <class S, class A>
  using boundary_edge = std::pair<node<S,A>*, std::vector<A>>;

  template <class S, class A>
  using node_factory = std::function<node<S,A> (S)>;
   
  // Contains a collection of nodes that are presumably part of the same game.
  // The nodes map is a memory deposit; root describes the current game state.
  // Any pointers to nodes should come from the memory deposit. Call
  // submerge_state to obtain a node pointer valid in this scope.
  template <class S, class A>
  struct scope {
    std::map<S,node<S,A>> nodes;
    node_factory<S,A> factory;
    node<S,A>* root;

    scope(const node_factory<S,A>& factory) : factory(factory) { }

    // Converts the scope into a collection of virtual edges
    graph::v_edges<S> v_edges() {
      return [&] (const S& state) {
        std::vector<S> result;
        auto node = submerge_state(state);
        for (auto& child : node->children) {
          result.push_back(child.second->state);
        }
        return result;
      };
    }

    // Removes nodes that are no longer reachable from the root
    std::unordered_set<S> flush() {
      auto reachable = graph::reachable(root->state, v_edges());
      nodes = colut::retain_keys<S,A>(nodes, reachable);
      return reachable;
    }

    node<S,A>* submerge_state(S state) {
      auto found = nodes.find(state);
      if (found == nodes.end()) {
        nodes.emplace(state, factory(state));
        found = nodes.find(state);
      }
      return &found->second;
    }

    // Performs one iteration of MCTS. The steps are a walk_to_boundary, a
    // depth_charge, and a propagation of the depth charge value up the path
    // taken in the currently represented tree. The node pointer in the result
    // may be NULL if the boundary node is terminal.
    boundary_edge<S,A> pass(machine::advancer<S,A>& advancer, 
      machine::evaluator<S>& evaluator) {
      auto path = walk_to_boundary(root);
      auto end = path.back();

      auto target = advancer.next(end.first->state, end.second);
      auto charge = mcts::depth_charge(target, advancer, evaluator);

      auto goals = evaluator.goals(charge.back());
      for (auto& edge : path) {
        inform_joint(edge.first->suggestors, edge.second, goals);
      }

      return path.back();
    }

    node<S,A>* expand(boundary_edge<S,A>& edge, 
    machine::advancer<S,A>& advancer, machine::evaluator<S>& evaluator) {
      if (edge.first == NULL) return NULL;
      auto next = advancer.next(edge.first->state, edge.second);
      if (evaluator.terminal(next)) return NULL;
      auto child = submerge_state(next);
      edge.first->children.emplace(edge.second, child);
      return child;
    }
  };

  // Finds the currently suggested path from the given node to the edge of the
  // reachable DAG. The end of the path returned may contain a terminal node.
  template <class S, class A>
  std::vector<boundary_edge<S,A>> walk_to_boundary(node<S,A>* root) {
    std::vector<boundary_edge<S,A>> result;
    std::vector<A> joint;
    node<S,A>* cur = root;
    while (cur != NULL) {
      joint.clear();
      for (auto &suggestor : cur->suggestors) {
        joint.push_back(suggestor.explore());
      }

      result.push_back(make_pair(cur, joint));
      auto next = cur->children.find(joint);
      if (next != cur->children.end()) cur = next->second;
      else cur = NULL;
    } 
    return result; 
  }
}

// This namespace contains an implementation of a suggestor for MCTS,
// namely the upper confidence bound suggestor.
namespace ucb {
  struct entry {
    double mean, count;
    entry() : mean(0.0), count(0.0) { }
    
    void update(double value) {
      count++;
      double delta = value - mean;
      mean += delta/count;
    }

    double upper(double c, double total) const {
      if (count == 0) return DBL_MAX;
      return mean + c * sqrt(2*log2(total)/count);
    }
  };

  template <class A>
  struct suggestor {
    std::map<A, entry> entries;
    double total, c;

    suggestor(double c, const std::vector<A>& actions) : total(0.0), c(c) {
      for (auto& action : actions) entries.emplace(action, entry());
    }

    void inform(A action, double value) {
      entries.find(action)->second.update(value);
      total++;
    }

    // Returns the action with the best mean
    A play() const {
      auto best = entries.begin();
      for (auto entry = entries.begin(); entry != entries.end(); entry++) {
        if (best->second.mean < entry->second.mean) {
          best = entry;
        }
      }
      return best->first;
    }

    // Returns the action with the best upper confidence bound
    A explore() const { 
      auto best = entries.begin();
      for (auto entry = entries.begin(); entry != entries.end(); entry++) {
        if (best->second.upper(c, total) < entry->second.upper(c, total)) {
          best = entry;
        }
      }
      return best->first;
    }
  };

  template <class S, class A>
  using suggestor_pool = std::map<S, std::vector<ucb::suggestor<A>>>;

  // Creates a UCB node for the given state using the given actions. The list
  // of actions at position i is the list of actions for player i. The pool
  // parameter is the location where the newly created suggestors are stored.
  template <class S, class A>
  mcts::node<S,A> make_node(S state, const std::vector<std::vector<A>>& actions,
    double c, suggestor_pool<S,A>& pool) {
    mcts::node<S,A> result;
    result.state = state;

    auto& target = pool[state];
    for (auto& slice : actions) {
      target.push_back(ucb::suggestor<A>(c, slice));
    }

    for (auto& raw : target) {
      result.suggestors.push_back(
        mcts::as_suggestor<A, ucb::suggestor<A>>(raw));
    }
    return result;
  }

  // This creates a factory for UCB nodes with the given exploration constant
  // the given place to deposit created ucb suggestors. 
  template <class S, class A> mcts::node_factory<S,A> 
  make_factory(double c, suggestor_pool<S,A>& pool,
    machine::advancer<S,A>& advancer) {
    return [=, &pool, &advancer] (S next) {
      return ucb::make_node(next, advancer.actions(next), c, pool);
    };
  }
}



