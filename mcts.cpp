// Written by Peter Pham, Nov. 2013
// Free for anything; good for nothing.

// This file is inteded to give an example of MCTS usage. In general, the steps
// are the following: implement your state machine (game logic), make a few
// mcts objects, and use!

#include <iostream>
#include <unordered_map>
#include <unordered_set>

#include "mcts.h"

using namespace std;
using namespace mcts;

// Begin game definition -- yours may not look like this. In general, you just
// need to satisfy the machine interfaces. This game has integer states and
// actions. Actions have the effect of increasing the value of the state. 

static vector<vector<int>> get_actions_for_state(int state, int n_roles) {
  vector<vector<int>> result;
  for (int i = 0; i < n_roles; i++) {
    vector<int> inner = {2};
    if (i == 0) inner = { 1, 2, state };
    result.push_back(inner);
  }
  return result;
}

static int get_next_state(int state, const vector<int>& joint) {
  for (auto action : joint) state += action; return state;
}

static vector<double> get_goals(int state, int n_roles) {
  vector<double> result;
  for (int i = 0; i < n_roles; i++) {
    double score = state % n_roles == i ? 100.0 : 0.0;
    result.push_back(score);
  }
  return result;
}

static bool is_terminal(int state) { return state > 10; }

// End game definition

int main() {
  // n_roles is a parameter specific to this game -- you should figure out how
  // to properly generate your machine based on your own player representation.
  // initial is the initial state.
  int n_roles = 2;
  int initial = 0;
  machine::standard<int,int> machine;
  machine.initial = [=] { return initial; };
  machine.evaluator.goals = [=] (int state) { return get_goals(state, n_roles); };
  machine.evaluator.terminal = is_terminal;
  machine.advancer.next = get_next_state;
  machine.advancer.actions = [=] (int state) {
    return get_actions_for_state(state, n_roles);
  };

  // These are the pieces you need in general for UCT (MCTS with UCB)
  ucb::suggestor_pool<int,int> pool;
  scope<int,int> s(ucb::make_factory<int,int>(50.0, pool, machine.advancer));

  // Set up MCTS by creating the root node
  auto actions = machine.advancer.actions(initial);
  s.root = s.submerge_state(initial);

  int n_charges = 10000;
  for (int i = 0; i < n_charges; i++) {
    auto edge = s.pass(machine.advancer, machine.evaluator);
    s.expand(edge, machine.advancer);
  }

  auto action = s.root->suggestors[0].play();
  auto value = pool[s.root->state][0].entries[action].mean;
  cout << "Action: " << action << ", Value: " << value << endl;

  for (auto& entry : s.nodes) cout << entry.first << " ";
  cout << endl;

  s.root = s.submerge_state(3);
  auto reachable = graph::reachable(s.root->state, s.v_edges());
  for (auto& state : reachable) cout << state << " ";
  cout << endl;

  cout << "Size before flush: " << s.nodes.size() << endl;
  s.flush();
  cout << "Size after flush: " << s.nodes.size() << endl;

  return 0;
}




