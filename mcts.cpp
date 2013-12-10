// Written by Peter Pham, Nov. 2013
// Free for anything; good for nothing.

// This file is inteded to give an example of MCTS usage. In general, the steps
// are the following: implement your state machine (game logic), make a few
// mcts objects, and use!

#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <bitset>

#include "mcts.h"

using namespace std;
using namespace mcts;

// Begin game definition. This is a terrible impelemntation of tic tac toe.
// Low order bits encode the marks for the x player, high order bits encode the
// marks for the o player and the bit at position 25 encodes whose turn it is.

inline static bool check_box(int state, int i, int j, int role) {
  int offset = (role == 0) ? 0 : 9;
  return (state & 1 << (offset + i + 3*j));
}

inline static bool box_filled(int state, int i, int j) {
  return check_box(state, i, j, 0) || check_box(state, i, j, 1);
}

static vector<vector<int>> get_actions_for_state(int state) {
  vector<vector<int>> result;
  int role_shift = (state & 1 << 25) ? 9 : 0;

  vector<int> moves, empty = { 0 };
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (!box_filled(state, i, j)) {
        moves.push_back(1 << (role_shift + i + 3*j));
      }
    }
  }

  if (role_shift == 0) {
    result.push_back(moves);
    result.push_back(empty);
  } else {
    result.push_back(empty);
    result.push_back(moves);
  }

  return result;
}

static int get_next_state(int state, const vector<int>& joint) {
  for (auto action : joint) state |= action;
  state ^= (1 << 25);
  return state;
}

static bool check_rows(int state, int role) {
  for (int i = 0; i < 3; i++) {
    bool filled = true;
    for (int j = 0; j < 3; j++) filled &= check_box(state, i, j, role);
    if (filled) return true;
  }
  return false;
}

static bool check_cols(int state, int role) {
  for (int j = 0; j < 3; j++) {
    bool filled = true;
    for (int i = 0; i < 3; i++) filled &= check_box(state, i, j, role);
    if (filled) return true;
  }
  return false;
}

static bool check_diagonals(int state, int role) {
  bool filled = true;
  for (int i = 0; i < 3; i++) filled &= check_box(state, i, i, role);
  if (filled) return true;
  filled = true;
  for (int i = 0; i < 3; i++) filled &= check_box(state, i, 2 - i, role);
  return filled;
}

static bool is_filled(int state) {
  auto actions = get_actions_for_state(state);
  return actions[0].size() == 0 || actions[1].size() == 0;
}

static vector<double> get_goals(int state) {
  vector<double> result;

  bool victory[2];
  for (int r = 0; r < 2; r++) {
    victory[r] = check_rows(state, r) || check_cols(state, r)
      || check_diagonals(state, r);
  }

  for (int r = 0; r < 2; r++) {
    double goal = 0.0;
    if (victory[r]) goal = 100.0;
    else if (!victory[0] && !victory[1]) goal = 50.0;
    result.push_back(goal);
  }

  return result;
}

static bool is_terminal(int state) {
  auto goals = get_goals(state);
  for (auto goal : goals) if (goal == 100.0) return true;
  return is_filled(state);
}

// End game definition

static void print_action_value(scope<int,int>& s, ucb::suggestor_pool<int,int>& pool, int role) {
  auto action = s.root->suggestors[role].play();
  auto value = pool[s.root->state][role].entries[action].mean;
  cout << "Role: " << role << ", Action: " << action << ", Value: " << value << endl;
}

int main() {
  int initial = 0; // 0x21030; // 0x18010;
  machine::standard<int,int> machine;
  machine.initial = [=] { return initial; };
  machine.evaluator.goals = get_goals;
  machine.evaluator.terminal = is_terminal;
  machine.advancer.next = get_next_state;
  machine.advancer.actions = get_actions_for_state;

  // These are the pieces you need in general for UCT (MCTS with UCB)
  ucb::suggestor_pool<int,int> pool;
  scope<int,int> s(ucb::make_factory<int,int>(50.0, pool, machine.advancer));

  // Set up MCTS by creating the root node
  auto actions = machine.advancer.actions(initial);
  s.root = s.submerge_state(initial);

  int n_charges = 100000;
  for (int i = 0; i < n_charges; i++) {
    auto edge = s.pass(machine.advancer, machine.evaluator);
    s.expand(edge, machine.advancer, machine.evaluator);
    if (i % 10000 == 0) {
      print_action_value(s, pool, 0);
      print_action_value(s, pool, 1);
    }
  }

  return 0;
}




