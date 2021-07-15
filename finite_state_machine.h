#ifndef FINITE_STATE_MACHINE_H
#define FINITE_STATE_MACHINE_H

typedef void (*stateFunctions_t)();

class State {
public:
  // constructors
  State(stateFunctions_t updateFunction); // for a state woth only update
  State(int id, stateFunctions_t updateFunction); // same as above but with id
  State(int id, stateFunctions_t enterFunction,
                 stateFunctions_t updateFunction,
                 stateFunctions_t exitFnction);
  void init(int id, stateFunctions_t enterFunction,
                    stateFunctions_t updateFunction,
                    stateFunctions_t exitFunction);

  int id(); // id acessor
  void enter();
  void update();
  void exit();

private:
  int _id;
  stateFunctions_t _enterFunction;
  stateFunctions_t _updateFunction;
  stateFunctions_t _exitFunction;

};

// FINITE STATE MACHINE

class FiniteStateMachine {
public:
  FiniteStateMachine(State* current);

  void update();

  void transitionTo(State* state);
  void immediateTransitionTo(State* state);

  State* getCurrentState();

private:
  State* _currentState; // pointer to the current state 
  State* _nextState;    // poiter to the next state
};

#endif
