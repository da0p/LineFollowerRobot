#include "finite_state_machine.h"
#include <Arduino.h>

// IMPLEMENT THE STATE CLASS

State::State(stateFunctions_t updateFunction) {
	init(-1, NULL, updateFunction, NULL);
}

State::State(int id, stateFunctions_t updateFunction) {
	init(id, NULL, updateFunction, NULL);
}

State::State(int id, stateFunctions_t enterFunction,
	                   stateFunctions_t updateFunction,
	                   stateFunctions_t exitFunction) {
	init(id, enterFunction, updateFunction, exitFunction);
}

void State::init(int id, stateFunctions_t enterFunction,
	                       stateFunctions_t updateFunction,
	                       stateFunctions_t exitFunction) {
	_id             = id;
	_enterFunction  = enterFunction;
	_updateFunction = updateFunction;
	_exitFunction   = exitFunction;
}

int State::id() {
  return _id;
}

void State::enter() {
  if(_enterFunction) {
    _enterFunction();
  }
}

void State::update() {
  if(_updateFunction) {
    _updateFunction();
  }
}

void State::exit() {
  if(_exitFunction) {
    _exitFunction();
  }
}

// FINITE STATE MACHINE

FiniteStateMachine::FiniteStateMachine(State* current) {
  _currentState = current;
  _nextState   = NULL;
}

void FiniteStateMachine::update() {
  if(_nextState == NULL) { // it is the first time we call update
    // *(_currentState).enter(); // 
    _currentState->enter();
    _nextState = _currentState;
  }

  if(_currentState != _nextState) {  // when we schedule a transition
    _currentState->exit();
    _currentState = _nextState;
    _currentState->enter();
  }

  _currentState->update();

}

void FiniteStateMachine::transitionTo(State* state) {
  _nextState = state; 
}

void FiniteStateMachine::immediateTransitionTo(State* state){
  _nextState = _currentState = state;
  _currentState->enter();
}

State* FiniteStateMachine::getCurrentState() {
  return _currentState;
}




























