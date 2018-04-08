#pragma once

#include <string>
#include <list>

namespace cooperative_driving {

    class State {
        public:
            std::string name;
            std::list<State*> nextStates;
            
            State(std::string n, bool s=false):name(n){};
            State(std::string n, bool s, std::list<State*> States):name(n), nextStates(States){};
            void addNextState(State *s);
            State* nextState(std::string n);
            bool operator==(std::string anotherState) const;
    };

    class ReflekteStateMachine {
        private:
            State currentState;

        public:
            ReflekteStateMachine():currentState(State("")){};
            void goToNextState(std::string stateName);
            State getState();
            void setStartState(State *state);
            void PrintNextStates();
    };
}