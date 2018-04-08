//////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2018, CCS Labs
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the <organization> nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////////

#include <string>
#include <list>
#include <algorithm>

#include <ros/console.h>

#include "cooperative_driving/reflekte_state_machine.h"

namespace cooperative_driving{      

    void State::addNextState(State *s){
        nextStates.insert(nextStates.end(),s);
    }

    State* State::nextState(std::string n){
        for (std::list<State*>::const_iterator it = this->nextStates.begin(); it != this->nextStates.end(); it++) {
            if((*it)->name.compare(n) == 0) {
                return *it;
            }
        }
    }

    void ReflekteStateMachine::PrintNextStates() {
        ROS_INFO_STREAM("Length = " + std::to_string(currentState.nextStates.size()));
        for (std::list<State*>::const_iterator it = currentState.nextStates.begin(); it != currentState.nextStates.end(); it++) {
            ROS_INFO_STREAM((*it)->name);
        }
    }

    bool State::operator==(std::string anotherState) const {
        return anotherState.compare(this->name)==0;
    }

    void ReflekteStateMachine::goToNextState(std::string stateName){
        currentState = (*currentState.nextState(stateName));
        ROS_INFO_STREAM("CURRENT STATE CHANGED TO: " + currentState.name);
    }

    State ReflekteStateMachine::getState(){
        return currentState;
    }

    void ReflekteStateMachine::setStartState(State* state){
        currentState = *state;
    }
}