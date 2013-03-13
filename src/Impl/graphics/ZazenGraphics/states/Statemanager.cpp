#include "Statemanager.h"

using namespace std;

StateManager* StateManager::instance = 0;

StateManager& StateManager::getInstance()
{
	if (StateManager::instance == 0)
		StateManager::instance = new StateManager();
	
	return *StateManager::instance;
}

void StateManager::dispose()
{
	if (StateManager::instance)
		delete StateManager::instance;
	
	StateManager::instance = 0;
}

StateManager::StateManager()
{
}

StateManager::~StateManager()
{
	map<string, State*>::iterator iter = this->states.begin();
	while (iter != this->states.end()) {
		delete iter->second;
		
		iter++;
	}
}

void StateManager::registerState(State* state)
{
	map<string, State*>::iterator findIter = this->states.find(state->getID());
	if (findIter != this->states.end())
		return;

	this->states[state->getID()] = state;
}

bool StateManager::isStateRegistered(const std::string& id)
{
	map<string, State*>::iterator findIter = this->states.find(id);
	return findIter != this->states.end();
}

void StateManager::enableState(const std::string& id)
{
	map<string, State*>::iterator findIter = this->states.find(id);
	if (findIter == this->states.end())
		return;
	
	State* state = findIter->second;
	
	if (state->counter == 0)
		state->enable();
	
	state->counter++;
}

void StateManager::disableState(const std::string& id)
{
	map<string, State*>::iterator findIter = this->states.find(id);
	if (findIter == this->states.end())
		return;
	
	State* state = findIter->second;
	
	if (state->counter == 1)
		state->disable();

	if (state->counter > 0)
		state->counter--;
}
