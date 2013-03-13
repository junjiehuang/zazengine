#ifndef STATEMANAGER_H_
#define STATEMANAGER_H_

#include "State.h"

#include <list>
#include <map>

class StateManager
{
 public:
	 static StateManager& getInstance();
	 static void dispose();
	 
	 void registerState(State*);
	 bool isStateRegistered(const std::string&);
	 
	 void enableState(const std::string&);
	 void disableState(const std::string&);

 private:
	 static StateManager* instance;
	 
	 StateManager();
	 ~StateManager();
	 
	 std::map<std::string, State*> states;
};

#endif /*STATEMANAGER_H_*/
