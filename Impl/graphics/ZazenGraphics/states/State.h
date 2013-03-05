#ifndef STATE_H_
#define STATE_H_

#include <string>

class State
{
 public:
	 State(const std::string& id) :id(id) { this->counter = 0; };
	 virtual ~State() {};
	 
	 int counter;
	 
	 const std::string& getID() { return this->id; };
	 
	 virtual void enable() = 0;
	 virtual void disable() = 0;
	 
 private:
	const std::string id;
};

#endif /*STATE_H_*/
