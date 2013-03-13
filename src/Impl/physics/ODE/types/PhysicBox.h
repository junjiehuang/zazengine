#ifndef PHYSICBOX_H_
#define PHYSICBOX_H_

#include "PhysicType.h"

class PhysicBox : public PhysicType
{
public:
	PhysicBox(bool, float, float, float, float);
	virtual ~PhysicBox();
	
	virtual bool create(dWorldID, dSpaceID);
	
 private:
	 float x;
	 float y;
	 float z;
};

#endif /*PHYSICBOX_H_*/
