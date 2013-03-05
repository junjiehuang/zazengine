#ifndef PHYSICPLANE_H_
#define PHYSICPLANE_H_

#include "PhysicType.h"

class PhysicPlane : public PhysicType
{
public:
	PhysicPlane(bool, float, float, float, float, float);
	virtual ~PhysicPlane();
	
	virtual bool create(dWorldID, dSpaceID);
	
 private:
	 float x;
	 float y;
	 float z;
	 float d;
};

#endif /*PHYSICPLANE_H_*/
