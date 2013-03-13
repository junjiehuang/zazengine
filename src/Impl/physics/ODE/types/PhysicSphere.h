#ifndef PHYSICSPHERE_H_
#define PHYSICSPHERE_H_

#include "PhysicType.h"

class PhysicSphere : public PhysicType
{
public:
	PhysicSphere( bool, float, float );
	virtual ~PhysicSphere();
	
	virtual bool create( dWorldID, dSpaceID );
	
 private:
	 float radius;
};

#endif /*PHYSICSPHERE_H_*/
