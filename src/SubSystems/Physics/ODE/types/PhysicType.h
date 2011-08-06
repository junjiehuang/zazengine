#ifndef PHYSICTYPE_H_
#define PHYSICTYPE_H_

#include <ode/ode.h>

class PhysicType
{
 public:
	 PhysicType(bool staticFlag, float mass) { this->staticFlag = staticFlag; this->mass = mass; };
	 virtual ~PhysicType()
	 { 
		 if (this->bodyID)
			dBodyDestroy(this->bodyID);
		 
		 dGeomDestroy(this->geomID);
	 };
	 
	 bool isStatic() { return this->staticFlag; };
	 float getMass() { return this->mass; };

	 void setPosition( double x, double y, double z )
	 {
		 if ( this->staticFlag )
			 dGeomSetPosition(this->geomID, x, y, z);
		 else
			 dBodySetPosition(this->bodyID, x, y, z);
	 };

	 dBodyID getBodyID() { return this->bodyID; };

	 virtual bool create( dWorldID, dSpaceID ) = 0;
	 
 protected:
	 bool staticFlag;
	 float mass;
	 
	 dBodyID bodyID;
	 dGeomID geomID;

};

#endif /*PHYSICTYPE_H_*/
