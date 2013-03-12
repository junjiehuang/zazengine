#include "PhysicPlane.h"

PhysicPlane::PhysicPlane( bool staticFlag, float mass, float x, float y, float z, float d )
	: PhysicType( staticFlag, mass ),
	x( x ),
	y( y ),
	z( z ),
	d( d )
{
}

PhysicPlane::~PhysicPlane()
{
}

bool PhysicPlane::create( dWorldID worldID, dSpaceID spaceID )
{
	this->bodyID = 0;
	this->geomID = dCreatePlane( spaceID, this->x, this->y, this->z, this->d );
	
	dGeomSetBody( this->geomID, 0 );
	
	return true;
}
