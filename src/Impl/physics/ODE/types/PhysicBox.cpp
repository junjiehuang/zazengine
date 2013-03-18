#include "PhysicBox.h"

PhysicBox::PhysicBox(bool staticFlag, float mass, float x, float y, float z)
	: PhysicType(staticFlag, mass),
	x(x),
	y(y),
	z(z)
{
}

PhysicBox::~PhysicBox()
{
}

bool PhysicBox::create(dWorldID worldID, dSpaceID spaceID)
{
	this->bodyID = 0;
	this->geomID = dCreateBox( spaceID, this->x, this->y, this->z );
	
	if ( false == this->staticFlag ) {
		this->bodyID = dBodyCreate( worldID );
		
		dMass massStruct;
		dMassSetZero( &massStruct );
		dMassSetBoxTotal( &massStruct, this->mass, this->x, this->y, this->z );
		
		dBodySetMass( this->bodyID, &massStruct );
		dGeomSetBody ( this->geomID, this->bodyID );

	}
	else 
	{
		dGeomSetBody( this->geomID, 0 );
	}
	
	return true;
}
