#include "PhysicSphere.h"

PhysicSphere::PhysicSphere( bool staticFlag, float mass, float radius )
	: PhysicType( staticFlag, mass ),
	radius( radius )
{
}

PhysicSphere::~PhysicSphere()
{
}

bool PhysicSphere::create( dWorldID worldID, dSpaceID spaceID )
{
	this->m_bodyID = 0;
	this->m_geomID = dCreateSphere( spaceID, this->radius );
	
	if ( false == this->m_staticFlag ) {
		this->m_bodyID = dBodyCreate( worldID );
		
		dMass massStruct;
		dMassSetZero(&massStruct);
		dMassSetSphere( &massStruct, 1, this->radius );

		dMassSetSphereTotal( &massStruct, this->m_mass, this->radius );

		dBodySetMass( this->m_bodyID, &massStruct );
		dGeomSetBody( this->m_geomID, this->m_bodyID );

	} 
	else 
	{
		dGeomSetBody( this->m_geomID, 0 );
	}
	
	return true;
}
