#include "PhysicBox.h"

PhysicBox::PhysicBox( bool staticFlag, float mass, float x, float y, float z )
	: PhysicType( staticFlag, mass ),
	m_x( x ),
	m_y( y ),
	m_z( z )
{
}

PhysicBox::~PhysicBox()
{
}

bool PhysicBox::create( dWorldID worldID, dSpaceID spaceID )
{
	this->m_bodyID = 0;
	this->m_geomID = dCreateBox( spaceID, this->m_x, this->m_y, this->m_z );
	
	if ( false == this->m_staticFlag ) {
		this->m_bodyID = dBodyCreate( worldID );
		
		dMass massStruct;
		dMassSetZero( &massStruct );
		dMassSetBoxTotal( &massStruct, this->m_mass, this->m_x, this->m_y, this->m_z );
		
		dBodySetMass( this->m_bodyID, &massStruct );
		dGeomSetBody ( this->m_geomID, this->m_bodyID );

	}
	else 
	{
		dGeomSetBody( this->m_geomID, 0 );
	}
	
	return true;
}
