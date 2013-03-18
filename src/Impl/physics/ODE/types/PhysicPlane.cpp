#include "PhysicPlane.h"

PhysicPlane::PhysicPlane( bool staticFlag, float mass, float x, float y, float z, float d )
	: PhysicType( staticFlag, mass ),
	m_x( x ),
	m_y( y ),
	m_z( z ),
	m_d( d )
{
}

PhysicPlane::~PhysicPlane()
{
}

bool PhysicPlane::create( dWorldID worldID, dSpaceID spaceID )
{
	this->m_bodyID = 0;
	this->m_geomID = dCreatePlane( spaceID, this->m_x, this->m_y, this->m_z, this->m_d );
	
	dGeomSetBody( this->m_geomID, 0 );
	
	return true;
}
