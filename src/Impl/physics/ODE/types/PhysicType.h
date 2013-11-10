#ifndef _PHYSICTYPE_H_
#define _PHYSICTYPE_H_

#include <ode/ode.h>

class PhysicType
{
	public:
		PhysicType( bool staticFlag, float mass )
		{ 
			this->m_staticFlag = staticFlag; 
			this->m_mass = mass; 
		};

		virtual ~PhysicType()
		{ 
			if ( this->m_bodyID )
			{
			dBodyDestroy( this->m_bodyID );
			}

			dGeomDestroy( this->m_geomID );
		};
	 
		bool isStatic() { return this->m_staticFlag; };
		float getMass() { return this->m_mass; };

		void setPosition( float x, float y, float z )
		{
			if ( this->m_staticFlag )
			{
				dGeomSetPosition( this->m_geomID, x, y, z );
			}
			else
			{
				dBodySetPosition( this->m_bodyID, x, y, z );
			}
		};

		dBodyID getBodyID() { return this->m_bodyID; };

		virtual bool create( dWorldID, dSpaceID ) = 0;
	 
	protected:
		bool m_staticFlag;
		float m_mass;
	 
		dBodyID m_bodyID;
		dGeomID m_geomID;
};

#endif /* _PHYSICTYPE_H_ */
