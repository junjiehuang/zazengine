#ifndef _ODEPHYSICSENTITY_H_
#define _ODEPHYSICSENTITY_H_

#include <physics/IPhysicsEntity.h>

#include "types/PhysicType.h"

class ODEPhysicsEntity : public IPhysicsEntity
{
	public:
		friend class ODEPhysics;

		ODEPhysicsEntity( IGameObject* p );
		virtual ~ODEPhysicsEntity();

		const std::string& getType() const { return this->m_entityType; };

		const bool isStatic() const { return this->m_physicType->isStatic(); };

		dBodyID getBodyId() { return this->m_physicType->getBodyID(); };

		const float* getPos() const { return this->m_pos; };
		const float* getRot() const { return this->m_rot; };
		const float* getVel() const { return this->m_vel; };
		
		long long getLastCollTs() { return this->m_lastCollTs; };
		void setLastCollTs( long long lastCollTs ) { this->m_lastCollTs = lastCollTs; };

		virtual void update();

		virtual bool sendEvent( Event& e );

	private:
		std::string m_entityType;

		long long m_lastCollTs;

		PhysicType* m_physicType;

		float m_pos[ 3 ];
		float m_rot[ 12 ];
		float m_vel[ 3 ];
};

#endif /* _ODEPHYSICSENTITY_H_ */
