/*
 * PlayGroundPhysicsEntity.h
 *
 *  Created on: 08.07.2010
 *      Author: Jonathan Thaler
 */

#ifndef ODEPHYSICSENTITY_H_
#define ODEPHYSICSENTITY_H_

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

		virtual void update();

		virtual bool sendEvent( Event& e );

	private:
		std::string m_entityType;

		PhysicType* m_physicType;

		float m_pos[ 3 ];
		float m_rot[ 12 ];
		float m_vel[ 3 ];
};

#endif /* PLAYGROUNDPHYSICSENTITY_H_ */
