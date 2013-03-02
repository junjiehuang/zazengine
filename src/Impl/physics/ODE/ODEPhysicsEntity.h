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

		const std::string& getType() const { return this->entityType; };

		const bool isStatic() const { return this->physicType->isStatic(); };

		const float* getPos() const { return this->pos; };
		const float* getRot() const { return this->rot; };
		const float* getVel() const { return this->vel; };

		virtual void update();

		virtual bool sendEvent( Event& e );

	private:
		std::string entityType;

		PhysicType* physicType;

		float pos[3];
		float rot[12];
		float vel[3];
};

#endif /* PLAYGROUNDPHYSICSENTITY_H_ */
