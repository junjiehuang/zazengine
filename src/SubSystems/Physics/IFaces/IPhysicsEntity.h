/*
 * IPhysicsEntity.h
 *
 *  Created on: 15.07.2010
 *      Author: joni
 */

#ifndef IPHYSICSENTITY_H_
#define IPHYSICSENTITY_H_

#include "ISubSystemEntity.h"

class IPhysicsEntity : public ISubSystemEntity
{
	public:
		IPhysicsEntity( IGameObject* p ) : ISubSystemEntity( p ) {};
		virtual ~IPhysicsEntity() {};

		virtual const bool isStatic() const = 0;

		virtual const float* getPos() const = 0;
		virtual const float* getRot() const = 0;
		virtual const float* getVel() const = 0;
};

#endif /* IPHYSICSENTITY_H_ */
