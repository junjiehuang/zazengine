#ifndef _IPHYSICSENTITY_H_
#define _IPHYSICSENTITY_H_

#include "core/ISubSystemEntity.h"

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

#endif /* _IPHYSICSENTITY_H_ */
