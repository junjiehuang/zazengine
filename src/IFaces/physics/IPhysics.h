#ifndef _IPHYSICS_H_
#define _IPHYSICS_H_

#include "../core/ISubSystem.h"

#include "IPhysicsEntity.h"

class IPhysics : public ISubSystem
{
	public:
		virtual ~IPhysics() {};

		virtual IPhysicsEntity* createEntity( TiXmlElement*, IGameObject* parent ) = 0;
};

#endif /* _IPHYSICS_H_ */
