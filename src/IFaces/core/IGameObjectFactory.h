#ifndef _IGAMEOBJECT_FACTORY_H_
#define _IGAMEOBJECT_FACTORY_H_

#include "IGameObject.h"

class IGameObjectFactory
{
	public:
		virtual ~IGameObjectFactory() {};

		virtual IGameObject* createObject( const std::string& objectClass ) = 0;
};

#endif /* _IGAMEOBJECT_FACTORY_H_ */
