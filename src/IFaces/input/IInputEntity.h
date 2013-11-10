#ifndef _IINPUTENTITY_H_
#define _IINPUTENTITY_H_

#include "core/ISubSystemEntity.h"

class IInputEntity : public ISubSystemEntity
{
	public:
		IInputEntity( IGameObject* p ) : ISubSystemEntity( p ) {};
		virtual ~IInputEntity() {};
};

#endif /* _IINPUTENTITY_H_ */
