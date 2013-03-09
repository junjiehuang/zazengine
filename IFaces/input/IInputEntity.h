/*
 * IInputEntity.h
 *
 *  Created on: 09.03.2013
 *      Author: Jonathan Thaler
 */

#ifndef IINPUTENTITY_H_
#define IINPUTENTITY_H_

#include <core/ISubSystemEntity.h>

class IInputEntity : public ISubSystemEntity
{
	public:
		IInputEntity( IGameObject* p ) : ISubSystemEntity( p ) {};
		virtual ~IInputEntity() {};
};

#endif /* IGRAPHICSENTITY_H_ */
