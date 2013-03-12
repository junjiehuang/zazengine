/*
 * ZazenGameObjectFactory.h
 *
 *  Created on: 12.03.2013
 *      Author: jonathan
 */

#ifndef IGAMEOBJECTFACTORY_H_
#define IGAMEOBJECTFACTORY_H_

#include <core/IGameObject.h>

class IGameObjectFactory
{
	public:
		virtual ~IGameObjectFactory() {};

		virtual IGameObject* createObject( const std::string& objectClass ) = 0;
};

#endif /* ZAZENGAMEOBJECTFACTORY_H_ */
