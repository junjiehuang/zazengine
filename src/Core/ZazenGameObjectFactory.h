/*
 * ZazenGameObjectFactory.h
 *
 *  Created on: 24.07.2010
 *      Author: jonathan
 */

#ifndef ZAZENGAMEOBJECTFACTORY_H_
#define ZAZENGAMEOBJECTFACTORY_H_

#include "IFaces/IGameObject.h"

class ZazenGameObjectFactory
{
	public:
		ZazenGameObjectFactory();
		~ZazenGameObjectFactory();

		IGameObject* createObject( const std::string& );
};

#endif /* ZAZENGAMEOBJECTFACTORY_H_ */
