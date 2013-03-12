/*
 * ZazenGameObjectFactory.h
 *
 *  Created on: 24.07.2010
 *      Author: jonathan
 */

#ifndef ZAZENGAMEOBJECTFACTORY_H_
#define ZAZENGAMEOBJECTFACTORY_H_

#include <core/IGameObject.h>

class ZazenGameObjectFactory
{
	public:
		ZazenGameObjectFactory();
		~ZazenGameObjectFactory();

		void registerPrototype( IGameObject* );

		IGameObject* createObject( const std::string& );

	private:
		std::map<std::string, IGameObject*> m_prototypes;
};

#endif /* ZAZENGAMEOBJECTFACTORY_H_ */
