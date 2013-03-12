/*
 * ZazenGameObjectFactory.cpp
 *
 *  Created on: 25.07.2010
 *      Author: jonathan
 */

#include "ZazenGameObjectFactory.h"

#include "ZazenGameObject.h"

ZazenGameObjectFactory::ZazenGameObjectFactory()
{
}

ZazenGameObjectFactory::~ZazenGameObjectFactory()
{
	this->m_prototypes.clear();
}

void
ZazenGameObjectFactory::registerPrototype( IGameObject* prototype )
{
	this->m_prototypes[ prototype->getObjectClass() ] = prototype;
}

IGameObject*
ZazenGameObjectFactory::createObject( const std::string& objectClass )
{
	IGameObject* gameObjectInstance = 0;

	std::map<std::string, IGameObject*>::iterator findIter = this->m_prototypes.find( objectClass );
	if ( findIter != this->m_prototypes.end() )
	{
		gameObjectInstance = findIter->second->clone();
	}

	if ( NULL == gameObjectInstance )
	{
		gameObjectInstance = new ZazenGameObject( "ZaZenObject" );
	}

	return gameObjectInstance;
}
