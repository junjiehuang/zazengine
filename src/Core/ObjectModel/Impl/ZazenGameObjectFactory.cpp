/*
 * ZazenGameObjectFactory.cpp
 *
 *  Created on: 25.07.2010
 *      Author: jonathan
 */

#include "ZazenGameObjectFactory.h"

#include "GameObject.h"

ZazenGameObjectFactory::ZazenGameObjectFactory()
{

}

ZazenGameObjectFactory::~ZazenGameObjectFactory()
{

}

IGameObject*
ZazenGameObjectFactory::createObject( const std::string& type )
{
	return new GameObject();
}
