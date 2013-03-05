/*
 * ZazenAIEntity.cpp
 *
 *  Created on: Sep 30, 2010
 *      Author: jonathan
 */

#include "ZazenAIEntity.h"

#include <iostream>

using namespace std;

ZazenAIEntity::ZazenAIEntity( IGameObject* p )
	: IAIEntity( p ),
	type( "ai" )
{
}

ZazenAIEntity::~ZazenAIEntity()
{

}

bool
ZazenAIEntity::sendEvent( Event& e )
{
	if ( e == "setOrientation" )
	{
		return true;
	}

	return false;
}

