#include "ZazenOISEntity.h"

#include <iostream>

using namespace std;

ZazenOISEntity::ZazenOISEntity( IGameObject* p )
	: IInputEntity( p ),
	m_type( "graphics" )
{
}

ZazenOISEntity::~ZazenOISEntity()
{
}

bool
ZazenOISEntity::sendEvent( Event& e )
{
	return true;
}
