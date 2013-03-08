/*
 * PlayGroundRendererEntity.cpp
 *
 *  Created on: 09.07.2010
 *      Author: joni
 */

#include "ZazenGraphicsEntity.h"
#include "ZazenGraphics.h"

#include <glm/gtc/type_ptr.hpp>

#include <iostream>

using namespace std;
using namespace boost;

ZazenGraphicsEntity::ZazenGraphicsEntity( IGameObject* p )
	: IGraphicsEntity( p ),
	m_type( "graphics" )
{
	this->m_orientation = 0;
}

ZazenGraphicsEntity::~ZazenGraphicsEntity()
{
}

bool
ZazenGraphicsEntity::sendEvent( Event& e )
{
	// process immediately because no call to graphics api, just coping of data (yet)
	if ( e == "updatePhysics" )
	{
		boost::any& pos = e.getValue( "pos" );
		boost::any& rot = e.getValue( "rot" );

		this->setOrientation( any_cast<const float*>( pos ), any_cast<const float*>( rot ) );

		return true;
	}
	else if  ( e == "SDLK_RIGHT" )
	{
		this->m_orientation->changeHeading( -0.1f * ZazenGraphics::getInstance().getCore().getProcessingFactor() );
	}
	else if ( e == "SDLK_LEFT" )
	{
		this->m_orientation->changeHeading( 0.1f * ZazenGraphics::getInstance().getCore().getProcessingFactor() );
	}
	else if ( e == "SDLK_UP" )
	{
		this->m_orientation->changePitch( -0.1f * ZazenGraphics::getInstance().getCore().getProcessingFactor() );
	}
	else if ( e == "SDLK_DOWN" )
	{
		this->m_orientation->changePitch( 0.1f * ZazenGraphics::getInstance().getCore().getProcessingFactor() );
	}
	else if ( e == "SDLK_w" )
	{
		this->m_orientation->strafeForward( -0.1f * ZazenGraphics::getInstance().getCore().getProcessingFactor() );
	}
	else if ( e == "SDLK_s" )
	{
		this->m_orientation->strafeForward( 0.1f * ZazenGraphics::getInstance().getCore().getProcessingFactor() );
	}
	else if ( e == "SDLK_d" )
	{
		this->m_orientation->changeRoll( -0.1f * ZazenGraphics::getInstance().getCore().getProcessingFactor() );
	}
	else if ( e == "SDLK_a" )
	{
		this->m_orientation->changeRoll( 0.1f * ZazenGraphics::getInstance().getCore().getProcessingFactor() );
	}

	return false;
}

void
ZazenGraphicsEntity::setOrientation( const float* pos, const float* rot)
{
	this->m_orientation->setRaw( rot, pos );
}
