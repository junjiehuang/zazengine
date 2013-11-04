/*
 * PlayGroundRendererEntity.cpp
 *
 *  Created on: 09.07.2010
 *      Author: Jonathan Thaler
 */

#include "ZazenGraphicsEntity.h"
#include "ZazenGraphics.h"

#include <core/ICore.h>

#include <glm/gtc/type_ptr.hpp>

#include <iostream>

using namespace std;
using namespace boost;

ZazenGraphicsEntity::ZazenGraphicsEntity( IGameObject* p )
	: IGraphicsEntity( p ),
	m_type( "graphics" ),
	Orientation( m_modelMatrix )
{
	this->m_activeAnimation = 0;

	this->m_visible = true;
	this->m_distance = 0;
	this->m_lastFrame = 0;
	
	this->m_mesh = 0;
	this->m_material = 0;

	this->m_light = 0;
	this->m_camera = 0;

	this->m_isAnimated = false;
	this->m_animRoll = 0;
	this->m_animPitch = 0;
	this->m_heading = 0;
}

ZazenGraphicsEntity::~ZazenGraphicsEntity()
{
}

void
ZazenGraphicsEntity::matrixChanged()
{
	Event e( "POSITION_CHANGED" );
	e.addValue( "matrix", glm::value_ptr( this->getModelMatrix() ) );
	e.setTarget( this->getParent() );

	ZazenGraphics::getInstance().getCore().getEventManager().postEvent( e );
}

void
ZazenGraphicsEntity::update()
{
	if ( this->m_isAnimated )
	{
		if ( 0.0f != this->m_heading )
		{
			this->m_camera->changeHeading( this->m_heading * ZazenGraphics::getInstance().getCore().getProcessingFactor() );
		}

		if ( 0.0f != this->m_animRoll )
		{
			this->m_camera->changeRoll( this->m_animRoll * ZazenGraphics::getInstance().getCore().getProcessingFactor() );
		}
	
		if ( 0.0f != this->m_animPitch )
		{
			this->m_camera->changePitch( this->m_animPitch * ZazenGraphics::getInstance().getCore().getProcessingFactor() );
		}
	}

	list<int>::iterator pressedKeysIter = this->m_keyDown.begin();
	while ( pressedKeysIter != this->m_keyDown.end() )
	{
		int keyCode = *pressedKeysIter++;
		
		// Q
		if ( 16 == keyCode )
		{
			this->m_camera->changeRoll( 50.0f * ZazenGraphics::getInstance().getCore().getProcessingFactor() );
			//this->postPositionChangedEvent();
		}
		// E
		else if ( 18 == keyCode )
		{
			this->m_camera->changeRoll( -50.0f * ZazenGraphics::getInstance().getCore().getProcessingFactor() );
			//this->postPositionChangedEvent();
		}
		// W
		else if ( 17 == keyCode )
		{
			this->m_camera->strafeForward( -100.0f * ZazenGraphics::getInstance().getCore().getProcessingFactor() );
			//this->postPositionChangedEvent();
		}
		// S
		else if ( 31 == keyCode )
		{
			this->m_camera->strafeForward( 100.0f * ZazenGraphics::getInstance().getCore().getProcessingFactor() );
			//this->postPositionChangedEvent();
		}
		// A
		else if ( 30 == keyCode )
		{
			this->m_camera->strafeRight( -100.0f * ZazenGraphics::getInstance().getCore().getProcessingFactor() );
			//this->postPositionChangedEvent();
		}
		// D
		else if ( 32 == keyCode )
		{
			this->m_camera->strafeRight( 100.0f * ZazenGraphics::getInstance().getCore().getProcessingFactor() );
			//this->postPositionChangedEvent();
		}
	}

	if ( this->m_activeAnimation )
	{
		this->m_activeAnimation->animate( this->m_mesh );
	}
}

bool
ZazenGraphicsEntity::sendEvent( Event& e )
{
	// process immediately because no call to graphics api, just coping of data (yet)
	if ( e == "UPDATE_PHYSICS" )
	{
		boost::any& pos = e.getValue( "pos" );
		boost::any& rot = e.getValue( "rot" );

		this->setOrientation( any_cast<const float*>( pos ), any_cast<const float*>( rot ) );

		return true;
	}
	else if ( e == "KEY_PRESSED" )
	{
		int keyCode = any_cast<int>( e.getValue( "key" ) );

		this->m_keyDown.push_back( keyCode );
	}
	else if ( e == "KEY_RELEASED" )
	{
		int keyCode = any_cast<int>( e.getValue( "key" ) );

		this->m_keyDown.remove( keyCode );
	}
	else if ( e == "MOUSE_MOVED" )
	{
		int x = any_cast<int>( e.getValue( "x" ) );
		int y = any_cast<int>( e.getValue( "y" ) );

		this->m_camera->changeHeading( -50.0f * x * ZazenGraphics::getInstance().getCore().getProcessingFactor() );
		this->m_camera->changePitch( 50.0f * y * ZazenGraphics::getInstance().getCore().getProcessingFactor() );

		//this->postPositionChangedEvent();
	}

	return false;
}

void
ZazenGraphicsEntity::setOrientation( const float* pos, const float* rot )
{
	this->m_camera->setRaw( rot, pos );
}

void
ZazenGraphicsEntity::setAnimation( float heading, float roll, float pitch )
{
	this->m_isAnimated = true;
	this->m_heading = heading;
	this->m_animRoll = roll;
	this->m_animPitch = pitch;
}

void
ZazenGraphicsEntity::recalculateDistance( const glm::mat4& viewMatrix )
{
	// calculate model-view for this instance
	glm::mat4 modelView = viewMatrix * this->m_modelMatrix;
	// calculate center of instance in view-space
	glm::vec4 vsCenter = modelView * glm::vec4( this->m_mesh->getCenter(), 1.0 );

	// distance from the viewer is the z-component of the center
	this->m_distance = vsCenter.z;
}
