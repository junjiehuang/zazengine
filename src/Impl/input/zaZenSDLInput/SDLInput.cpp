/*
 *  SDLInput.cpp
 *  PlayGround
 *
 *  Created by Jonathan Thaler on 17.07.10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "SDLInput.h"

#include <core/ICore.h>

#include <iostream>

#include <SDL/SDL.h>

using namespace std;

SDLInput::SDLInput( const std::string& id, ICore* core )
	: id ( id ),
	type ( "input" ),
	m_core( core )
{
	
}

SDLInput::~SDLInput()
{
	
}

bool
SDLInput::initialize( TiXmlElement* configNode )
{
	cout << endl << "=============== SDLInput initializing... ===============" << endl;

	if ( false == this->initSDL( configNode ) )
	{
		return false;
	}

	cout << "================ SDLInput initialized =================" << endl;

	return true;
}

bool
SDLInput::shutdown()
{
	cout << endl << "=============== SDLInput shutting down... ===============" << endl;

	cout << "================ SDLInput shutdown =================" << endl;

	return true;
}

bool
SDLInput::start()
{
	return true;
}

bool
SDLInput::stop()
{
	return true;
}

bool
SDLInput::pause()
{
	return true;
}

bool
SDLInput::process( double factor )
{
	//cout << "SDLInput::process" << endl;

	SDL_Event event;
	
	if ( SDL_PollEvent( &event ) )
	{
		if ( SDL_KEYDOWN == event.type )
		{
			//ScriptSystem::getInstance().callFunc( "onKeyDown" );

			pressedKeys.push_back( event.key.keysym.sym );
			
		}
		else if ( SDL_KEYUP == event.type )
		{
			//ScriptSystem::getInstance().callFunc( "onKeyUp" );

			pressedKeys.remove( event.key.keysym.sym );
			
		}
		else if ( SDL_MOUSEMOTION == event.type )
		{
			//this->camera->changeHeading(event.motion.xrel * loopFactor * 0.01);
			//this->camera->changePitch(event.motion.yrel * loopFactor * 0.01);
			
		}
		else if ( SDL_QUIT == event.type )
		{
			this->m_core->stop();
		}
	}
	
	list<int>::iterator pressedKeysIter = pressedKeys.begin();
	while ( pressedKeysIter != pressedKeys.end() )
	{
		int key = *pressedKeysIter++;
		
		switch( key )
		{
			case SDLK_RIGHT:
			{
				this->m_core->getEventManager().postEvent( Event( "SDLK_RIGHT" ) );
				break;
			}
			case SDLK_LEFT:
				this->m_core->getEventManager().postEvent( Event( "SDLK_LEFT" ) );
				break;
				
			case SDLK_UP:
				this->m_core->getEventManager().postEvent( Event( "SDLK_UP" ) );
				break;
				
			case SDLK_DOWN:
				this->m_core->getEventManager().postEvent( Event( "SDLK_DOWN" ) );
				break;
				
			case SDLK_w:
				this->m_core->getEventManager().postEvent( Event( "SDLK_w" ) );
				break;
				
			case SDLK_s:
				this->m_core->getEventManager().postEvent( Event( "SDLK_s" ) );
				break;
				
			case SDLK_d:
				this->m_core->getEventManager().postEvent( Event( "SDLK_d" ) );
				break;
				
			case SDLK_a:
				this->m_core->getEventManager().postEvent( Event( "SDLK_a" ) );
				break;

			case SDLK_q:
				this->m_core->stop();
				
			default:
				break;
		}
	}
	
	// TODO only do when not already initialized e.g. by graphics-system
	SDL_PumpEvents();

	//cout << "SDL_process end" << endl;

	return true;
}

bool
SDLInput::finalizeProcess()
{
	return true;
}

bool
SDLInput::sendEvent( Event& e )
{
	return true;
}

ISubSystemEntity*
SDLInput::createEntity( TiXmlElement* cfgNode, IGameObject* parent )
{
	return 0;
}

bool
SDLInput::initSDL( TiXmlElement* configNode )
{
	cout << "Initializing SDL..." << endl;

	int error = SDL_Init( SDL_INIT_JOYSTICK );
	if (error != 0)
	{
		cout << "ERROR ... in SDLInput::initSDL: initializing SDL-Joystick failed" << endl;
		return false;
	}
	else
	{
		cout << "OK ... SDL-Joystick initialized" << endl;
	}

	return true;
}

extern "C"
{	
	__declspec( dllexport ) ISubSystem*
	createInstance ( const char* id, ICore* core )
	{
		return new SDLInput( id, core );
	}

	__declspec( dllexport ) void
	deleteInstance ( ISubSystem* subSys )
	{
		if ( 0 == subSys )
			return;

		if ( 0 == dynamic_cast<SDLInput*>( subSys ) )
			return;

		delete subSys;
	}
}
