/*
 *  SDLInput.cpp
 *  PlayGround
 *
 *  Created by Jonathan Thaler on 17.07.10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "SDLInput.h"

#include "../../../Core/Core.h"

#include "../../../Core/ScriptSystem/ScriptSystem.h"

#include <iostream>

#include <SDL/SDL.h>

using namespace std;

SDLInput::SDLInput()
	: id ( "SDLInput" ),
	type ("input")
{
	
}

SDLInput::~SDLInput()
{
	
}

bool
SDLInput::initialize( TiXmlElement* )
{
	cout << endl << "=============== SDLInput initializing... ===============" << endl;

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
SDLInput::process(double factor)
{
	//cout << "SDLInput::process" << endl;

	SDL_Event event;
	
	if (SDL_PollEvent(&event))
	{
		if (event.type == SDL_KEYDOWN)
		{
			ScriptSystem::getInstance().callFunc( "onKeyDown" );

			pressedKeys.push_back(event.key.keysym.sym);
			
		}
		else if (event.type == SDL_KEYUP)
		{
			ScriptSystem::getInstance().callFunc( "onKeyUp" );

			pressedKeys.remove(event.key.keysym.sym);
			
		}
		else if (event.type == SDL_MOUSEMOTION)
		{
			//this->camera->changeHeading(event.motion.xrel * loopFactor * 0.01);
			//this->camera->changePitch(event.motion.yrel * loopFactor * 0.01);
			
		}
		else if (event.type == SDL_QUIT)
		{
			Core::getInstance().stop();
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
				Core::getInstance().getEventManager().postEvent( Event( "SDLK_RIGHT" ) );
				break;
			}
			case SDLK_LEFT:
				Core::getInstance().getEventManager().postEvent( Event( "SDLK_LEFT" ) );
				break;
				
			case SDLK_UP:
				Core::getInstance().getEventManager().postEvent( Event( "SDLK_UP" ) );
				break;
				
			case SDLK_DOWN:
				Core::getInstance().getEventManager().postEvent( Event( "SDLK_DOWN" ) );
				break;
				
			case SDLK_w:
				Core::getInstance().getEventManager().postEvent( Event( "SDLK_w" ) );
				break;
				
			case SDLK_s:
				Core::getInstance().getEventManager().postEvent( Event( "SDLK_s" ) );
				break;
				
			case SDLK_d:
				Core::getInstance().getEventManager().postEvent( Event( "SDLK_d" ) );
				break;
				
			case SDLK_a:
				Core::getInstance().getEventManager().postEvent( Event( "SDLK_a" ) );
				break;

			case SDLK_q:
				Core::getInstance().stop();
				
			default:
				break;
		}
	}
	
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
