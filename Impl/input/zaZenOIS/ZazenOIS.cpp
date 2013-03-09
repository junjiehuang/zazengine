/*
 *  ZazenOIS.cpp
 *  Zazengine
 *
 *  Created by Jonathan Thaler on 09.03.13
 *
 */

#include "ZazenOIS.h"

#include <iostream>

using namespace std;

ZazenOIS::ZazenOIS( const std::string& id, ICore* core )
	: id ( id ),
	type ( "input"),
	m_core( core )
{
	
}

ZazenOIS::~ZazenOIS()
{
	
}

bool
ZazenOIS::initialize( TiXmlElement* element )
{
	cout << endl << "=============== ZazenOIS initializing... ===============" << endl;

	cout << "================ ZazenOIS initialized =================" << endl;

	return true;
}

bool
ZazenOIS::shutdown()
{
	cout << endl << "=============== ZazenOIS shutting down... ===============" << endl;

	cout << "================ ZazenOIS shutdown =================" << endl;

	return true;
}

bool
ZazenOIS::start()
{
	return true;
}

bool
ZazenOIS::stop()
{
	return true;
}

bool
ZazenOIS::pause()
{
	return true;
}

bool
ZazenOIS::process(double factor)
{
	//cout << "SDLInput::process" << endl;
	/*

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
	*/

	//cout << "SDL_process end" << endl;

	return true;
}

bool
ZazenOIS::finalizeProcess()
{
	return true;
}

bool
ZazenOIS::sendEvent( Event& e )
{
	return true;
}

ZazenOISEntity*
ZazenOIS::createEntity( TiXmlElement* cfgNode, IGameObject* parent )
{
	// TODO implement when necessary
	return 0;
}

extern "C"
{	
	__declspec( dllexport ) ISubSystem*
	createInstance ( const char* id, ICore* core )
	{
		return new ZazenOIS( id, core );
	}

	__declspec( dllexport ) void
	deleteInstance ( ISubSystem* subSys )
	{
		if ( 0 == subSys )
			return;

		if ( 0 == dynamic_cast<ZazenOIS*>( subSys ) )
			return;

		delete subSys;
	}
}
