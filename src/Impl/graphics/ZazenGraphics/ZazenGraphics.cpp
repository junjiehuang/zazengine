/*
 * ZazenGraphics.cpp
 *
 *  Created on: 06.07.2010
 *      Author: joni
 */

#include "ZazenGraphics.h"

#include <GL/glew.h>
#include <glm/gtc/matrix_transform.hpp>

#include "Geometry/GeometryFactory.h"
#include "Material/Material.h"

#include "Renderer/DRRenderer.h"

#include "../../../Core/Core.h"

#include <iostream>

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600

using namespace std;

ZazenGraphics::ZazenGraphics()
	: m_id ("ZazenGraphics"),
	  m_type ("graphics")
{
}

ZazenGraphics::~ZazenGraphics()
{
}

bool
ZazenGraphics::initialize( TiXmlElement* configNode )
{
	cout << endl << "=============== ZazenGraphics initializing... ===============" << endl;

	if ( false == this->initSDL() )
	{
		return false;
	}

	if ( false == this->initGL() )
	{
		return false;
	}
	
	// cannot initialize renderer now because camera not yet loaded
	this->m_renderer = new DRRenderer();

	if ( false == Material::loadAll() )
	{
		cout << "Coulnd't load materials - exit" << endl;
		return false;
	}

	cout << "================ ZazenGraphics initialized =================" << endl;
	
	return true;
}

bool
ZazenGraphics::shutdown()
{
	cout << endl << "=============== ZazenGraphics shutting down... ===============" << endl;

	Core::getInstance().getEventManager().unregisterForEvent( "SDLK_RIGHT", this );

	std::list<ZazenGraphicsEntity*>::iterator iter = this->m_entities.begin();
	while ( iter != this->m_entities.end() )
	{
		ZazenGraphicsEntity* entity = *iter++;
		delete entity;
	}

	this->m_entities.clear();

	Material::freeAll();
	Texture::freeAll();
	GeometryFactory::freeAll();
	
	SDL_Quit();

	cout << "================ ZazenGraphics shutdown =================" << endl;

	return true;
}

bool
ZazenGraphics::start()
{
	if ( 0 == this->m_camera )
	{
		cout << "ERROR ... missing camera in ZazenGraphics - exit" << endl;
		return false;
	}

	this->m_renderer->setCamera( this->m_camera );
	if ( false == this->m_renderer->initialize() )
	{
		cout << "ERROR ... initializing renderer failed - exit" << endl;
		return false;
	}

	return true;
}

bool
ZazenGraphics::stop()
{
	return true;
}

bool
ZazenGraphics::pause()
{
	return true;
}

bool
ZazenGraphics::process( double iterationFactor )
{
	bool flag = true;
	//cout << "ZazenGraphics::process enter" << endl;

	// process events of m_entities
	std::list<ZazenGraphicsEntity*>::iterator iter = this->m_entities.begin();
	while ( iter != this->m_entities.end() )
	{
		ZazenGraphicsEntity* entity = *iter++;

		std::list<Event>::iterator eventsIter = entity->queuedEvents.begin();
		while ( eventsIter != entity->queuedEvents.end() )
		{
			//Event& e = *eventsIter++;

			//cout << "received Event '" << e.getID() << "' in ZazenGraphics from GO '" << entity->getParent()->getName() << endl;
		}

		entity->queuedEvents.clear();
	}

	this->m_lastItFact = iterationFactor;
	flag = this->m_renderer->renderFrame( this->m_instances, this->m_lights );

	//cout << "ZazenGraphics::process leave" << endl;

	return flag;
}

bool
ZazenGraphics::finalizeProcess()
{
	//cout << "ZazenGraphics::finalizeProcess" << endl;

	return true;
}

bool
ZazenGraphics::sendEvent( Event& e )
{
	return true;
}

ZazenGraphicsEntity*
ZazenGraphics::createEntity( TiXmlElement* objectNode, IGameObject* parent )
{
	ZazenGraphicsEntity* entity = new ZazenGraphicsEntity( parent );

	TiXmlElement* instanceNode = objectNode->FirstChildElement( "instance" );
	if ( instanceNode )
	{
		Instance* instance = new Instance();

		const char* str = instanceNode->Attribute( "mesh" );
		if ( 0 != str )
		{
			instance->geom = GeometryFactory::get( str );
		}

		str = instanceNode->Attribute( "material" );
		if ( 0 != str )
		{
			instance->material = Material::get( str );
		}

		entity->m_orientation = instance;

		this->m_instances.push_back( instance );
	}

	if ( 0 == entity->m_orientation )
	{
		TiXmlElement* lightNode = objectNode->FirstChildElement( "light" );
		if ( lightNode )
		{
			std::string lightType = "SPOT";

			const char* str = lightNode->Attribute( "type" );
			if ( 0 != str )
			{
				lightType = str;
			}

			Light* light = 0;

			if ( lightType == "DIRECTIONAL" )
			{
				light = Light::createDirectionalLight( WINDOW_WIDTH, WINDOW_HEIGHT );

			}
			else if ( lightType == "POINT" )
			{
				light = Light::createPointLight( WINDOW_HEIGHT );
			}
			// default is spot
			else
			{
				light = Light::createSpoptLight( 90, WINDOW_WIDTH, WINDOW_HEIGHT );
			}

			entity->m_orientation = light;
			this->m_lights.push_back( light );
		}
	}

	if ( 0 == entity->m_orientation )
	{
		TiXmlElement* cameraNode = objectNode->FirstChildElement( "camera" );
		if ( cameraNode )
		{
			float fov = 90.0f;
			std::string mode = "PROJ";

			const char* str = cameraNode->Attribute( "fov" );
			if ( 0 == str )
			{
				cout << "INFO ... fov attribute missing in cameraNode - use default " << endl;
			}
			else
			{
				fov = atof( str );
			}

			str = cameraNode->Attribute( "view" );
			if ( 0 == str )
			{
				cout << "INFO ... view attribute missing in cameraNode - use default " << endl;
			}
			else
			{
				mode = str;
			}

			Viewer* camera = new Viewer( WINDOW_WIDTH, WINDOW_HEIGHT );
			if ( mode == "PROJ" )
			{
				camera->setFov( fov );
				camera->setupPerspective();
			}
			else if ( mode == "ORTHO" )
			{
				camera->setupOrtho();
			}

			entity->m_orientation = camera;
			this->m_camera = camera;
		}
	}

	if ( 0 == entity->m_orientation )
	{
		cout << "No valid entity defined in ZazenGraphics for Object \"" << parent->getName() << "\" - error " << endl;
		return 0;
	}

	TiXmlElement* orientationNode = objectNode->FirstChildElement( "orientation" );
	if ( orientationNode )
	{
		glm::vec3 v;
		float roll = 0.0f;
		float pitch = 0.0f;
		float heading = 0.0f;

		const char* str = orientationNode->Attribute( "x" );
		if ( 0 != str )
		{
			v[ 0 ] = atof( str );
		}

		str = orientationNode->Attribute( "y" );
		if ( 0 != str )
		{
			v[ 1 ] = atof( str );
		}

		str = orientationNode->Attribute( "z" );
		if ( 0 != str )
		{
			v[ 2 ] = atof( str );
		}

		str = orientationNode->Attribute( "heading" );
		if ( 0 == str )
		{
			cout << "INFO ... heading attribute missing in orientation - use default " << endl;
		}
		else
		{
			heading = atof( str );
		}

		str = orientationNode->Attribute( "roll" );
		if ( 0 == str )
		{
			cout << "INFO ... roll attribute missing in orientation - use default " << endl;
		}
		else
		{
			roll = atof( str );
		}

		str = orientationNode->Attribute( "pitch" );
		if ( 0 == str )
		{
			cout << "INFO ... pitch attribute missing in orientation - use default " << endl;
		}
		else
		{
			pitch = atof( str );
		}

		entity->m_orientation->set( v, pitch, heading, roll );
	}

	this->m_entities.push_back( entity );

	return entity;
}


bool
ZazenGraphics::initSDL()
{
	cout << "Initializing SDL..." << endl;
	int error = SDL_Init(SDL_INIT_VIDEO);
	if (error != 0)
	{
		cout << "FAILED ... Initializing SDL failed - exit..." << endl;
		return false;
	}
	else
	{
		cout << "OK ... SDL initialized" << endl;
	}

	if (SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1) == -1)
	{
		cout << "FAILED ... SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER) failed with " << SDL_GetError() << endl;
		return false;
	}

	this->m_drawContext = SDL_SetVideoMode( WINDOW_WIDTH, WINDOW_HEIGHT, 32, SDL_OPENGL /*| SDL_FULLSCREEN*/);
	if ( 0 == this->m_drawContext )
	{
		cout << "FAILED ... SDL_SetVideoMode failed with " << SDL_GetError() << endl;
		return false;
	}

	cout << "OK ... Videocontext created" << endl;

	return true;
}

bool
ZazenGraphics::initGL()
{
	//int argc = 0;
	//glutInit(&argc, NULL);

	int major, minor;
	GLenum err = glewInit();
	if ( GLEW_OK != err )
	{
		cout << "ERROR ... GLEW failed with " <<  glewGetErrorString(err) << endl;
		return false;
	}
	else
	{
		cout << "OK ... GLEW " << glewGetString(GLEW_VERSION) << " initialized " << endl;
	}

	cout << "OpenGL Version: " << glGetString(GL_VERSION) << endl;

	glGetIntegerv(GL_MAJOR_VERSION, &major); // major = 3
	glGetIntegerv(GL_MINOR_VERSION, &minor); // minor = 2

	// opengl 3.3 minimum
	if ( 3 > major )
	{
		cout << "ERROR ... OpenGL3.3 or above required" << endl;
		return false;
	}
	else if ( 3 == major )
	{
		if ( 3 > minor )
		{
			cout << "ERROR ... OpenGL3.3 or above required" << endl;
			return false;
		}
	}

	return true;
}
