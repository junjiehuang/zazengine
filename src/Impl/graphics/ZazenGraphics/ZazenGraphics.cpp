/*
 * ZazenGraphics.cpp
 *
 *  Created on: 06.07.2010
 *      Author: Jonathan Thaler
 */

#include "ZazenGraphics.h"

#include "Geometry/GeometryFactory.h"
#include "Geometry/GeomSkyBox.h"

#include "Material/Material.h"
#include "Texture/TextureFactory.h"

#include "Renderer/DRRenderer.h"

#include "Context/RenderingContext.h"

#include <GL/glew.h>

#include <glm/gtc/matrix_transform.hpp>

#include <boost/algorithm/string.hpp>

#include <Windows.h>

#include <iostream>

#define REQUIRED_MAJOR_OPENGL_VER 3
#define REQUIRED_MINOR_OPENGL_VER 1

using namespace std;
using namespace boost;

ZazenGraphics* ZazenGraphics::instance = NULL;

ZazenGraphics::ZazenGraphics( const std::string& id, ICore* core )
	: m_id ( id ),
	  m_type ( "graphics" ),
	  m_core( core )
{
	this->m_camera = NULL;
	this->m_renderer = NULL;

	ZazenGraphics::instance = this;
}

ZazenGraphics::~ZazenGraphics()
{
	ZazenGraphics::instance = NULL;
}

bool
ZazenGraphics::initialize( TiXmlElement* configNode )
{
	cout << endl << "=============== ZazenGraphics initializing... ===============" << endl;

	if ( false == this->initPipelinePath( configNode ) )
	{
		return false;
	}

	if ( false == this->initModelDataPath( configNode ) )
	{
		return false;
	}

	if ( false == this->initTextureDataPath( configNode ) )
	{
		return false;
	}

	if ( false == this->initMaterialDataPath( configNode ) )
	{
		return false;
	}

	// important: need to initialize window first because only then we have a valid opengl-context
	if ( false == this->createWindow( configNode ) )
	{
		return false;
	}

	TextureFactory::init( this->m_textureDataPath );

	GeometryFactory::setDataPath( this->m_modelDataPath );

	if ( false == Material::init( this->m_materialDataPath ) )
	{
		return false;
	}

	// cannot initialize renderer now because camera not yet loaded
	this->m_renderer = new DRRenderer();

	cout << "================ ZazenGraphics initialized =================" << endl;
	
	return true;
}

bool
ZazenGraphics::shutdown()
{
	cout << endl << "=============== ZazenGraphics shutting down... ===============" << endl;

	this->m_core->getEventManager().unregisterForEvent( "KEY_RELEASED", this );

	std::list<ZazenGraphicsEntity*>::iterator iter = this->m_entities.begin();
	while ( iter != this->m_entities.end() )
	{
		ZazenGraphicsEntity* entity = *iter++;
		delete entity;
	}

	this->m_entities.clear();

	Material::freeAll();
	TextureFactory::freeAll();
	GeometryFactory::freeAll();
	
	RenderingContext::shutdown();

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
	if ( false == this->m_renderer->initialize( this->m_pipelinePath ) )
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
		entity->update();
		entity->queuedEvents.clear();
	}

	MSG		msg;

	if ( PeekMessage( &msg, NULL, 0, 0, PM_REMOVE ) )	// Is There A Message Waiting?
	{
		TranslateMessage( &msg );				// Translate The Message
		DispatchMessage( &msg );				// Dispatch The Message
	}
	else										// If There Are No Messages
	{
		// Draw The Scene.  Watch For ESC Key And Quit Messages From DrawGLScene()
		if ( RenderingContext::getRef().isActive() )								// Program Active?
		{
			flag = this->m_renderer->renderFrame( this->m_instances, this->m_lights );
			RenderingContext::getRef().swapBuffers();
		}
	}

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
			if ( NULL == instance->geom )
			{
				cout << "WARNING ... couldn't get mesh " << str << " - will be ignored" << endl;
			}
		}

		str = instanceNode->Attribute( "material" );
		if ( 0 != str )
		{
			instance->material = Material::get( str );
			if ( NULL == instance->material )
			{
				cout << "WARNING ... couldn't get material " << str << " - will be ignored" << endl;
			}
		}

		if ( NULL != instance->geom && NULL != instance->material )
		{
			entity->m_orientation = instance;

			this->m_instances.push_back( instance );
		}
	}

	TiXmlElement* sceneNode = objectNode->FirstChildElement( "scene" );
	if ( sceneNode )
	{
		TiXmlElement* skyBoxNode = sceneNode->FirstChildElement( "skyBox" );
		if ( skyBoxNode )
		{
			const char* str = skyBoxNode->Attribute( "path" );
			if ( 0 == str )
			{
				cout << "ERROR ... missing path-attribute in skybox node" << endl;
				return false;
			}

			std::string skyFolderPathStr = str;
			filesystem::path fullSkyBoxFolderPath = filesystem::path( m_textureDataPath.generic_string() + skyFolderPathStr );
			if ( ! filesystem::exists( fullSkyBoxFolderPath ) )
			{
				cout << "ERROR ... skybox-path " << fullSkyBoxFolderPath << " does not exist" << endl;
				return false;
			}

			if ( false == filesystem::is_directory( fullSkyBoxFolderPath ) )
			{
				cout << "ERROR ... skybox-path " << fullSkyBoxFolderPath << " is not a directory" << endl;
				return false;
			}

			str = skyBoxNode->Attribute( "format" );
			if ( 0 == str )
			{
				cout << "ERROR ... missing format-attribute in skybox node" << endl;
				return false;
			}

			string skyBoxFormat = str;
			filesystem::path skyBoxFolderPath = filesystem::path( skyFolderPathStr );

			if ( false == GeomSkyBox::initialize( skyBoxFolderPath, skyBoxFormat ) )
			{
				cout << "ERROR ... failed to initialize Sky-Box" << endl;
				return false;
			}
		}
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

			bool castShadow = false;
			float fov = 90.0f;
			int shadowMapResX = 512;
			int shadowMapResY = 512;
			GeomType* boundingGeom = NULL;

			TiXmlElement* shadowingNode = objectNode->FirstChildElement( "shadowing" );
			if ( shadowingNode )
			{
				castShadow = true;

				str = shadowingNode->Attribute( "fov" );
				if ( 0 != str )
				{
					fov = ( float ) atof( str );
				}

				str = shadowingNode->Attribute( "resMapX" );
				if ( 0 != str )
				{
					shadowMapResX = atoi( str );
				}

				str = shadowingNode->Attribute( "resMapY" );
				if ( 0 != str )
				{
					shadowMapResY = atoi( str );
				}
			}

			if ( lightType == "DIRECTIONAL" )
			{
				light = Light::createDirectionalLight( RenderingContext::getRef().getWidth(), RenderingContext::getRef().getHeight(), castShadow );
				boundingGeom = GeometryFactory::createQuad( ( float ) RenderingContext::getRef().getWidth(), ( float ) RenderingContext::getRef().getHeight() );
			}
			// default is spot
			else
			{
				light = Light::createSpotLight( fov, shadowMapResX, shadowMapResY, castShadow );
				// TODO load correct bounding-geometry
				boundingGeom = GeometryFactory::createQuad( ( float ) RenderingContext::getRef().getWidth(), ( float ) RenderingContext::getRef().getHeight() );
			}

			light->setBoundingGeometry( boundingGeom );

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
				fov = ( float ) atof( str );
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

			Viewer* camera = new Viewer( RenderingContext::getRef().getWidth(), RenderingContext::getRef().getHeight() );
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
		delete entity;
		return 0;
	}

	TiXmlElement* orientationNode = objectNode->FirstChildElement( "orientation" );
	if ( orientationNode )
	{
		glm::vec3 v;
		float roll = 0.0f;
		float pitch = 0.0f;
		float heading = 0.0f;
		float scale = 1.0f;

		const char* str = orientationNode->Attribute( "x" );
		if ( 0 != str )
		{
			v[ 0 ] = ( float ) atof( str );
		}

		str = orientationNode->Attribute( "y" );
		if ( 0 != str )
		{
			v[ 1 ] = ( float ) atof( str );
		}

		str = orientationNode->Attribute( "z" );
		if ( 0 != str )
		{
			v[ 2 ] = ( float ) atof( str );
		}

		str = orientationNode->Attribute( "heading" );
		if ( 0 == str )
		{
			cout << "INFO ... heading attribute missing in orientation - use default " << endl;
		}
		else
		{
			heading = ( float ) atof( str );
		}

		str = orientationNode->Attribute( "roll" );
		if ( 0 == str )
		{
			cout << "INFO ... roll attribute missing in orientation - use default " << endl;
		}
		else
		{
			roll = ( float ) atof( str );
		}

		str = orientationNode->Attribute( "pitch" );
		if ( 0 == str )
		{
			cout << "INFO ... pitch attribute missing in orientation - use default " << endl;
		}
		else
		{
			pitch = ( float ) atof( str );
		}

		str = orientationNode->Attribute( "scale" );
		if ( 0 == str )
		{
			cout << "INFO ... scale attribute missing in orientation - use default " << endl;
		}
		else
		{
			scale = ( float ) atof( str );
		}

		entity->m_orientation->set( v, pitch, heading, roll, scale );
	}

	TiXmlElement* animationNode = objectNode->FirstChildElement( "animation" );
	if ( animationNode )
	{
		glm::vec3 animRot;

		const char* str = animationNode->Attribute( "heading" );
		if ( 0 == str )
		{
			cout << "INFO ... heading attribute missing in animation - use default " << endl;
		}
		else
		{
			animRot[ 0 ] = ( float ) atof( str );
		}

		str = animationNode->Attribute( "roll" );
		if ( 0 == str )
		{
			cout << "INFO ... roll attribute missing in animation - use default " << endl;
		}
		else
		{
			animRot[ 1 ] = ( float ) atof( str );
		}

		str = animationNode->Attribute( "pitch" );
		if ( 0 == str )
		{
			cout << "INFO ... pitch attribute missing in animation - use default " << endl;
		}
		else
		{
			animRot[ 2 ] = ( float ) atof( str );
		}

		entity->setAnimation( animRot[ 0 ], animRot[ 1 ], animRot[ 2 ] ); 
	}

	this->m_entities.push_back( entity );

	return entity;
}

void*
ZazenGraphics::getWindowHandle()
{
	return RenderingContext::getRef().getHandle();
}

bool
ZazenGraphics::toggleFullscreen()
{
	RenderingContext::getRef().toggleFullscreen();

	/* TODO fix it, not yet working */
	this->m_renderer->shutdown();
	this->m_renderer->initialize( this->m_pipelinePath );

	return true;
}

bool
ZazenGraphics::createWindow( TiXmlElement* configNode )
{
	int windowWidth = 1024;
	int windowHeight = 768;
	bool fullScreen = false;
	string windowTitle = "zaZengine";	

	TiXmlElement* windowNode = configNode->FirstChildElement( "window" );
	if ( 0 != windowNode )
	{
		const char* str = windowNode->Attribute( "fullscreen" );
		if ( 0 != str )
		{
			if ( boost::iequals( str, "true" ) )
			{
				fullScreen = true;
			}
		}

		str = windowNode->Attribute( "width" );
		if ( 0 != str )
		{
			windowWidth = atoi( str );
		}

		str = windowNode->Attribute( "height" );
		if ( 0 != str )
		{
			windowHeight = atoi( str );
		}

		str = windowNode->Attribute( "title" );
		if ( 0 != str )
		{
			windowTitle = str;
		}
	}

	if ( false == RenderingContext::initialize( windowTitle, windowWidth, windowHeight, fullScreen ) )
	{
		cout << "ERROR ... in ZazenGraphics::createWindow: failed creating window" << endl;
		return false;
	}

	return true;
}

bool
ZazenGraphics::initPipelinePath( TiXmlElement* configElem )
{
	TiXmlElement* pipelineNode = configElem->FirstChildElement( "pipeline" );
	if ( 0 == pipelineNode )
	{
		cout << "ERROR ... missing pipeline-config in graphics-config" << endl;
		return false;
	}

	const char* str = pipelineNode->Attribute( "path" );
	if ( 0 == str )
	{
		cout << "ERROR ... missing pipeline-path in graphics-config" << endl;
		return false;
	}

	this->m_pipelinePath = filesystem::path( str );
	if ( ! filesystem::exists( this->m_pipelinePath ) )
	{
		cout << "ERROR ... pipeline-path " << this->m_pipelinePath << " does not exist" << endl;
		return false;
	}

	if ( false == filesystem::is_directory( this->m_pipelinePath ) )
	{
		cout << "ERROR ... pipeline-path " << this->m_pipelinePath << " is not a directory" << endl;
		return false;
	}

	return true;
}

bool
ZazenGraphics::initModelDataPath( TiXmlElement* configElem )
{
	TiXmlElement* modelDataNode = configElem->FirstChildElement( "modelData" );
	if ( 0 == modelDataNode )
	{
		cout << "ERROR ... missing modelData-config in graphics-config" << endl;
		return false;
	}

	const char* str = modelDataNode->Attribute( "path" );
	if ( 0 == str )
	{
		cout << "ERROR ... missing modelData-path in graphics-config" << endl;
		return false;
	}

	this->m_modelDataPath = filesystem::path( str );
	if ( ! filesystem::exists( this->m_modelDataPath ) )
	{
		cout << "ERROR ... modelData-path " << this->m_modelDataPath << " does not exist" << endl;
		return false;
	}

	if ( false == filesystem::is_directory( this->m_modelDataPath ) )
	{
		cout << "ERROR ... modelData-path " << this->m_modelDataPath << " is not a directory" << endl;
		return false;
	}

	return true;
}

bool
ZazenGraphics::initTextureDataPath( TiXmlElement* configElem )
{
	TiXmlElement* textureDataNode = configElem->FirstChildElement( "textureData" );
	if ( 0 == textureDataNode )
	{
		cout << "ERROR ... missing textureData-config in graphics-config" << endl;
		return false;
	}

	const char* str = textureDataNode->Attribute( "path" );
	if ( 0 == str )
	{
		cout << "ERROR ... missing textureData-path in graphics-config" << endl;
		return false;
	}

	this->m_textureDataPath = filesystem::path( str );
	if ( ! filesystem::exists( this->m_textureDataPath ) )
	{
		cout << "ERROR ... textureData-path " << this->m_textureDataPath << " does not exist" << endl;
		return false;
	}

	if ( false == filesystem::is_directory( this->m_textureDataPath ) )
	{
		cout << "ERROR ... textureData-path " << this->m_textureDataPath << " is not a directory" << endl;
		return false;
	}

	return true;
}

bool
ZazenGraphics::initMaterialDataPath( TiXmlElement* configElem )
{
	TiXmlElement* materialDataNode = configElem->FirstChildElement( "materialData" );
	if ( 0 == materialDataNode )
	{
		cout << "ERROR ... missing materialData-config in graphics-config" << endl;
		return false;
	}

	const char* str = materialDataNode->Attribute( "path" );
	if ( 0 == str )
	{
		cout << "ERROR ... missing materialData-path in graphics-config" << endl;
		return false;
	}

	this->m_materialDataPath = filesystem::path( str );
	if ( ! filesystem::exists( this->m_materialDataPath ) )
	{
		cout << "ERROR ... materialData-path " << this->m_materialDataPath << " does not exist" << endl;
		return false;
	}

	if ( false == filesystem::is_directory( this->m_materialDataPath ) )
	{
		cout << "ERROR ... materialData-path " << this->m_materialDataPath << " is not a directory" << endl;
		return false;
	}

	return true;
}

bool
ZazenGraphics::initSkyBoxFolderPath( TiXmlElement* configElem )
{
	TiXmlElement* materialDataNode = configElem->FirstChildElement( "materialData" );
	if ( 0 == materialDataNode )
	{
		cout << "ERROR ... missing materialData-config in graphics-config" << endl;
		return false;
	}

	const char* str = materialDataNode->Attribute( "path" );
	if ( 0 == str )
	{
		cout << "ERROR ... missing materialData-path in graphics-config" << endl;
		return false;
	}

	this->m_materialDataPath = filesystem::path( str );
	if ( ! filesystem::exists( this->m_materialDataPath ) )
	{
		cout << "ERROR ... materialData-path " << this->m_materialDataPath << " does not exist" << endl;
		return false;
	}

	if ( false == filesystem::is_directory( this->m_materialDataPath ) )
	{
		cout << "ERROR ... materialData-path " << this->m_materialDataPath << " is not a directory" << endl;
		return false;
	}

	return true;
}

extern "C"
{	
	__declspec( dllexport ) ISubSystem*
	createInstance ( const char* id, ICore* core )
	{
		return new ZazenGraphics( id, core );
	}

	__declspec( dllexport ) void
	deleteInstance ( ISubSystem* subSys )
	{
		if ( 0 == subSys )
			return;

		if ( 0 == dynamic_cast<ZazenGraphics*>( subSys ) )
			return;

		delete subSys;
	}
}