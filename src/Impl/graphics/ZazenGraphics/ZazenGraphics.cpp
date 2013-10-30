/*
 * ZazenGraphics.cpp
 *
 *  Created on: 06.07.2010
 *      Author: Jonathan Thaler
 */

#include "ZazenGraphics.h"

#include "Program/ProgramManagement.h"
#include "Program/UniformManagement.h"

#include "Geometry/GeometryFactory.h"
#include "Geometry/GeomSkyBox.h"

#include "Animation/AnimationFactory.h"
#include "Material/MaterialFactory.h"
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
	this->m_logger = this->m_core->getLogger( "zaZenGraphics" );
	this->m_logger->logInfo( "=============== ZazenGraphics initializing... ===============" );

	if ( false == this->initPipelinePath( configNode ) )
	{
		return false;
	}

	if ( false == this->initModelDataPath( configNode ) )
	{
		return false;
	}

	if ( false == this->initAnimationDataPath( configNode ) )
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

	this->initScreenShotPath( configNode );

	// important: need to initialize window first because only then we have a valid opengl-context
	if ( false == this->createWindow( configNode ) )
	{
		return false;
	}

	ProgramManagement::init( this->m_pipelinePath );
	TextureFactory::init( this->m_textureDataPath );

	GeometryFactory::setDataPath( this->m_modelDataPath );
	AnimationFactory::setDataPath( this->m_animationDataPath );

	if ( false == MaterialFactory::init( this->m_materialDataPath ) )
	{
		return false;
	}

	// cannot initialize renderer now because camera not yet loaded
	this->m_renderer = new DRRenderer();

	this->m_logger->logInfo( "================ ZazenGraphics initialized =================" );

	return true;
}

bool
ZazenGraphics::shutdown()
{
	this->m_logger->logInfo( "=============== ZazenGraphics shutting down... ===============" );

	this->m_core->getEventManager().unregisterForEvent( "KEY_RELEASED", this );

	std::list<ZazenGraphicsEntity*>::iterator iter = this->m_entities.begin();
	while ( iter != this->m_entities.end() )
	{
		ZazenGraphicsEntity* entity = *iter++;
		delete entity;
	}

	this->m_entities.clear();

	MaterialFactory::freeAll();
	TextureFactory::freeAll();
	GeometryFactory::freeAll();
	GeomSkyBox::shutdown();
	UniformManagement::freeAllBlocks();
	ProgramManagement::freeAll();

	if ( NULL != this->m_renderer )
	{
		this->m_renderer->shutdown();
	}

	RenderingContext::shutdown();

	this->m_logger->logInfo( "================ ZazenGraphics shutdown =================" );

	return true;
}

bool
ZazenGraphics::start()
{
	if ( 0 == this->m_camera )
	{
		this->m_logger->logError( "missing camera in ZazenGraphics - exit" );
		return false;
	}

	this->m_renderer->setCamera( this->m_camera );
	if ( false == this->m_renderer->initialize() )
	{
		this->m_logger->logError( "initializing renderer failed - exit" );
		return false;
	}

	this->m_core->getEventManager().registerForEvent( "KEY_PRESSED", this );

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

	// process events of m_entities
	std::list<ZazenGraphicsEntity*>::iterator iter = this->m_entities.begin();
	while ( iter != this->m_entities.end() )
	{
		ZazenGraphicsEntity* entity = *iter++;
		entity->update();
		entity->queuedEvents.clear();
	}

	MSG msg;

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

	return flag;
}

bool
ZazenGraphics::finalizeProcess()
{
	return true;
}

bool
ZazenGraphics::sendEvent( Event& e )
{
	if ( e == "KEY_PRESSED" )
	{
		int keyCode = any_cast<int>( e.getValue( "key" ) );
		// F12 triggers screen-shot
		if ( 88 == keyCode )
		{
			RenderingContext::getRef().takeScreenShot();
		}
	}

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
				this->m_logger->logWarning() << "couldn't get mesh " << str << " - will be ignored";
			}
		}

		str = instanceNode->Attribute( "material" );
		if ( 0 != str )
		{
			instance->material = MaterialFactory::get( str );
			if ( NULL == instance->material )
			{
				this->m_logger->logWarning() << "couldn't get material " << str << " - will be ignored";
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
				this->m_logger->logError() << "missing path-attribute in skybox node";
				return false;
			}

			std::string skyFolderPathStr = str;
			filesystem::path fullSkyBoxFolderPath = filesystem::path( m_textureDataPath.generic_string() + skyFolderPathStr );
			if ( ! filesystem::exists( fullSkyBoxFolderPath ) )
			{
				this->m_logger->logError() << "skybox-path " << fullSkyBoxFolderPath << " does not exist";
				return false;
			}

			if ( false == filesystem::is_directory( fullSkyBoxFolderPath ) )
			{
				this->m_logger->logError() << "skybox-path " << fullSkyBoxFolderPath << " is not a directory";
				return false;
			}

			str = skyBoxNode->Attribute( "format" );
			if ( 0 == str )
			{
				this->m_logger->logError( "missing format-attribute in skybox node" );
				return false;
			}

			string skyBoxFormat = str;
			filesystem::path skyBoxFolderPath = filesystem::path( skyFolderPathStr );

			if ( false == GeomSkyBox::initialize( skyBoxFolderPath, skyBoxFormat ) )
			{
				this->m_logger->logError( "failed to initialize Sky-Box" );
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
				this->m_logger->logWarning() << "fov attribute missing in cameraNode - use default: " << fov;
			}
			else
			{
				fov = ( float ) atof( str );
			}

			str = cameraNode->Attribute( "view" );
			if ( 0 == str )
			{
				this->m_logger->logWarning() << "view attribute missing in cameraNode - use default: " << str;
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
		this->m_logger->logError() << "No valid entity defined in ZazenGraphics for Object \"" << parent->getName() << "\" - error ";

		delete entity;
		return NULL;
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
			this->m_logger->logWarning( "heading attribute missing in orientation - use default: 0.0" ); 
		}
		else
		{
			heading = ( float ) atof( str );
		}

		str = orientationNode->Attribute( "roll" );
		if ( 0 == str )
		{
			this->m_logger->logWarning( "roll attribute missing in orientation - use default: 0.0" );
		}
		else
		{
			roll = ( float ) atof( str );
		}

		str = orientationNode->Attribute( "pitch" );
		if ( 0 == str )
		{
			this->m_logger->logWarning( "pitch attribute missing in orientation - use default: 0.0" );
		}
		else
		{
			pitch = ( float ) atof( str );
		}

		str = orientationNode->Attribute( "scale" );
		if ( 0 == str )
		{
			this->m_logger->logWarning( "scale attribute missing in orientation - use default: 1.0" );
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
		const char* str = animationNode->Attribute( "directory" );
		if ( 0 != str )
		{
			string directory = str;
			
			str = animationNode->Attribute( "extension" );
			if ( 0 != str )
			{
				string extension = str;

				entity->m_allAnimations = AnimationFactory::loadDir( directory, extension );

				str = animationNode->Attribute( "active" );
				if ( 0 != str )
				{
					string fileName = directory + "/" + str + extension;

					entity->m_activeAnimation = AnimationFactory::get( fileName );
				}
			}
		}
		else
		{
			glm::vec3 animRot;

			const char* str = animationNode->Attribute( "heading" );
			if ( 0 == str )
			{
				this->m_logger->logWarning( "heading attribute missing in animation - use default: 0.0" );
			}
			else
			{
				animRot[ 0 ] = ( float ) atof( str );
			}

			str = animationNode->Attribute( "roll" );
			if ( 0 == str )
			{
				this->m_logger->logWarning( "roll attribute missing in animation - use default: 0.0" );
			}
			else
			{
				animRot[ 1 ] = ( float ) atof( str );
			}

			str = animationNode->Attribute( "pitch" );
			if ( 0 == str )
			{
				this->m_logger->logWarning( "pitch attribute missing in animation - use default: 0.0" );
			}
			else
			{
				animRot[ 2 ] = ( float ) atof( str );
			}

			entity->setAnimation( animRot[ 0 ], animRot[ 1 ], animRot[ 2 ] ); 
		}
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
	this->m_renderer->initialize();

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

	if ( false == RenderingContext::initialize( windowTitle, windowWidth, windowHeight, fullScreen, this->m_screenShotPath ) )
	{
		this->m_logger->logError( "in ZazenGraphics::createWindow: failed creating window" );
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
		this->m_logger->logError( "missing pipeline-config in graphics-config" );
		return false;
	}

	const char* str = pipelineNode->Attribute( "path" );
	if ( 0 == str )
	{
		this->m_logger->logError( "missing pipeline-path in graphics-config" );
		return false;
	}

	this->m_pipelinePath = filesystem::path( str );
	if ( ! filesystem::exists( this->m_pipelinePath ) )
	{
		this->m_logger->logError() << "pipeline-path " << this->m_pipelinePath << " does not exist";
		return false;
	}

	if ( false == filesystem::is_directory( this->m_pipelinePath ) )
	{
		this->m_logger->logError() << "pipeline-path " << this->m_pipelinePath << " is not a directory";
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
		this->m_logger->logError( "missing modelData-config in graphics-config" );
		return false;
	}

	const char* str = modelDataNode->Attribute( "path" );
	if ( 0 == str )
	{
		this->m_logger->logError( "missing modelData-path in graphics-config" );
		return false;
	}

	this->m_modelDataPath = filesystem::path( str );
	if ( ! filesystem::exists( this->m_modelDataPath ) )
	{
		this->m_logger->logError() << "modelData-path " << this->m_modelDataPath << " does not exist";
		return false;
	}

	if ( false == filesystem::is_directory( this->m_modelDataPath ) )
	{
		this->m_logger->logError() << "modelData-path " << this->m_modelDataPath << " is not a directory";
		return false;
	}

	return true;
}

bool
ZazenGraphics::initAnimationDataPath( TiXmlElement* configElem )
{
	TiXmlElement* animationDataNode = configElem->FirstChildElement( "animationData" );
	if ( 0 == animationDataNode )
	{
		this->m_logger->logError( "missing animationData-config in graphics-config" );
		return false;
	}

	const char* str = animationDataNode->Attribute( "path" );
	if ( 0 == str )
	{
		this->m_logger->logError( "missing animationData-path in graphics-config" );
		return false;
	}

	this->m_animationDataPath = filesystem::path( str );
	if ( ! filesystem::exists( this->m_animationDataPath ) )
	{
		this->m_logger->logError() << "animationData-path " << this->m_animationDataPath << " does not exist";
		return false;
	}

	if ( false == filesystem::is_directory( this->m_animationDataPath ) )
	{
		this->m_logger->logError() << "animationData-path " << this->m_animationDataPath << " is not a directory";
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
		this->m_logger->logError( "missing textureData-config in graphics-config" );
		return false;
	}

	const char* str = textureDataNode->Attribute( "path" );
	if ( 0 == str )
	{
		this->m_logger->logError( "missing textureData-path in graphics-config" );
		return false;
	}

	this->m_textureDataPath = filesystem::path( str );
	if ( ! filesystem::exists( this->m_textureDataPath ) )
	{
		this->m_logger->logError() << "textureData-path " << this->m_textureDataPath << " does not exist";
		return false;
	}

	if ( false == filesystem::is_directory( this->m_textureDataPath ) )
	{
		this->m_logger->logError() << "textureData-path " << this->m_textureDataPath << " is not a directory";
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
		this->m_logger->logError( "missing materialData-config in graphics-config" );
		return false;
	}

	const char* str = materialDataNode->Attribute( "path" );
	if ( 0 == str )
	{
		this->m_logger->logError( "missing materialData-path in graphics-config" );
		return false;
	}

	this->m_materialDataPath = filesystem::path( str );
	if ( ! filesystem::exists( this->m_materialDataPath ) )
	{
		this->m_logger->logError() << "materialData-path " << this->m_materialDataPath << " does not exist";
		return false;
	}

	if ( false == filesystem::is_directory( this->m_materialDataPath ) )
	{
		this->m_logger->logError() << "materialData-path " << this->m_materialDataPath << " is not a directory";
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
		this->m_logger->logError( "missing materialData-config in graphics-config" );
		return false;
	}

	const char* str = materialDataNode->Attribute( "path" );
	if ( 0 == str )
	{
		this->m_logger->logError( "missing materialData-path in graphics-config" );
		return false;
	}

	this->m_materialDataPath = filesystem::path( str );
	if ( ! filesystem::exists( this->m_materialDataPath ) )
	{
		this->m_logger->logError() << "materialData-path " << this->m_materialDataPath << " does not exist";
		return false;
	}

	if ( false == filesystem::is_directory( this->m_materialDataPath ) )
	{
		this->m_logger->logError() << "materialData-path " << this->m_materialDataPath << " is not a directory";
		return false;
	}

	return true;
}

void
ZazenGraphics::initScreenShotPath( TiXmlElement* configElem )
{
	TiXmlElement* screenShotsNode = configElem->FirstChildElement( "screenShots" );
	if ( 0 == screenShotsNode )
	{
		return;
	}

	const char* str = screenShotsNode->Attribute( "path" );
	if ( 0 == str )
	{
		return;
	}

	this->m_screenShotPath = filesystem::path( str );
	if ( ! filesystem::exists( this->m_screenShotPath ) )
	{
		this->m_logger->logWarning() << "screenShots-path " << this->m_screenShotPath << " does not exist - setting default";
		return;
	}

	if ( false == filesystem::is_directory( this->m_screenShotPath ) )
	{
		this->m_logger->logWarning() << "screenShots-path " << this->m_screenShotPath << " is not a directory - setting default";
		return;
	}
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