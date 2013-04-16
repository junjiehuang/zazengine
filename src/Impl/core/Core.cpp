/*
 * Core.cpp
 *
 *  Created on: 27.06.2010
 *      Author: Jonathan Thaler
 */

#include "Core.h"

#include "Logger.h"

#include "ZazenGameObject.h"
#include "ScriptSystem.h"

#include <windows.h>

#include <iostream>
#include <sstream>

#include <log4cplus/configurator.h>

using namespace std;

Core* Core::instance = 0;

bool
Core::initalize( const std::string& configPath, IGameObjectFactory* gameObjectFactory )
{
	if ( 0 == Core::instance )
	{
		new Core();

		if ( false == Core::instance->configLogging( configPath ) )
		{
			return false;
		}

		Core::instance->m_logger->logInfo( "********************************************************" );
		Core::instance->m_logger->logInfo( "***************** Initializing Core... *****************" );
		Core::instance->m_logger->logInfo( "********************************************************" );

		Core::instance->m_gameObjectFactory = gameObjectFactory;
		Core::instance->m_subSystemFactory = new ZazenSubSystemFactory();

		Core::instance->m_eventManager = new EventManager();

		if ( false == ScriptSystem::initialize() )
		{
			Core::instance->m_logger->logError( "Failed initializing ScriptSystem - exit" );
			return false;
		}

		if ( false == Core::instance->loadConfig( configPath ) )
		{
			Core::instance->m_logger->logError( "Failed initializing Core - exit" );
			return false;
		}

		Core::instance->m_logger->logInfo( "********************************************************" );
		Core::instance->m_logger->logInfo( "************** Core successfully initialized ***********" );
		Core::instance->m_logger->logInfo( "********************************************************" );
	}

	return true;
}

bool
Core::configLogging( const std::string& configPath )
{
	try
	{
		log4cplus::PropertyConfigurator::doConfigure( log4cplus::helpers::towstring( configPath + "/log4cplus.properties" ) );
	}
	catch( exception e )
	{
		return false;
	}

	this->m_logger = new Logger( "zaZenCore" );

	return true;
}

bool
Core::shutdown()
{
	if ( Core::instance )
	{
		ScriptSystem::getInstance().callFunc( "onShutdown" );

		list<ISubSystem*>::iterator iter;

		iter = Core::instance->m_subSystems.begin();
		while ( iter != Core::instance->m_subSystems.end() )
		{
			ISubSystem* subSys = *iter++;
			subSys->shutdown();
			delete subSys;
		}

		if ( Core::instance->m_eventManager )
		{
			delete Core::instance->m_eventManager;
		}

		if ( Core::instance->m_subSystemFactory )
		{
			delete Core::instance->m_subSystemFactory;
		}

		// TODO delete loggers

		ScriptSystem::shutdown();

		delete Core::instance;
	}

	return true;
}

Core::Core()
{
	Core::instance = this;

	this->m_processingFactor = 0;
	this->m_runCore = false;

	this->m_eventManager = 0;

	this->m_gameObjectFactory = 0;
	this->m_subSystemFactory = 0;

	this->m_audio = 0;
	this->m_graphics = 0;
	this->m_input = 0;
	this->m_physics = 0;

	this->m_logger = NULL;
}

Core::~Core()
{
	Core::instance = 0;
}

void
Core::start()
{
	list<ISubSystem*>::iterator subSysIter = this->m_subSystems.begin();
	while ( subSysIter != this->m_subSystems.end() )
	{
		ISubSystem* subSys = *subSysIter++;
		if ( false == subSys->start() )
		{
			this->m_logger->logError( "Failed starting SubSystem \"" + subSys->getID() + " - exit" );
			return;
		}
	}

	long long startTicks = 0;
	long long endTicks = 0;
	
	this->m_logger->logTrace( "Start processing" );

	this->m_runCore = true;
	while ( this->m_runCore )
	{
		startTicks = this->getCurrentMillis();
		
		this->m_eventManager->processQueue();

		if ( false == ScriptSystem::getInstance().callFunc( "beginFrame" ) )
		{
			this->m_runCore = false;
			break;
		}

		subSysIter = this->m_subSystems.begin();
		while ( subSysIter != this->m_subSystems.end() )
		{
			if ( false == (*subSysIter)->process( this->m_processingFactor ) )
			{
				this->m_runCore = false;
				break;
			}

			subSysIter++;
		}

		if ( false == ScriptSystem::getInstance().callFunc( "endFrame" ) )
		{
			this->m_runCore = false;
			break;
		}

		subSysIter = this->m_subSystems.begin();
		while ( subSysIter != this->m_subSystems.end() )
		{
			(*subSysIter)->finalizeProcess();
			subSysIter++;
		}
		
		endTicks = this->getCurrentMillis();
		
		this->m_processingFactor = ( float ) ( endTicks - startTicks ) / ( float ) 1000;
	};

	subSysIter = this->m_subSystems.begin();
	while ( subSysIter != this->m_subSystems.end() )
	{
		( *subSysIter )->stop();
		subSysIter++;
	}
}

void
Core::stop()
{
	this->m_runCore = false;
}

ISubSystem*
Core::getSubSystemByID( const std::string& id )
{
	list<ISubSystem*>::iterator iter = this->m_subSystems.begin();
	while ( iter != this->m_subSystems.end() )
	{
		ISubSystem* subSys = *iter++;
		if ( subSys->getID() == id )
		{
			return subSys;
		}
	}

	return NULL;
}

ILogger*
Core::getLogger( const std::string& name )
{
	ILogger* logger = NULL;

	map<string, ILogger*>::iterator findIter = this->m_loggers.find( name );
	if ( findIter != this->m_loggers.end() )
	{
		return findIter->second;
	}

	logger = new Logger( name );
	this->m_loggers[ name ] = logger;

	return logger;
}

long long
Core::getCurrentMillis() const
{
	static LARGE_INTEGER s_frequency;
    static BOOL s_use_qpc = QueryPerformanceFrequency( &s_frequency );

    if ( s_use_qpc ) 
	{
        LARGE_INTEGER now;
        QueryPerformanceCounter( &now );
        return ( 1000LL * now.QuadPart ) / s_frequency.QuadPart;
    } 
	else 
	{
        return GetTickCount();
    }
}

ISubSystem*
Core::getSubSystemByType( const std::string& type )
{
	list<ISubSystem*>::iterator iter = this->m_subSystems.begin();
	while ( iter != this->m_subSystems.end() )
	{
		ISubSystem* subSys = *iter++;
		if ( subSys->getType() == type )
		{
			return subSys;
		}
	}

	return 0;
}

bool
Core::loadConfig( const std::string& configPath )
{
	std::string configFileName = configPath + "config.xml";

	TiXmlDocument doc( configFileName.c_str() );

	if ( false == doc.LoadFile() )
	{
		this->m_logger->logError( "ERROR ... could not load file " + configFileName + " - reason = " + doc.ErrorDesc() );
		return false;
	}

	TiXmlElement* rootNode = doc.FirstChildElement( "config" );
	if ( 0 == rootNode )
	{
		this->m_logger->logError( "Root-node \"config\" in " + configFileName + " not found" );
		return false;
	}

	TiXmlElement* coreScriptNode = rootNode->FirstChildElement( "coreScript" );
	if ( 0 == coreScriptNode )
	{
		this->m_logger->logError( "Node \"coreScript\" in " + configFileName + " not found" );
		return false;
	}

	string coreScriptFileName = coreScriptNode->Attribute( "file" );

	if ( false == ScriptSystem::getInstance().loadFile( coreScriptFileName ) )
	{
		return false;
	}

	if ( false == ScriptSystem::getInstance().callFunc( "onStartup" ) )
	{
		return false;
	}

	TiXmlElement* subSystemListNode = rootNode->FirstChildElement( "subSystemList" );
	if ( 0 == subSystemListNode )
	{
		this->m_logger->logError( "Node \"subSystemList\" in " + configFileName + " not found" );
		return false;
	}

	for (TiXmlElement* subSystemNode = subSystemListNode->FirstChildElement(); subSystemNode != 0; subSystemNode = subSystemNode->NextSiblingElement())
	{
		const char* str = subSystemNode->Value();
		if (str == 0)
			continue;

		if ( 0 == strcmp(str, "subSystem" ) )
		{
			str = subSystemNode->Attribute( "file" );
			if ( 0 == str )
			{
				this->m_logger->logWarning( "No file defined for subSystem - will be ignored" );
				continue;
			}
			else
			{
				ISubSystem* subSystem = this->loadSubSystem( str, configPath );
				if ( subSystem )
				{
					this->m_subSystems.push_back( subSystem );
				}
				else
				{
					return false;
				}
			}
		}
	}

	TiXmlElement* objectListNode = rootNode->FirstChildElement( "objectList" );
	if ( 0 == objectListNode )
	{
		this->m_logger->logError( "ERROR ... node \"objectList\" in " + configFileName + " not found" );
		return false;
	}

	for (TiXmlElement* objectNode = objectListNode->FirstChildElement(); objectNode != 0; objectNode = objectNode->NextSiblingElement())
	{
		const char* str = objectNode->Value();
		if ( 0 == str )
		{
			continue;
		}

		if ( 0 == strcmp( str, "object" ) )
		{
			std::string objectClass;
			IGameObject* object = NULL;

			const char* str = objectNode->Attribute( "class" );
			if ( 0 != str )
			{
				objectClass = str;
			}

			if ( false == objectClass.empty() )
			{
				object = this->m_gameObjectFactory->createObject( objectClass );
			}

			if ( NULL == object )
			{
				object = new ZazenGameObject( "ZaZenGameObject" );
			}

			if ( object->initialize( objectNode ) )
			{
				this->m_gameObjects.push_back( object );
			}
			else
			{
				delete object;
			}
		}
	}

	std::string target;

	TiXmlElement* controlNode = rootNode->FirstChildElement( "control" );
	if ( 0 == controlNode )
	{
		this->m_logger->logInfo( "No controlNode defined - using defaults" );
	}
	else
	{
		const char* str = controlNode->Attribute( "target" );
		if ( 0 == str )
		{
			this->m_logger->logInfo( "Target attribute missing in controlNode - use default " );
		}
		else
		{
			target = str;
		}
	}

	if ( false == target.empty() )
	{
		IGameObject* inputTarget = this->getObjectByName( target );
		if ( inputTarget )
		{
			this->m_eventManager->registerForEvent( "KEY_PRESSED", inputTarget );
			this->m_eventManager->registerForEvent( "KEY_RELEASED", inputTarget );
			this->m_eventManager->registerForEvent( "MOUSE_MOVED", inputTarget );
		}
	}

	return true;
}

IGameObject*
Core::getObjectByName( const std::string& name )
{
	std::list<IGameObject*>::iterator iter = this->m_gameObjects.begin();
	while ( iter != this->m_gameObjects.end() )
	{
		IGameObject* obj = *iter++;

		if ( obj->getName() == name )
		{
			return obj;
		}
	}

	return 0;
}

ISubSystem*
Core::loadSubSystem( const std::string& fileName, const std::string& configPath )
{
	string fullFileName = configPath + fileName;

	TiXmlDocument doc( fullFileName.c_str() );

	if ( false == doc.LoadFile() )
	{
		this->m_logger->logError( "ERROR ... could not load file " + fullFileName + " - reason = " + doc.ErrorDesc() );
		return 0;
	}

	TiXmlElement* subSystemNode = doc.FirstChildElement( "subSystem" );
	if ( 0 == subSystemNode )
	{
		this->m_logger->logError( "ERROR ... no root-node \"subSystem\" defined in " + fullFileName + " - exit " );
		return 0;
	}

	string subSystemType;
	const char* str = subSystemNode->Attribute( "type" );
	if ( 0 == str )
	{
		this->m_logger->logWarning( "No type defined for subSystem - ignoring" );
		return NULL;
	}
	else
	{
		subSystemType = str;
	}

	string subSystemFile;
	str = subSystemNode->Attribute( "file" );
	if ( 0 == str )
	{
		this->m_logger->logWarning( "No file defined for subSystem - ignoring" );
		return NULL;
	}
	else
	{
		subSystemFile = configPath + str;
	}

	ISubSystem* subSystem = this->m_subSystemFactory->createSubSystem( subSystemFile, subSystemType );
	if ( 0 == subSystem )
	{
		return 0;
	}

	if ( false == this->checkSubSystemType( subSystem ) )
	{
		subSystem->m_libStruct->destructorFunc( subSystem );
		return 0;
	}

	if ( false == subSystem->initialize( subSystemNode ) )
	{
		this->m_logger->logError( "Initializing " + subSystemType + " Subsystem failed - exit" );
		delete subSystem;
		return NULL;
	}


	return subSystem;
}

bool
Core::checkSubSystemType( ISubSystem* subSystem )
{
	if ( dynamic_cast<IAudio*>( subSystem ) )
	{
		if ( NULL != this->m_audio )
		{
			this->m_logger->logWarning( "Trying to load Audio-Subsystem but is already present, it is not allowed to have two SubSystems of same type" );
		}

		this->m_audio = ( IAudio* ) subSystem;
	}

	if ( dynamic_cast<IGraphics*>( subSystem ) )
	{
		if ( NULL != this->m_graphics )
		{
			this->m_logger->logWarning( "Trying to load Graphics-Subsystem but is already present, it is not allowed to have two SubSystems of same type" );
		}

		this->m_graphics = ( IGraphics* ) subSystem;
	}

	if ( dynamic_cast<IInput*>( subSystem ) )
	{
		if ( NULL != this->m_input )
		{
			this->m_logger->logWarning( "Trying to load Input-Subsystem but is already present, it is not allowed to have two SubSystems of same type" );
		}

		this->m_input = ( IInput* ) subSystem;
	}

	if ( dynamic_cast<IPhysics*>( subSystem ) )
	{
		if ( NULL != this->m_physics )
		{
			this->m_logger->logWarning( "Trying to load Physics-Subsystem but is already present, it is not allowed to have two SubSystems of same type" );
		}

		this->m_physics = ( IPhysics* ) subSystem;
	}

	return true;
}
