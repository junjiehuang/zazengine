/*
 * Core.cpp
 *
 *  Created on: 27.06.2010
 *      Author: joni
 */

#include "Core.h"

#include "ScriptSystem.h"

#include <windows.h>

#include <iostream>
#include <sstream>

using namespace std;

Core* Core::instance = 0;

bool
Core::initalize( const std::string& configPath )
{
	// TODO: use filesystem-utilities of boost to handle path & files

	if ( 0 == Core::instance )
	{
		new Core();

		cout << "********************************************************" << endl;
		cout << "***************** Initializing Core... *****************" << endl;
		cout << "********************************************************" << endl;

		Core::instance->m_gameObjectFactory = new ZazenGameObjectFactory();
		Core::instance->m_subSystemFactory = new ZazenSubSystemFactory();

		Core::instance->m_eventManager = new EventManager();

		if ( false == ScriptSystem::initialize() )
		{
			cout << "ERROR ... failed initializing ScriptSystem - exit" << endl;
			return false;
		}

		if ( false == Core::instance->loadConfig( configPath ) )
		{
			cout << "ERROR ... failed initializing Core - exit" << endl;
			return false;
		}

		cout << "********************************************************" << endl;
		cout << "************** Core successfully initialized ***********" << endl;
		cout << "********************************************************" << endl;
	}

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
			delete Core::instance->m_eventManager;

		if ( Core::instance->m_gameObjectFactory )
			delete Core::instance->m_gameObjectFactory;

		if ( Core::instance->m_subSystemFactory )
			delete Core::instance->m_subSystemFactory;

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
			cout << "Failed starting SubSystem \"" << subSys->getID() << " - exit" << endl;
			return;
		}
	}

	FILETIME t;
	long long startTicks = 0;
	long long endTicks = 0;
	
	cout << "Start processing" << endl;

	this->m_runCore = true;
	while ( this->m_runCore )
	{
		GetSystemTimeAsFileTime( &t );
		startTicks = 0; // TODO
		
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

		//sleep( 1 );

		subSysIter = this->m_subSystems.begin();
		while ( subSysIter != this->m_subSystems.end() )
		{
			(*subSysIter)->finalizeProcess();
			subSysIter++;
		}
		
		GetSystemTimeAsFileTime( &t );
		endTicks = 0; // TODO
		
		this->m_processingFactor = (double)(endTicks - startTicks) / (double) 1000;
	};

	subSysIter = this->m_subSystems.begin();
	while ( subSysIter != this->m_subSystems.end() )
	{
		(*subSysIter)->stop();
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
			return subSys;
	}

	return 0;
}

ISubSystem*
Core::getSubSystemByType( const std::string& type )
{
	list<ISubSystem*>::iterator iter = this->m_subSystems.begin();
	while ( iter != this->m_subSystems.end() )
	{
		ISubSystem* subSys = *iter++;
		if ( subSys->getType() == type )
			return subSys;
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
		cout << "ERROR ... could not load file " << configFileName << " - reason = " << doc.ErrorDesc() << " at row = " << doc.ErrorRow() << " col = " << doc.ErrorCol() << endl;
		return false;
	}

	TiXmlElement* rootNode = doc.FirstChildElement( "config" );
	if ( 0 == rootNode )
	{
		cout << "ERROR ... root-node \"config\" in " << configFileName << " not found" << endl;
		return false;
	}

	TiXmlElement* coreScriptNode = rootNode->FirstChildElement( "coreScript" );
	if ( 0 == coreScriptNode )
	{
		cout << "ERROR ... node \"coreScript\" in " << configFileName << " not found" << endl;
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
		cout << "ERROR ... node \"subSystemList\" in " << configFileName << " not found" << endl;
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
				cout << "No file defined for subSystem - will be ignored" << endl;
				continue;
			}
			else
			{
				ISubSystem* subSystem = this->loadSubSystem( str, configPath );
				if ( subSystem )
					this->m_subSystems.push_back( subSystem );
				else
					return false;
			}
		}
	}

	TiXmlElement* objectListNode = rootNode->FirstChildElement("objectList");
	if ( 0 == objectListNode )
	{
		cout << "ERROR ... node \"objectList\" in " << configFileName << " not found" << endl;
		return false;
	}

	for (TiXmlElement* objectNode = objectListNode->FirstChildElement(); objectNode != 0; objectNode = objectNode->NextSiblingElement())
	{
		const char* str = objectNode->Value();
		if ( 0 == str )
			continue;

		if ( 0 == strcmp( str, "object" ) )
		{
			IGameObject* object = this->m_gameObjectFactory->createObject( "" );
			if ( object->initialize( objectNode ) )
				this->m_gameObjects.push_back( object );
			else
				delete object;
		}
	}

	std::string target;

	TiXmlElement* controlNode = rootNode->FirstChildElement( "control" );
	if ( 0 == controlNode )
	{
		cout << "INFO ... no controlNode defined - using defaults" << endl;
	}
	else
	{
		const char* str = controlNode->Attribute( "target" );
		if ( 0 == str )
		{
			cout << "INFO ... target attribute missing in controlNode - use default " << endl;
		}
		else
		{
			target = str;
		}
	}

	if ( false == target.empty() )
	{
		IGameObject* inputTarget = getObjectByName( target );
		if ( inputTarget )
		{
			Core::getInstance().getEventManager().registerForEvent( "SDLK_RIGHT", inputTarget );
			Core::getInstance().getEventManager().registerForEvent( "SDLK_LEFT", inputTarget );
			Core::getInstance().getEventManager().registerForEvent( "SDLK_UP", inputTarget );
			Core::getInstance().getEventManager().registerForEvent( "SDLK_DOWN", inputTarget );
			Core::getInstance().getEventManager().registerForEvent( "SDLK_w", inputTarget );
			Core::getInstance().getEventManager().registerForEvent( "SDLK_s", inputTarget );
			Core::getInstance().getEventManager().registerForEvent( "SDLK_d", inputTarget );
			Core::getInstance().getEventManager().registerForEvent( "SDLK_a", inputTarget );
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
		cout << "ERROR ... could not load file " << fullFileName << " - reason = " << doc.ErrorDesc() << endl;
		return 0;
	}

	TiXmlElement* subSystemNode = doc.FirstChildElement( "subSystem" );
	if ( 0 == subSystemNode )
	{
		cout << "ERROR ... no root-node \"subSystem\" defined in " << fullFileName << " - exit " << endl;
		return 0;
	}

	string subSystemType;
	const char* str = subSystemNode->Attribute( "type" );
	if ( 0 == str )
	{
		cout << "No type defined for subSystem" << endl;
		return 0;
	}
	else
	{
		subSystemType = str;
	}

	string subSystemFile;
	str = subSystemNode->Attribute( "file" );
	if ( 0 == str )
	{
		cout << "No file defined for subSystem" << endl;
		return 0;
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

	if ( false == subSystem->initialize( subSystemNode ) )
	{
		cout << "Initializing " << subSystemType << " Subsystem failed - exit" << endl;
		delete subSystem;
		return 0;
	}


	return subSystem;
}
