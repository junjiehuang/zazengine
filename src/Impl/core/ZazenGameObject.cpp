/*
 * ZazenGameObject.cpp
 *
 *  Created on: 27.06.2010
 *      Author: Jonathan Thaler
 */

#include "ZazenGameObject.h"

#include "Core.h"
#include "ScriptSystem.h"

#include <iostream>

using namespace std;

ZazenGameObject::GameObjectID ZazenGameObject::nextID = 0;

ZazenGameObject::ZazenGameObject( const std::string& objectClass )
	: m_id ( ZazenGameObject::nextID++ ),
	m_objectClass( objectClass )
{
}

ZazenGameObject::~ZazenGameObject()
{
}

bool
ZazenGameObject::sendEvent( Event& e )
{
	map<string, ISubSystemEntity*>::iterator iter = this->m_subSystemEntities.begin();
	while ( iter != this->m_subSystemEntities.end() )
	{
		ISubSystemEntity* entity = iter->second;
		iter++;

		if ( e == "collidesWith" )
		{
		} else 
		{
			entity->sendEvent( e );
		}
	}

	// TODO: call into script-function when provided for this event

	return true;
}

bool
ZazenGameObject::initialize( TiXmlElement* objectNode )
{
	string objectName;
	string scriptFile;

	const char* str = objectNode->Attribute( "name" );
	if ( 0 == str )
	{
		Core::getRef().getCoreLogger().logWarning( "No id defined for object - will be ignored" );
		return false;
	}
	else
	{
		this->m_name = str;
	}

	str = objectNode->Attribute( "script" );
	if ( 0 == str )
	{
		Core::getRef().getCoreLogger().logInfo( "No script defined for object " + objectName );
	}
	else
	{
		scriptFile = str;
	}

	if ( false == scriptFile.empty() )
	{
		if ( false == ScriptSystem::getInstance().loadFile( scriptFile ) )
		{
			return false;
		}
	}

	TiXmlElement* propertiesNode = objectNode->FirstChildElement( "properties" );
	if ( 0 != propertiesNode )
	{
		for (TiXmlElement* propNode = propertiesNode->FirstChildElement(); propertiesNode != 0; propertiesNode = propertiesNode->NextSiblingElement())
		{
			const char* str = propNode->Value();
			if ( 0 == str )
			{
				continue;
			}
		}
	}

	for (TiXmlElement* subSystemEntityNode = objectNode->FirstChildElement(); subSystemEntityNode != 0; subSystemEntityNode = subSystemEntityNode->NextSiblingElement())
	{
		const char* str = subSystemEntityNode->Value();
		if ( 0 == str )
		{
			continue;
		}

		string subSystemType = str;
		ISubSystem* subSystem = Core::getRef().getSubSystemByType( str );
		if ( 0 == subSystem )
		{
			Core::getRef().getCoreLogger().logError( "No according SubSystem for definition \"" + subSystemType + "\" found - object will be ignored" );
			return false;
		}
		else
		{
			ISubSystemEntity* subSystemEntity = subSystem->createEntity( subSystemEntityNode, this );
			if ( 0 == subSystemEntity )
			{
				Core::getRef().getCoreLogger().logError( "Failed creating instance for subsystem-type \"" + subSystemType + "\"" );
				return false;
			}

			this->m_subSystemEntities.insert( make_pair( subSystemEntity->getType(), subSystemEntity ) );
		}
	}

	/*
	map<string, ISubSystemEntity*>::iterator iter = this->subSystemEntities.begin();
	while ( iter != this->subSystemEntities.end() )
	{
		ISubSystemEntity* entity = iter->second;
		iter++;

		vector<string> dependencies = entity->getDependencies();
		for ( unsigned int i = 0; i < dependencies.size(); i++ )
		{
			map<string, ISubSystemEntity*>::iterator findIter = this->subSystemEntities.find( dependencies[i] );
			if ( findIter != this->subSystemEntities.end() )
			{
				findIter->second->addConsumer( entity );
			}
			else
			{
				cout << "ERROR ... failed resolving dependency \"" << dependencies[i] << "\"" << endl;
				return false;
			}
		}
	}
	*/

	return true;
}
