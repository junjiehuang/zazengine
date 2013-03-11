/*
 * FModAudio.cpp
 *
 *  Created on: 06.07.2010
 *      Author: Jonathan Thaler
 */

#include "FModAudio.h"

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace boost;

FModAudio::FModAudio( const std::string& id, ICore* core )
	: id ( id ),
	type ( "audio" ),
	m_core( core )
{
	this->m_system = 0;
	this->m_bgMusic = 0;
	this->m_bgMusicCh = 0;
}

FModAudio::~FModAudio()
{

}

bool
FModAudio::initialize( TiXmlElement* configElem )
{
	cout << endl << "=============== FModAudio initializing... ===============" << endl;

	FMOD_RESULT result;
	unsigned int version;

	result = FMOD::System_Create( &this->m_system );
    if ( FMOD_OK != result )
    {
        printf( "FMOD error! (%d) %s\n", result, FMOD_ErrorString( result ) );
       	return false;
    }

    result = this->m_system->getVersion( &version );
    if ( FMOD_OK != result )
    {
        printf( "FMOD error! (%d) %s\n", result, FMOD_ErrorString( result ) );
       	return false;
    }

    if ( FMOD_VERSION > version )
    {
        printf( "Error!  You are using an old version of FMOD %08x.  This program requires %08x\n", version, FMOD_VERSION );
        return false;
    }

    result = this->m_system->init( 10, FMOD_INIT_NORMAL, 0 );
    if ( FMOD_OK != result )
    {
        printf( "FMOD error! (%d) %s\n", result, FMOD_ErrorString( result ) );
       	return false;
    }

    result = m_system->createSound( "../media/audio/getout.ogg", FMOD_SOFTWARE | FMOD_2D, 0, &this->m_bgMusic );
    if ( FMOD_OK != result )
    {
    	printf( "FMOD error! (%d) %s\n", result, FMOD_ErrorString( result ) );
    	return false;
    }

	cout << "================ FModAudio initialized =================" << endl;

	return true;
}

bool
FModAudio::shutdown()
{
	cout << endl << "=============== FModAudio shutting down... ===============" << endl;

	FMOD_RESULT result;

	std::list<FModAudioEntity*>::iterator iter = this->entities.begin();
	while ( iter != this->entities.end() )
	{
		FModAudioEntity* entity = *iter++;

		result = entity->sound->release();
		if ( FMOD_OK != result )
		{
			printf( "FMOD error! (%d) %s\n", result, FMOD_ErrorString( result ) );
		}

		delete entity;
	}

	this->entities.clear();

	if ( this->m_bgMusic )
	{
		result = this->m_bgMusic->release();
		if ( FMOD_OK != result )
		{
			printf( "FMOD error! (%d) %s\n", result, FMOD_ErrorString( result ) );
		}
	}

	if ( this->m_system )
	{
		result = this->m_system->close();
		if ( FMOD_OK != result )
		{
			printf( "FMOD error! (%d) %s\n", result, FMOD_ErrorString( result ) );
		}

		result = this->m_system->release();
		if ( FMOD_OK != result )
		{
			printf( "FMOD error! (%d) %s\n", result, FMOD_ErrorString( result ) );
		}
	}

	cout << "================ FModAudio shutdown =================" << endl;

	return true;
}

bool
FModAudio::start()
{
	FMOD_RESULT result;

    result = this->m_system->playSound( FMOD_CHANNEL_FREE, this->m_bgMusic, false, &this->m_bgMusicCh );
    if ( FMOD_OK != result )
	{
    	printf( "FMOD error! (%d) %s\n", result, FMOD_ErrorString( result ) );
    	return false;
    }

	std::list<FModAudioEntity*>::iterator iter = this->entities.begin();
	while ( iter != this->entities.end() )
	{
		FModAudioEntity* entity = *iter++;

		result = this->m_system->playSound( FMOD_CHANNEL_FREE, entity->sound, false, &entity->channel );
		if ( FMOD_OK != result )
		{
			printf( "FMOD error! (%d) %s\n", result, FMOD_ErrorString( result ) );
		    return false;
		}
	}

	return true;
}

bool
FModAudio::stop()
{
	return true;
}

bool
FModAudio::pause()
{
	return true;
}

bool
FModAudio::process(double factor)
{
	//cout << "FModAudio::process enter" << endl;

	// process events of entities
	std::list<FModAudioEntity*>::iterator iter = this->entities.begin();
	while ( iter != this->entities.end() )
	{
		FModAudioEntity* entity = *iter++;

		std::list<Event>::iterator eventsIter = entity->queuedEvents.begin();
		while ( eventsIter != entity->queuedEvents.end() )
		{
			Event& e = *eventsIter++;

			//cout << "received Event '" << e.getID() << "' in FModAudio from GO '" << entity->getParent()->getName() << endl;

			if ( e == "updatePhysics" )
			{
				boost::any& pos = e.getValue( "pos" );
				boost::any& vel = e.getValue( "vel" );

				entity->setPosVel( any_cast<const float*>( pos ), any_cast<const float*>( vel )  );
			}
		}

		entity->queuedEvents.clear();
	}

	//cout << "FModAudio::process leave" << endl;

	return true;
}

bool
FModAudio::finalizeProcess()
{
	//cout << "FModAudio::finalizeProcess" << endl;

	return true;
}

bool
FModAudio::sendEvent( Event& e )
{
	return true;
}

FModAudioEntity*
FModAudio::createEntity( TiXmlElement* objectNode, IGameObject* parent )
{
	FModAudioEntity* entity = new FModAudioEntity( parent );

	for (TiXmlElement* soundNode = objectNode->FirstChildElement(); soundNode != 0; soundNode = soundNode->NextSiblingElement())
	{
		const char* str = soundNode->Value();
		if ( 0 == str )
			continue;

		if ( 0 == strcmp( str, "sound" ) )
		{
			str = soundNode->Attribute( "file" );

			FMOD_RESULT result = FMOD_OK;
		    result = this->m_system->createSound( str, FMOD_SOFTWARE | FMOD_3D, 0, &entity->sound );
		    if ( FMOD_OK != result )
		    {
		    	cout << "ERROR ... loading sound from file \"" << str << ": " << FMOD_ErrorString( result ) << endl;
		    	delete entity;
		    	return 0;
		    }

		}
	}

	this->entities.push_back( entity );

	return entity;
}

extern "C"
{	
	__declspec( dllexport ) ISubSystem*
	createInstance ( const char* id, ICore* core )
	{
		return new FModAudio( id, core );
	}

	__declspec( dllexport ) void
	deleteInstance ( ISubSystem* subSys )
	{
		if ( 0 == subSys )
			return;

		if ( 0 == dynamic_cast<FModAudio*>( subSys ) )
			return;

		delete subSys;
	}
}
