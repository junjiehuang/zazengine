/*
 * FModAudio.cpp
 *
 *  Created on: 06.07.2010
 *      Author: joni
 */

#include "FModAudio.h"

#include <iostream>
#include <stdio.h>

using namespace std;

FModAudio::FModAudio()
	: id ("FModAudio"),
	type ("audio")
{
	this->system = 0;
	this->bgMusic = 0;
	this->bgMusicCh = 0;
}

FModAudio::~FModAudio()
{

}

bool
FModAudio::initialize( TiXmlElement* )
{
	cout << endl << "=============== FModAudio initializing... ===============" << endl;

	FMOD_RESULT result;
	unsigned int version;

	result = FMOD::System_Create( &this->system );
    if (result != FMOD_OK)
    {
        printf( "FMOD error! (%d) %s\n", result, FMOD_ErrorString( result ) );
       	return false;
    }

    result = this->system->getVersion( &version );
    if (result != FMOD_OK)
    {
        printf( "FMOD error! (%d) %s\n", result, FMOD_ErrorString( result ) );
       	return false;
    }

    if ( FMOD_VERSION > version )
    {
        printf( "Error!  You are using an old version of FMOD %08x.  This program requires %08x\n", version, FMOD_VERSION );
        return false;
    }

    result = this->system->init( 10, FMOD_INIT_NORMAL, 0 );
    if (result != FMOD_OK)
    {
        printf( "FMOD error! (%d) %s\n", result, FMOD_ErrorString( result ) );
       	return false;
    }

    result = system->createSound("media/audio/getout.ogg", FMOD_SOFTWARE | FMOD_2D, 0, &this->bgMusic );
    if (result != FMOD_OK)
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
		if (result != FMOD_OK)
		{
			printf( "FMOD error! (%d) %s\n", result, FMOD_ErrorString( result ) );
		}

		delete entity;
	}

	this->entities.clear();

	if ( this->bgMusic )
	{
		result = this->bgMusic->release();
		if (result != FMOD_OK)
		{
			printf( "FMOD error! (%d) %s\n", result, FMOD_ErrorString( result ) );
		}
	}

	if ( this->system )
	{
		result = this->system->close();
		if (result != FMOD_OK)
		{
			printf( "FMOD error! (%d) %s\n", result, FMOD_ErrorString( result ) );
		}

		result = this->system->release();
		if (result != FMOD_OK)
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

    result = this->system->playSound( FMOD_CHANNEL_FREE, this->bgMusic, false, &this->bgMusicCh );
    if (result != FMOD_OK)
	{
    	printf( "FMOD error! (%d) %s\n", result, FMOD_ErrorString( result ) );
    	return false;
    }

	std::list<FModAudioEntity*>::iterator iter = this->entities.begin();
	while ( iter != this->entities.end() )
	{
		FModAudioEntity* entity = *iter++;

		result = this->system->playSound( FMOD_CHANNEL_FREE, entity->sound, false, &entity->channel );
		if (result != FMOD_OK)
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
				const Value& pos = e.getValue( "pos" );
				const Value& vel = e.getValue( "vel" );

				entity->setPosVel( pos.data, vel.data );
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
		    result = this->system->createSound( str, FMOD_SOFTWARE | FMOD_3D, 0, &entity->sound );
		    if (result != FMOD_OK)
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
