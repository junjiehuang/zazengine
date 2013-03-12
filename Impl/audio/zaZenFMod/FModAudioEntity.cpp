/*
 * FModAudioEntity.cpp
 *
 *  Created on: 09.07.2010
 *      Author: Jonathan Thaler
 */

#include "FModAudioEntity.h"

#include "FModAudio.h"

#include <fmodex/fmod_errors.h>

#include <iostream>

using namespace std;
using namespace FMOD;

FModAudioEntity::FModAudioEntity( Sound* sound, IGameObject* p )
	: IAudioEntity( p ),
	type( "audio" )
{
	this->m_sound = sound;
	this->m_channel = 0;
}

FModAudioEntity::~FModAudioEntity()
{
	if ( this->m_sound )
	{
		FMOD_RESULT result = this->m_sound->release();
		if ( FMOD_OK != result )
		{
			printf( "FMOD error! (%d) %s\n", result, FMOD_ErrorString( result ) );
		}

		// IMPORTANT: DO NOT DELETE AFTER RELEASE, IT IS RELEASED BY FMOD SYSTEM!
		//delete this->m_sound;
	}
}

bool
FModAudioEntity::sendEvent( Event& e )
{
	if ( e == "updatePhysics" )
	{
		this->queuedEvents.push_back( e );
		return true;
	}

	return false;
}

bool
FModAudioEntity::playSound()
{
	FMOD_RESULT result = FModAudio::getInstance().getSystem()->playSound( FMOD_CHANNEL_FREE, this->m_sound, false, &this->m_channel );
	if ( FMOD_OK != result )
	{
		printf( "FMOD error! (%d) %s\n", result, FMOD_ErrorString( result ) );
		return false;
	}

	return true;
}

void
FModAudioEntity::setPosVel( const float* pos, const float* vel )
{
	FMOD_VECTOR position;
	FMOD_VECTOR velocity;

	position.x = pos[ 0 ];
	position.y = pos[ 1 ];
	position.z = pos[ 2 ];

	velocity.x = vel[ 0 ];
	velocity.y = vel[ 1 ];
	velocity.z = vel[ 2 ];

	//cout << "Audio:" << this->getParent()->getName() << " has position of (" << pos[0] << "/" << pos[1] << "/" << pos[2] << ")" << endl;

	this->m_channel->set3DAttributes( &position, &velocity );
}
