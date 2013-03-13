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
using namespace boost;
using namespace FMOD;

FModAudioEntity::FModAudioEntity( IGameObject* p )
	: IAudioEntity( p ),
	type( "audio" )
{
	this->m_sound = 0;
	this->m_channel = 0;

	this->m_pos.x = 0.0f;
	this->m_pos.y = 0.0f;
	this->m_pos.z = 0.0f;

	this->m_vel.x = 0.0f;
	this->m_vel.y = 0.0f;
	this->m_vel.z = 0.0f;
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
		boost::any& pos = e.getValue( "pos" );
		boost::any& vel = e.getValue( "vel" );

		this->updatePosVel( any_cast<const float*>( pos ), any_cast<const float*>( vel )  );
	}
	else if ( e == "POSITION_CHANGED" )
	{
		boost::any& matrixAny = e.getValue( "matrix" );
		const float* matrixValues = any_cast<const float*>( matrixAny );

		FMOD_VECTOR forward;
		FMOD_VECTOR up;
		FMOD_VECTOR vel = { 0.0f, 0.0f, 0.0f };
		FMOD_VECTOR pos;

		up.x = matrixValues[ 4 ];
		up.y = matrixValues[ 5 ];
		up.z = matrixValues[ 6 ];

		forward.x = matrixValues[ 8 ];
		forward.y = matrixValues[ 9 ];
		forward.z = matrixValues[ 10 ];

		pos.x = matrixValues[ 12 ];
		pos.y = matrixValues[ 13 ];
		pos.z = matrixValues[ 14 ];

		FModAudio::getInstance().getSystem()->set3DListenerAttributes( 0, &pos, &vel, &forward, &up );
	}

	return false;
}

void
FModAudioEntity::setSound( FMOD::Sound* sound )
{
	this->m_sound = sound;
}

void
FModAudioEntity::setPosition( float x, float y, float z )
{
	this->m_pos.x = x;
	this->m_pos.y = y;
	this->m_pos.z = z;
}

bool
FModAudioEntity::playSound()
{
	if ( this->m_sound )
	{
		FMOD_RESULT result = FModAudio::getInstance().getSystem()->playSound( FMOD_CHANNEL_FREE, this->m_sound, true, &this->m_channel );
		if ( FMOD_OK != result )
		{
			printf( "FMOD error! (%d) %s\n", result, FMOD_ErrorString( result ) );
			return false;
		}

		result = this->m_channel->set3DAttributes( &this->m_pos, &this->m_vel );
		if ( FMOD_OK != result )
		{
			printf( "FMOD error! (%d) %s\n", result, FMOD_ErrorString( result ) );
			return false;
		}

		result = this->m_channel->setPaused( false );
		if ( FMOD_OK != result )
		{
			printf( "FMOD error! (%d) %s\n", result, FMOD_ErrorString( result ) );
			return false;
		}
	}

	return true;
}

void
FModAudioEntity::updatePosVel( const float* pos, const float* vel )
{
	if ( this->m_channel )
	{
		this->m_pos.x = pos[ 0 ];
		this->m_pos.y = pos[ 1 ];
		this->m_pos.z = pos[ 2 ];

		this->m_vel.x = vel[ 0 ];
		this->m_vel.y = vel[ 1 ];
		this->m_vel.z = vel[ 2 ];

		//cout << "Audio:" << this->getParent()->getName() << " has position of (" << pos[0] << "/" << pos[1] << "/" << pos[2] << ")" << endl;

		this->m_channel->set3DAttributes( &this->m_pos, &this->m_vel );
	}
}
