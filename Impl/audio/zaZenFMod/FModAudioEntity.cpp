/*
 * FModAudioEntity.cpp
 *
 *  Created on: 09.07.2010
 *      Author: joni
 */

#include "FModAudioEntity.h"

#include <iostream>

using namespace std;

FModAudioEntity::FModAudioEntity( IGameObject* p )
	: IAudioEntity( p ),
	type( "audio" )
{
	this->sound = 0;
	this->channel = 0;
}

FModAudioEntity::~FModAudioEntity()
{

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

void
FModAudioEntity::setPosVel( const float* pos, const float* vel )
{
	FMOD_VECTOR position;
	FMOD_VECTOR velocity;

	position.x = pos[0];
	position.y = pos[1];
	position.z = pos[2];

	velocity.x = vel[0];
	velocity.y = vel[1];
	velocity.z = vel[2];

	//cout << "Audio:" << this->getParent()->getName() << " has position of (" << pos[0] << "/" << pos[1] << "/" << pos[2] << ")" << endl;

	this->channel->set3DAttributes( &position, &velocity );
}
