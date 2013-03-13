/*
 * FModAudioEntity.h
 *
 *  Created on: 08.07.2010
 *      Author: Jonathan Thaler
 */

#ifndef FMODAUDIOENTITY_H_
#define FMODAUDIOENTITY_H_

#include <audio/IAudioEntity.h>

#include <fmodex/fmod.hpp>

class FModAudioEntity : public IAudioEntity
{
	public:
		FModAudioEntity( IGameObject* );
		virtual ~FModAudioEntity();

		const std::string& getType() const { return this->type; };

		bool sendEvent( Event& e );

		bool playSound();
		void updatePosVel( const float* pos, const float* vel );

		void setSound( FMOD::Sound* );
		void setPosition( float x, float y, float z );

	private:
		std::string type;

		FMOD_VECTOR m_pos;
		FMOD_VECTOR m_vel;

		FMOD::Sound* m_sound;
		FMOD::Channel* m_channel;
};

#endif /* FMODAUDIOENTITY_H_ */
