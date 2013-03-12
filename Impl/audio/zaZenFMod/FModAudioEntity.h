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
		FModAudioEntity( FMOD::Sound*, IGameObject* );
		virtual ~FModAudioEntity();

		const std::string& getType() const { return this->type; };

		bool sendEvent( Event& e );

		bool playSound();
		void setPosVel( const float* pos, const float* vel );

	private:
		std::string type;

		FMOD::Sound* m_sound;
		FMOD::Channel* m_channel;
};

#endif /* FMODAUDIOENTITY_H_ */
