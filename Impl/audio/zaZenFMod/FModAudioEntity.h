/*
 * FModAudioEntity.h
 *
 *  Created on: 08.07.2010
 *      Author: joni
 */

#ifndef FMODAUDIOENTITY_H_
#define FMODAUDIOENTITY_H_

#include <audio/IAudioEntity.h>

#include <fmodex/fmod.hpp>

class FModAudioEntity : public IAudioEntity
{
	public:
		friend class FModAudio;

		FModAudioEntity( IGameObject* p );
		virtual ~FModAudioEntity();

		const std::string& getType() const { return this->type; };

		virtual bool sendEvent( Event& e );

		void setPosVel( const float* pos, const float* vel );

	private:
		std::string type;

		FMOD::Sound* sound;
		FMOD::Channel* channel;
};

#endif /* FMODAUDIOENTITY_H_ */
