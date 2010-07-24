/*
 * FModAudio.h
 *
 *  Created on: 06.07.2010
 *      Author: joni
 */

#ifndef FMODAUDIO_H_
#define FMODAUDIO_H_

#include "../../../Core/SubSystems/IFaces/IAudio.h"

#include "FModAudioEntity.h"

#include <AL/alut.h>
#include <fmod/fmod.hpp>
#include <fmod/fmod_errors.h>

class FModAudio : public IAudio
{
	public:
		FModAudio();
		virtual ~FModAudio();

		const std::string& getID() const { return this->id; };
		const std::string& getType() const { return this->type; };

		bool isAsync() const { return false; };

		bool initialize( TiXmlElement* );
		bool shutdown();

		bool start();
		bool stop();
		bool pause();

		bool process( double );
		bool finalizeProcess();

		bool sendEvent( const Event& e );

		FModAudioEntity* createEntity( TiXmlElement* );

	private:
		std::string id;
		std::string type;

		FMOD::System* system;
		FMOD::Sound* bgMusic;
		FMOD::Channel* bgMusicCh;

};

#endif /* PLAYGROUNDAUDIO_H_ */
