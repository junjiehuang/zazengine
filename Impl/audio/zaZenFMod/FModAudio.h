/*
 * FModAudio.h
 *
 *  Created on: 06.07.2010
 *      Author: joni
 */

#ifndef FMODAUDIO_H_
#define FMODAUDIO_H_

#include <audio/IAudio.h>

#include "FModAudioEntity.h"

#include <fmodex/fmod.hpp>
#include <fmodex/fmod_errors.h>

#include <list>

class FModAudio : public IAudio
{
	public:
		FModAudio( const std::string&, ICore* );
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

		bool sendEvent( Event& e );

		FModAudioEntity* createEntity( TiXmlElement*, IGameObject* parent );

	private:
		std::string id;
		std::string type;

		ICore* m_core;

		FMOD::System* m_system;
		FMOD::Sound* m_bgMusic;
		FMOD::Channel* m_bgMusicCh;

		std::list<FModAudioEntity*> entities;

};

#endif /* PLAYGROUNDAUDIO_H_ */
