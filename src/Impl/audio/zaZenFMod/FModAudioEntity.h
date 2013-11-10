#ifndef _FMODAUDIOENTITY_H_
#define _FMODAUDIOENTITY_H_

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

		void setImmediatePlayback( bool );
		bool isImmediatePlayback() { return this->m_immediatePlayback; };

		void setSound( FMOD::Sound* );
		void setPosition( float x, float y, float z );

	private:
		std::string type;

		bool m_immediatePlayback;

		FMOD_VECTOR m_pos;
		FMOD_VECTOR m_vel;

		FMOD::Sound* m_sound;
		FMOD::Channel* m_channel;
};

#endif /* _FMODAUDIOENTITY_H_ */
