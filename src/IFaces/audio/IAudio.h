#ifndef _IAUDIO_H_
#define _IAUDIO_H_

#include "../core/ISubSystem.h"
#include "IAudioEntity.h"

class IAudio : public ISubSystem
{
	public:
		virtual ~IAudio() {};

		virtual IAudioEntity* createEntity( TiXmlElement*, IGameObject* parent ) = 0;

};

#endif /* _IAUDIO_H_ */
