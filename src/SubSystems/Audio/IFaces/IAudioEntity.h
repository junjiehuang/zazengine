/*
 * IAudioEntity.h
 *
 *  Created on: 15.07.2010
 *      Author: joni
 */

#ifndef IAUDIOENTITY_H_
#define IAUDIOENTITY_H_

#include "ISubSystemEntity.h"

class IAudioEntity : public ISubSystemEntity
{
	public:
		IAudioEntity( IGameObject* p ) : ISubSystemEntity( p ) {};
		virtual ~IAudioEntity() {};

		virtual void setPosVel( const float* pos, const float* vel ) = 0;
};

#endif /* IAUDIOENTITY_H_ */
