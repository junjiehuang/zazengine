/*
 * IAudioEntity.h
 *
 *  Created on: 15.07.2010
 *      Author: Jonathan Thaler
 */

#ifndef IAUDIOENTITY_H_
#define IAUDIOENTITY_H_

#include "core/ISubSystemEntity.h"

class IAudioEntity : public ISubSystemEntity
{
	public:
		IAudioEntity( IGameObject* p ) : ISubSystemEntity( p ) {};
		virtual ~IAudioEntity() {};

		virtual bool playSound() = 0;
		virtual void updatePosVel( const float* pos, const float* vel ) = 0;
};

#endif /* IAUDIOENTITY_H_ */
