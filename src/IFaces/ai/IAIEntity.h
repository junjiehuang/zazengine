/*
 * IAIEntity.h
 *
 *  Created on: 15.07.2010
 *      Author: Jonathan Thaler
 */

#ifndef IAIENTITY_H_
#define IAIENTITY_H_

#include "ISubSystemEntity.h"

class IAIEntity : public ISubSystemEntity
{
	public:
		IAIEntity( IGameObject* p ) : ISubSystemEntity( p ) {};
		virtual ~IAIEntity() {};

};

#endif /* IAIENTITY_H_ */
