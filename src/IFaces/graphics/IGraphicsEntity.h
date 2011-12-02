/*
 * IGraphicsEntity.h
 *
 *  Created on: 15.07.2010
 *      Author: joni
 */

#ifndef IGRAPHICSENTITY_H_
#define IGRAPHICSENTITY_H_

#include "ISubSystemEntity.h"

class IGraphicsEntity : public ISubSystemEntity
{
	public:
		IGraphicsEntity( IGameObject* p ) : ISubSystemEntity( p ) {};
		virtual ~IGraphicsEntity() {};

		virtual void setOrientation( const float* pos, const float* rot) = 0;
};

#endif /* IGRAPHICSENTITY_H_ */
