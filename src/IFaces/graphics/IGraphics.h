/*
 * IGraphics
 *
 *  Created on: 05.07.2010
 *      Author: joni
 */

#ifndef IGRAPHICS_H_
#define IGRAPHICS_H_

#include "ISubSystem.h"
#include "IGraphicsEntity.h"

class IGraphics : public ISubSystem
{
	public:
		virtual ~IGraphics() {};

		virtual IGraphicsEntity* createEntity( TiXmlElement*, IGameObject* parent ) = 0;

};

#endif /* IGRAPHICS_H_ */
