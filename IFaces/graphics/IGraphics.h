/*
 * IGraphics
 *
 *  Created on: 05.07.2010
 *      Author: Jonathan Thaler
 */

#ifndef IGRAPHICS_H_
#define IGRAPHICS_H_

#include <core/ISubSystem.h>
#include <graphics/IGraphicsEntity.h>

class IGraphics : public ISubSystem
{
	public:
		virtual ~IGraphics() {};

		virtual void* getWindowHandle() = 0;

		virtual IGraphicsEntity* createEntity( TiXmlElement*, IGameObject* parent ) = 0;

};

#endif /* IGRAPHICS_H_ */
