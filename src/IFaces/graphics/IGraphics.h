#ifndef _IGRAPHICS_H_
#define _IGRAPHICS_H_

#include "core/ISubSystem.h"

#include "IGraphicsEntity.h"

class IGraphics : public ISubSystem
{
	public:
		virtual ~IGraphics() {};

		virtual void* getWindowHandle() = 0;
		virtual bool toggleFullscreen() = 0;

		virtual IGraphicsEntity* createEntity( TiXmlElement*, IGameObject* parent ) = 0;
};

#endif /* _IGRAPHICS_H_ */
