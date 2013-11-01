#ifndef RENDERER_H_
#define RENDERER_H_

#include "../Scene/Viewer.h"
#include "../Scene/Instance.h"
#include "../Lighting/Light.h"
#include "../ZazenGraphicsEntity.h"

#include <list>

class Renderer
{
 public:
	Renderer();
	virtual ~Renderer();

	virtual bool initialize() = 0;
	virtual bool shutdown() = 0;
	virtual bool renderFrame( std::list<ZazenGraphicsEntity*>& entities ) = 0;

	void printInfo();

 protected:
	 long long frame;

	 Viewer* m_camera;
};

#endif /*RENDERER_H_*/
