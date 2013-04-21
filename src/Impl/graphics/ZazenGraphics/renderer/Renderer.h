#ifndef RENDERER_H_
#define RENDERER_H_

#include "../Scene/Viewer.h"
#include "../Scene/Instance.h"
#include "../Lighting/Light.h"

#include <list>

class Renderer
{
 public:
	Renderer();
	virtual ~Renderer();

	void setCamera( Viewer* cam ) { this->m_camera = cam; };

	virtual bool initialize() = 0;
	virtual bool shutdown() = 0;
	virtual bool renderFrame( std::list<Instance*>& instances, std::list<Light*>& lights ) = 0;

	void printInfo();

 protected:
	 long long frame;

	 Viewer* m_camera;

	 static bool geomInstanceDistCmp( Instance* a, Instance* b )
	 {
	 	return a->distance < b->distance;
	 }
};

#endif /*RENDERER_H_*/
