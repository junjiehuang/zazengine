#ifndef STDRENDERER_H_
#define STDRENDERER_H_

#include "Renderer.h"

#include <list>

class StandardRenderer : public Renderer
{
 public:
	StandardRenderer( Camera& camera, std::string& );
	virtual ~StandardRenderer();

	virtual bool initialize();
	virtual bool shutdown();

	 // renders this list of geominstances which must be in front-to-back order
	 bool renderFrame( std::list<Instance*>& instances );

 private:
	 bool renderGeom( Matrix& transf, GeomType* );
};

#endif /*STDRENDERER_H_*/
