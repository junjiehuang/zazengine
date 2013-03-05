#ifndef SFXRENDERER_H_
#define SFXRENDERER_H_

#include "Renderer.h"

#include <list>
#include <string>
#include <vector>

class SFXRenderer : public Renderer
{
 public:
	SFXRenderer(Camera& camera, std::string&);
	 virtual ~SFXRenderer();

	 // renders this list of geominstances which must be in front-to-back order
	 bool renderFrame(GeomInstance*);

 private:
	 bool parentIntersectingFrustum;

	 std::list<GeomInstance*> instancesQueue;
	 std::list<GeomInstance*> renderQueue;

	 void processRenderQueue();
	 void traverseInstance(GeomInstance*);
	 void processInstance(GeomInstance*);

	 GLuint backgroundTex;
	 GLuint backgroundFBO;

	 void createFBO(GLuint*, GLuint*);
};

#endif /*SFXRENDERER_H_*/
