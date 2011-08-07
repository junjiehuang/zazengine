//#include <GL/glew.h>
//#include <SDL/SDL.h>
//#include <GL/glut.h>
//#include <GL/glu.h>

#include "Renderer.h"

#include <algorithm>
#include <iostream>

using namespace std;

Renderer::Renderer( )
{
	this->m_camera = 0;
	this->frame = 0;
}

Renderer::~Renderer()
{
}
