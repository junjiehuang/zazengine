#ifndef GEOMBOX_H_
#define GEOMBOX_H_

#include <GL/glut.h>

#include "GeomType.h"

class GeomBox : public GeomType
{
 public:
	GeomBox(float size) { this->size = size; };
	virtual ~GeomBox() {};
 
	void render() { glutSolidCube(this->size); };

 private:
	float size;
	
};

#endif /*BOXNODE_H_*/
