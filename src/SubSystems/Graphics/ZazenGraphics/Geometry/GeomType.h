#ifndef GEOMTYPE_H_
#define GEOMTYPE_H_

#include <GL/glew.h>

#include "../../../../Core/Utils/Math/Vector.h"
#include "../../../../Core/Utils/Math/Matrix.h"
#include "../Material/Material.h"

#include <vector>

class GeomType
{
 public:
	 GeomType();
	 virtual ~GeomType();

	 Material* material;

	 void compareBB(const Vector&, const Vector&);
	 
	 const Vector& getCenter() { return this->center; };
	 const Vector& getBBMin() { return this->bbMin; };
	 const Vector& getBBMax() { return this->bbMax; };
	 
	 void setBB(const Vector&, const Vector&);
	 
	 virtual int getFaceCount() { return 0; };
	 
	 void renderBB();
	 virtual bool render();

	 GeomType* parent;
	 std::vector<GeomType*> children;
	 
 private:
	 GLuint bbVBO;
		
	 Vector bbMin;
	 Vector bbMax;
	 Vector center;

};

#endif /*GEOMNODE_H_*/
