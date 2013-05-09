#ifndef INSTANCE_H_
#define INSTANCE_H_

#include "../Material/Material.h"
#include "../Geometry/GeomType.h"
#include "../Orientation/Orientation.h"

class Instance : public Orientation
{
 public:
	Instance();
	~Instance();

	bool visible;
	float distance;
	long lastFrame;

	Material* material;
	GeomType* geom;

	glm::mat4 m_modelMatrix;

	void recalculateDistance( const glm::mat4& viewMatrix );

};

#endif /*INSTANCE_H_*/
