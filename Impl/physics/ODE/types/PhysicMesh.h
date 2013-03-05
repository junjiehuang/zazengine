#ifndef PHYSICMESH_H_
#define PHYSICMESH_H_

#include "PhysicType.h"

#include <string>

class PhysicMesh : public PhysicType
{
public:
	PhysicMesh(bool, float, const std::string&);
	virtual ~PhysicMesh();
	
	virtual bool create(dWorldID, dSpaceID);
	
 private:
	 const std::string meshFile;
	 dTriMeshDataID meshID;
};

#endif /*PHYSICMESH_H_*/
