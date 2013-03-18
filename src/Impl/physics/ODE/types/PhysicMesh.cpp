#include "PhysicMesh.h"

#include <iostream>

using namespace std;

PhysicMesh::PhysicMesh( bool staticFlag, float mass, const string& meshFile )
	: PhysicType( staticFlag, mass ),
	m_meshFile(meshFile)
{
}

PhysicMesh::~PhysicMesh()
{
	dGeomTriMeshDataDestroy( this->m_meshID );
}

bool PhysicMesh::create( dWorldID worldID, dSpaceID spaceID )
{
	return false;

	/*
	GeomType* model = Model::get(this->meshFile);
	if (model == 0) {
		cout << "ERROR ... couldn't load physic-mesh " << meshFile << endl;
		return false;
	}
		
	return false;
	

	this->bodyID = 0;
	this->meshID = dGeomTriMeshDataCreate();
	this->geomID = dCreateTriMesh(spaceID, this->meshID, 0, 0, 0);

	dGeomTriMeshDataBuildSingle1(this->meshID, model->getVertices(), 3 * sizeof(float), model->getVertextCount(), model->getIndices(), model->getVertextCount(), 3 * sizeof(int), model->getNormals());

	if (this->staticFlag == false) {
		this->bodyID = dBodyCreate(worldID);
		
		dMass massStruct;
		dMassSetZero(&massStruct);
		dMassSetSphereTotal(&massStruct, mass, 1);
		
		dBodySetMass(this->bodyID, &massStruct);
		dGeomSetBody (this->geomID, this->bodyID);

	} else {
		dGeomSetBody(this->geomID, 0);
	}
	
	return true;
	*/
}
