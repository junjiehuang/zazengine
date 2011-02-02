#include "Scene.h"

#ifdef OCCLUSION_CULLING
	#include "../Renderer/OCRenderer.h"
#elif SFX_RENDERING
	#include "../Renderer/SFXRenderer.h"
#elif DR_RENDERING
	#include "../Renderer/DRRenderer.h"
#else
	#include "../Renderer/StandardRenderer.h"
#endif

#include "../Geometry/GeometryFactory.h"

#include <iostream>

using namespace std;

Scene::Scene(const string& name, Camera* camera)
	: name(name)
{
	this->camera = camera;
	this->renderer = 0;
}

Scene::~Scene()
{	
	delete this->renderer;
	delete this->camera;
}

bool
Scene::load(bool randomizeInstances, int rows, int columns, int density)
{
	if (this->renderer)
		delete this->renderer;

#ifdef OCCLUSION_CULLING
	this->renderer = new OCRenderer(*this->camera, this->skyBoxFolder);
#elif SFX_RENDERING
	this->renderer = new SFXRenderer(*this->camera, this->skyBoxFolder);
#elif DR_RENDERING
	this->renderer = new DRRenderer(*this->camera, this->skyBoxFolder);
#else
	this->renderer = new StandardRenderer( *this->camera, this->skyBoxFolder );
#endif
	
	if ( false == this->renderer->initialize() )
	{
		cout << "ERROR ... initializing renderer failed - exit" << endl;
		return false;
	}

	map<string, EntityDefinition>::iterator iter = this->entitiesDef.begin();
	while ( iter != this->entitiesDef.end() )
	{
		EntityDefinition& entity = iter->second;

		if ( entity.modelFile != "" )
			GeometryFactory::loadMesh( entity.name, entity.modelFile );

		iter++;
	}

	for (unsigned int i = 0; i < this->instanceDef.size(); i++)
	{
		InstanceDefinition* instanceDef = this->instanceDef[ i ];
		EntityDefinition entityDef = this->entitiesDef[ instanceDef->entity ];

		GeomType* model = GeometryFactory::get( instanceDef->entity );
		if ( 0 == model )
		{
			cout << "ERROR ... instance \"" << instanceDef->entity << "\" has no model - ignoring instance" << endl;
			continue;
		}

		Instance* newInstance = new Instance( model );
		newInstance->transform.matrix = instanceDef->transform->matrix;

		this->instances.push_back( newInstance );
	}
		
	this->sceneMeasures.data[0] = this->sceneBBMax[0] - this->sceneBBMin[0];
	this->sceneMeasures.data[1] = this->sceneBBMax[1] - this->sceneBBMin[1];
	this->sceneMeasures.data[2] = this->sceneBBMax[2] - this->sceneBBMin[2];
	
	return true;
}

void
Scene::setSceneBB(const Vector& sceneBBMin, const Vector& sceneBBMax)
{
	this->sceneBBMax = sceneBBMax;
	this->sceneBBMin = sceneBBMin;

	this->sceneMeasures.data[0] = this->sceneBBMax[0] - this->sceneBBMin[0];
	this->sceneMeasures.data[1] = this->sceneBBMax[1] - this->sceneBBMin[1];
	this->sceneMeasures.data[2] = this->sceneBBMax[2] - this->sceneBBMin[2];
}

bool
Scene::processFrame(double loopFactor)
{	
	return this->renderer->renderFrame( this->instances );
}

void
Scene::printInfo()
{
}
