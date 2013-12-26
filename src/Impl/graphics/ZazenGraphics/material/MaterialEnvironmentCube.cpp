#include "MaterialEnvironmentCube.h"

#include "../Program/UniformManagement.h"

using namespace std;

bool
MaterialEnvironmentCube::activate( Program* currentProgramm )
{
	// TODO: upload params
	return true;
}

MaterialEnvironmentCube::MaterialEnvironmentCube( const std::string& name )
	: Material( name, Material::MATERIAL_ENVIRONMENT_CUBE )
{
	this->m_materialParamsBlock = UniformManagement::getBlock( "EnvironmentCubeMaterialUniforms" );
}

MaterialEnvironmentCube::~MaterialEnvironmentCube()
{
}
