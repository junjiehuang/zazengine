#include "MaterialEnvironmentPlanar.h"

#include "../Program/UniformManagement.h"

using namespace std;

bool
MaterialEnvironmentPlanar::activate( Program* currentProgramm )
{
	// TODO: upload params
	return true;
}

MaterialEnvironmentPlanar::MaterialEnvironmentPlanar( const std::string& name )
	: Material( name, Material::MATERIAL_ENVIRONMENT_PLANAR )
{
	this->m_materialParamsBlock = UniformManagement::getBlock( "EnvironmentPlanarMaterialUniforms" );
}

MaterialEnvironmentPlanar::~MaterialEnvironmentPlanar()
{
}
