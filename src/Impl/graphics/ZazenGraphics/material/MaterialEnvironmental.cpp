#include "MaterialEnvironmental.h"

#include "../Program/UniformManagement.h"

using namespace std;

bool
MaterialEnvironmental::activate( Program* currentProgramm )
{
	// TODO: upload params
	return true;
}

MaterialEnvironmental::MaterialEnvironmental( const std::string& name )
	: Material( name, Material::MATERIAL_ENVIRONMENTAL_CUBE )
{
	this->m_materialParamsBlock = UniformManagement::getBlock( "EnvironmentalCubeMaterialUniforms" );
}

MaterialEnvironmental::~MaterialEnvironmental()
{
}
