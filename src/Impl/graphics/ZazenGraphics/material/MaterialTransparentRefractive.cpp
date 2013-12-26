#include "MaterialTransparentRefractive.h"

#include "../Program/UniformManagement.h"

using namespace std;

bool
MaterialTransparentRefractive::activate( Program* currentProgramm )
{
	// TODO: implement to allow a rgb color instead of texture
	// set through subroutine? or if/then
	if ( this->m_diffuseTexture )
	{
		this->m_diffuseTexture->bind( 0 );
	}

	if ( this->m_normalTexture )
	{
		this->m_normalTexture->bind( 1 );
	}

	this->m_materialParamsBlock->bindBuffer();
	this->m_materialParamsBlock->updateField( "TransparentRefractiveMaterialUniforms.params", m_materialParams );

	return true;
}

MaterialTransparentRefractive::MaterialTransparentRefractive( const std::string& name )
	: Material( name, Material::MATERIAL_TRANSPARENT_REFRACTIVE )
{
	this->m_diffuseTexture = NULL;
	this->m_normalTexture = NULL;

	this->m_materialParamsBlock = UniformManagement::getBlock( "TransparentRefractiveMaterialUniforms" );
}

MaterialTransparentRefractive::~MaterialTransparentRefractive()
{
}
