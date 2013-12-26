#include "MaterialTransparentClassic.h"

#include "../Program/UniformManagement.h"

using namespace std;

bool
MaterialTransparentClassic::activate( Program* currentProgramm )
{
	// TODO: implement to allow a rgb color instead of texture
	// set through subroutine? or if/then
	if ( this->m_diffuseTexture )
	{
		this->m_diffuseTexture->bind( 0 );
	}

	this->m_materialParamsBlock->bindBuffer();
	this->m_materialParamsBlock->updateField( "TransparentClassicMaterialUniforms.params", this->m_materialParams );

	return true;
}

MaterialTransparentClassic::MaterialTransparentClassic( const std::string& name )
	: Material( name, Material::MATERIAL_TRANSPARENT_CLASSIC )
{
	this->m_diffuseTexture = NULL;
	this->m_materialParamsBlock = UniformManagement::getBlock( "TransparentClassicMaterialUniforms" );
}

MaterialTransparentClassic::~MaterialTransparentClassic()
{
}
