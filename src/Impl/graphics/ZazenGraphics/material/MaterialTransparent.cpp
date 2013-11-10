#include "MaterialTransparent.h"

#include "../Program/UniformManagement.h"

using namespace std;

bool
MaterialTransparent::activate( Program* currentProgramm )
{
	if ( this->m_diffuseTexture )
	{
		this->m_diffuseTexture->bind( 0 );
		currentProgramm->setUniformInt( "DiffuseTexture", 0 );
	}

	if ( this->m_normalTexture )
	{
		this->m_normalTexture->bind( 1 );
		currentProgramm->setUniformInt( "NormalTexture", 1 );
	}

	glm::vec4 materialCfg;
	materialCfg[ 0 ] = this->m_blendingFactor;
	materialCfg[ 1 ] = this->m_refractionFactor;

	this->m_materialConfig->bindBuffer();

	this->m_materialConfig->updateField( "TransparentMaterialUniforms.config", materialCfg );

	return true;
}

MaterialTransparent::MaterialTransparent( const std::string& name )
	: Material( name, Material::MATERIAL_TRANSPARENT )
{
	this->m_diffuseTexture = NULL;
	this->m_normalTexture = NULL;

	this->m_materialConfig = UniformManagement::getBlock( "TransparentMaterialUniforms" );
}

MaterialTransparent::~MaterialTransparent()
{
}
