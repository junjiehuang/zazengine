#include "MaterialClassic.h"

#include "../Program/UniformManagement.h"

using namespace std;

bool
MaterialClassic::activate( Program* currentProgramm )
{
	if ( this->m_diffuseTexture )
	{
		this->m_diffuseTexture->bind( 0 );

		currentProgramm->setUniformInt( "DiffuseTexture", 0 );
		currentProgramm->activateSubroutine( "classicMaterialTextured", Shader::FRAGMENT_SHADER );
	}
	else
	{
		currentProgramm->activateSubroutine( "classicMaterial", Shader::FRAGMENT_SHADER );
	}

	glm::vec2 materialCfg;
	materialCfg[ 0 ] = ( float ) this->getType();

	this->m_materialConfig->bindBuffer();
	this->m_materialConfig->updateField( "MaterialUniforms.config", materialCfg );
	this->m_materialConfig->updateField( "MaterialUniforms.color", this->m_color );

	return true;
}

MaterialClassic::MaterialClassic( const std::string& name, MaterialType type )
	: Material( name, type )
{
	this->m_diffuseTexture = NULL;

	this->m_materialConfig = UniformManagement::getBlock( "MaterialUniforms" );
}

MaterialClassic::~MaterialClassic()
{
}
