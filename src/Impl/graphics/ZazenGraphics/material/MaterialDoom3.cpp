/*
 * MaterialDoom3.h
 *
 *  Created on: Oct 22, 2013
 *      Author: jonathan
 */

#include "MaterialDoom3.h"

#include "../Program/UniformManagement.h"

using namespace std;

bool
MaterialDoom3::activate( Program* currentProgramm )
{
	if ( this->m_diffuseTexture )
	{
		this->m_diffuseTexture->bind( 0 );
		currentProgramm->setUniformInt( "DiffuseTexture", 0 );
	}

	if ( this->m_specularTexture )
	{
		this->m_specularTexture->bind( 1 );
		currentProgramm->setUniformInt( "SpecularTexture", 1 );
	}

	if ( this->m_heightMap )
	{
		this->m_heightMap->bind( 2 );
		currentProgramm->setUniformInt( "HeightMap", 2 );
	}

	if ( this->m_normalMap )
	{
		this->m_normalMap->bind( 3 );
		currentProgramm->setUniformInt( "NormalMap", 3 );
	}

	glm::vec4 materialCfg;
	materialCfg[ 0 ] = ( float ) this->getType();

	this->m_materialConfig->bindBuffer();

	this->m_materialConfig->updateField( "MaterialUniforms.config", materialCfg );

	return true;
}

MaterialDoom3::MaterialDoom3( const std::string& name )
	: Material( name, Material::MATERIAL_DOOM3 )
{
	this->m_diffuseTexture = NULL;
	this->m_specularTexture = NULL;
	this->m_heightMap = NULL;
	this->m_normalMap = NULL;

	this->m_materialConfig = UniformManagement::getBlock( "MaterialUniforms" );
}

MaterialDoom3::~MaterialDoom3()
{
}
