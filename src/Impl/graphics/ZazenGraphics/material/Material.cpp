/*
 * Material.cpp
 *
 *  Created on: Feb 25, 2011
 *      Author: jonathan
 */

#include "Material.h"

#include "../ZazenGraphics.h"

#include "../Texture/TextureFactory.h"

#include <glm/gtc/type_ptr.hpp>

#include <iostream>

#include <core/XML/tinyxml.h>

using namespace std;
using namespace boost;

std::map<std::string, Material*> Material::allMaterials;

bool
Material::init( const filesystem::path& path )
{
	string fullFileName = path.generic_string() + "materials.xml";

	TiXmlDocument doc( fullFileName.c_str() );

	if ( false == doc.LoadFile() )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "could not load file " << fullFileName << " - reason = " << doc.ErrorDesc() << " at row = " << doc.ErrorRow() << " col = " << doc.ErrorCol();
		return false;
	}

	TiXmlElement* rootNode = doc.FirstChildElement( "materials" );
	if ( 0 == rootNode )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "root-node \"materials\" in " << fullFileName << " not found";
		return false;
	}

	for (TiXmlElement* materialNode = rootNode->FirstChildElement(); materialNode != 0; materialNode = materialNode->NextSiblingElement())
	{
		const char* str = materialNode->Value();
		if (str == 0)
			continue;

		if ( 0 == strcmp(str, "material") )
		{
			std::string name;
			std::string typeID;
			Material::MaterialType materialType = Material::MATERIAL_DIFFUSE;

			str = materialNode->Attribute( "name" );
			if ( 0 == str )
			{
				ZazenGraphics::getInstance().getLogger().logWarning( "No name for material - will be ignored" );
				continue;
			}
			else
			{
				name = str;
			}

			TiXmlElement* materialTypeNode = materialNode->FirstChildElement( "type" );
			if ( 0 == materialTypeNode )
			{
				ZazenGraphics::getInstance().getLogger().logWarning() << "node \"type\" for material " << name << " not found";
				return false;
			}

			str = materialTypeNode->Attribute( "id" );
			if ( 0 == str )
			{
				ZazenGraphics::getInstance().getLogger().logWarning( "No id for material-type - will be ignored" );
				continue;
			}
			else
			{
				typeID = str;
			}

			if ( "DIFFUSE" == typeID )
			{
				materialType = Material::MATERIAL_DIFFUSE;
			}
			else if ( "LAMBERTIAN" == typeID )
			{
				materialType = Material::MATERIAL_LAMBERTIAN;
			}
			else if ( "PHONG" == typeID )
			{
				materialType = Material::MATERIAL_PHONG;
			}
			else if ( "ORENNAYAR" == typeID )
			{
				materialType = Material::MATERIAL_ORENNAYAR;
			}
			else if ( "SSS" == typeID )
			{
				materialType = Material::MATERIAL_SSS;
			}
			else if ( "WARDS" == typeID )
			{
				materialType = Material::MATERIAL_WARDS;
			}
			else if ( "TORRANCESPARROW" == typeID )
			{
				materialType = Material::MATERIAL_TORRANCESPARROW;
			}
			else if ( "TRANSPARENT" == typeID )
			{
				materialType = Material::MATERIAL_TRANSPARENT;
			}

			Material* material = new Material( name, materialType );

			for (TiXmlElement* materialCfgNode = materialTypeNode->FirstChildElement(); materialCfgNode != 0; materialCfgNode = materialCfgNode->NextSiblingElement())
			{
				str = materialCfgNode->Value();
				if (str == 0)
					continue;

				if ( 0 == strcmp( str, "diffuseColor" ) )
				{
					if ( 0 == material->m_diffuseTexture )
					{
						str = materialCfgNode->Attribute( "file" );
						if ( 0 != str )
						{
							Texture* texture = TextureFactory::get( str );
							if ( texture )
							{
								material->m_diffuseTexture = texture;
							}
						}
					}
				}
				else if ( 0 == strcmp( str, "color" ) )
				{
					glm::vec4 color;

					str = materialCfgNode->Attribute( "r" );
					if ( 0 != str )
					{
						color.r = ( float ) atof( str );
					}

					str = materialCfgNode->Attribute( "g" );
					if ( 0 != str )
					{
						color.g = ( float ) atof( str );
					}

					str = materialCfgNode->Attribute( "b" );
					if ( 0 != str )
					{
						color.b = ( float ) atof( str );
					}

					material->m_color = color;
				}
			}

			Material::allMaterials.insert( make_pair( material->m_name, material ) );
		}
	}

	return true;
}

void
Material::freeAll()
{
	map<string, Material*>::iterator iter = Material::allMaterials.begin();
	while(iter != Material::allMaterials.end())
	{
		delete iter->second;
		iter++;
	}

	Material::allMaterials.clear();
}

Material*
Material::get( const std::string& name )
{
	map<string, Material*>::iterator findIter = Material::allMaterials.find( name );
	if ( findIter != Material::allMaterials.end() )
	{
		return findIter->second;
	}

	return NULL;
}

bool
Material::activate( UniformBlock* materialUniforms, Program* currentProgramm )
{
	if ( this->m_diffuseTexture )
	{
		this->m_diffuseTexture->bind( 0 );
		currentProgramm->setUniformInt( "DiffuseTexture", 0 );
	}

	glm::vec4 materialCfg;
	materialCfg[ 0 ] = ( float ) this->m_type;
	materialCfg[ 1 ] = this->m_diffuseTexture == 0 ? 0.0f : 1.0f;

	// IMPORTANT: materialUniforms->bindBuffer() must have been already called by client 
	if ( false == materialUniforms->updateVec4( materialCfg, 0 ) )
	{
		return false;
	}

	if ( false == materialUniforms->updateVec4( this->m_color, 16 ) )
	{
		return false;
	}

	return true;
}

Material::Material( const std::string& name, MaterialType type )
	: m_name( name ),
	  m_type( type )
{
	this->m_diffuseTexture = NULL;
}

Material::~Material()
{
	// don't delete textures, they are shared
}