/*
 * Material.cpp
 *
 *  Created on: Feb 25, 2011
 *      Author: jonathan
 */

#include "Material.h"

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
		cout << "ERROR ... could not load file " << fullFileName << " - reason = " << doc.ErrorDesc() << " at row = " << doc.ErrorRow() << " col = " << doc.ErrorCol() << endl;
		return false;
	}

	TiXmlElement* rootNode = doc.FirstChildElement("materials");
	if ( 0 == rootNode )
	{
		cout << "ERROR ... root-node \"materials\" in " << fullFileName << " not found" << endl;
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
				cout << "No name for material - will be ignored" << endl;
				continue;
			}
			else
			{
				name = str;
			}

			TiXmlElement* materialTypeNode = materialNode->FirstChildElement("type");
			if ( 0 == materialTypeNode )
			{
				cout << "ERROR ... node \"type\" for material " << name << " not found" << endl;
				return false;
			}

			str = materialTypeNode->Attribute( "id" );
			if ( 0 == str )
			{
				cout << "No name for material - will be ignored" << endl;
				continue;
			}
			else
			{
				typeID = str;
			}

			if ( "LAMBERTIAN" == typeID )
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
							Texture* texture = Texture::get( str );
							if ( texture )
							{
								material->m_diffuseTexture = texture;
							}
						}
					}
				}
				else if ( 0 == strcmp( str, "normalMap" ) )
				{
					if ( 0 == material->m_normalMap )
					{
						str = materialCfgNode->Attribute( "file" );
						if ( 0 != str )
						{
							Texture* texture = Texture::get( str );
							if ( texture )
							{
								material->m_normalMap = texture;
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
	if (findIter != Material::allMaterials.end())
		return findIter->second;

	return 0;
}

bool
Material::activate( UniformBlock* materialUniforms )
{
	if ( this->m_diffuseTexture )
		this->m_diffuseTexture->bind( 0 );

	if ( this->m_normalMap )
		this->m_normalMap->bind( 1 );

	glm::vec4 materialCfg;
	materialCfg[ 0 ] = ( float ) this->m_type;
	materialCfg[ 1 ] = this->m_diffuseTexture == 0 ? 0.0f : 1.0f;
	materialCfg[ 2 ] = this->m_normalMap == 0 ? 0.0f : 1.0f;

	if ( false == materialUniforms->updateData( glm::value_ptr( materialCfg ), 0, 16 ) )
		return false;

	if ( false == materialUniforms->updateData( glm::value_ptr( this->m_genericParams1 ), 16, 16 ) )
		return false;

	if ( false == materialUniforms->updateData( glm::value_ptr( this->m_genericParams1 ), 32, 16 ) )
		return false;

	if ( false == materialUniforms->updateData( glm::value_ptr( this->m_color ), 48, 16 ) )
		return false;

	return true;
}

Material::Material( const std::string& name, MaterialType type )
	: m_name( name ),
	  m_type( type )
{
	this->m_normalMap = 0;
	this->m_diffuseTexture = 0;
}

Material::~Material()
{
	// don't delete textures, they are shared
}
