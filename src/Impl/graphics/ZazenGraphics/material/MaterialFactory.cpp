#include "MaterialFactory.h"

#include "MaterialClassic.h"
#include "MaterialDoom3.h"
#include "MaterialTransparent.h"

#include "../ZazenGraphics.h"

#include "../Texture/TextureFactory.h"

#include <iostream>

using namespace std;
using namespace boost;

std::map<std::string, Material*> MaterialFactory::allMaterials;

bool
MaterialFactory::init( const filesystem::path& path )
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
		if ( NULL == str )
		{
			continue;
		}

		if ( 0 == strcmp(str, "material") )
		{
			std::string name;
			std::string typeID;
			Material::MaterialType materialType = Material::MATERIAL_LAMBERTIAN;

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

			Material* material = NULL;

			if ( "LAMBERTIAN" == typeID )
			{
				material = MaterialFactory::createClassicMaterial( name, Material::MATERIAL_LAMBERTIAN, materialTypeNode );
			}
			else if ( "PHONG" == typeID )
			{
				material = MaterialFactory::createClassicMaterial( name, Material::MATERIAL_PHONG, materialTypeNode );
			}
			else if ( "DOOM3" == typeID )
			{
				material = MaterialFactory::createDoom3Material( name, materialTypeNode );
			}
			else if ( "TRANSPARENT" == typeID )
			{
				material = MaterialFactory::createTransparentMaterial( name, materialTypeNode );
			}

			if ( NULL != material )
			{
				MaterialFactory::allMaterials.insert( make_pair( material->getName(), material ) );
			}
		}
	}

	return true;
}

void
MaterialFactory::freeAll()
{
	map<string, Material*>::iterator iter = MaterialFactory::allMaterials.begin();
	while(iter != MaterialFactory::allMaterials.end() )
	{
		delete iter->second;
		iter++;
	}

	MaterialFactory::allMaterials.clear();
}

Material*
MaterialFactory::get( const std::string& name )
{
	map<string, Material*>::iterator findIter = MaterialFactory::allMaterials.find( name );
	if ( findIter != MaterialFactory::allMaterials.end() )
	{
		return findIter->second;
	}

	return NULL;
}

Material*
MaterialFactory::createClassicMaterial( const std::string& name, Material::MaterialType type, TiXmlElement* materialTypeNode )
{
	MaterialClassic* material = new MaterialClassic( name, type );

	for (TiXmlElement* materialCfgNode = materialTypeNode->FirstChildElement(); materialCfgNode != 0; materialCfgNode = materialCfgNode->NextSiblingElement() )
	{
		const char* str = materialCfgNode->Value();
		if ( NULL == str )
		{
			continue;
		}

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

	return material;
}

Material*
MaterialFactory::createDoom3Material( const std::string& name, TiXmlElement* materialTypeNode )
{
	MaterialDoom3* material = new MaterialDoom3( name );

	for (TiXmlElement* materialCfgNode = materialTypeNode->FirstChildElement(); materialCfgNode != 0; materialCfgNode = materialCfgNode->NextSiblingElement() )
	{
		const char* str = materialCfgNode->Value();
		if ( NULL == str )
		{
			continue;
		}

		if ( 0 == strcmp( str, "diffuseTexture" ) )
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
		else if ( 0 == strcmp( str, "specularTexture" ) )
		{
			if ( 0 == material->m_specularTexture )
			{
				str = materialCfgNode->Attribute( "file" );
				if ( 0 != str )
				{
					Texture* texture = TextureFactory::get( str );
					if ( texture )
					{
						material->m_specularTexture = texture;
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
					Texture* texture = TextureFactory::get( str );
					if ( texture )
					{
						material->m_normalMap = texture;
					}
				}
			}
		}
	}

	return material;
}

Material*
MaterialFactory::createTransparentMaterial( const std::string& name, TiXmlElement* materialTypeNode )
{
	MaterialTransparent* material = new MaterialTransparent( name );

	for (TiXmlElement* materialCfgNode = materialTypeNode->FirstChildElement(); materialCfgNode != 0; materialCfgNode = materialCfgNode->NextSiblingElement())
	{
		const char* str = materialCfgNode->Value();
		if ( NULL == str )
		{
			continue;
		}

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
		else if ( 0 == strcmp( str, "normalMap" ) )
		{
			if ( 0 == material->m_normalTexture )
			{
				str = materialCfgNode->Attribute( "file" );
				if ( 0 != str )
				{
					Texture* texture = TextureFactory::get( str );
					if ( texture )
					{
						material->m_normalTexture = texture;
					}
				}
			}
		}
		else if ( 0 == strcmp( str, "params" ) )
		{
			str = materialCfgNode->Attribute( "blendingFactor" );
			if ( 0 != str )
			{
				material->m_blendingFactor = ( float ) atof( str );
			}

			str = materialCfgNode->Attribute( "refractionFator" );
			if ( 0 != str )
			{
				material->m_refractionFactor = ( float ) atof( str );
			}
		}
	}

	return material;
}
