/*
 * MaterialFactory.h
 *
 *  Created on: May 09, 2013
 *      Author: jonathan
 */

#ifndef MATERIALFACTORY_H_
#define MATERIALFACTORY_H_

#include "Material.h"

#include <core/ICore.h>

#include <boost/filesystem.hpp>

#include <string>
#include <map>

class MaterialFactory
{
	public:
		static bool init( const boost::filesystem::path& );
		static void freeAll();

		static Material* get( const std::string& );

	private:
		static std::map<std::string, Material*> allMaterials;

		static Material* createClassicMaterial( const std::string&, Material::MaterialType, TiXmlElement* );
		static Material* createDoom3Material( const std::string&, TiXmlElement* );
		static Material* createTransparentMaterial( const std::string&, TiXmlElement* );
};

#endif /* MATERIAL_H_ */
