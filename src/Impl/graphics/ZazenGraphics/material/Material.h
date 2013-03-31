/*
 * Material.h
 *
 *  Created on: Feb 25, 2011
 *      Author: jonathan
 */

#ifndef MATERIAL_H_
#define MATERIAL_H_

#include "Program.h"
#include "UniformBlock.h"
#include "Texture.h"

#include <boost/filesystem.hpp>

#include <glm/glm.hpp>

#include <string>
#include <map>
#include <vector>

class Material
{
	public:
		enum MaterialType
		{
			MATERIAL_DIFFUSE = 0,
			MATERIAL_LAMBERTIAN,
			MATERIAL_PHONG,
			MATERIAL_ORENNAYAR,
			MATERIAL_SSS,
			MATERIAL_WARDS,
			MATERIAL_TORRANCESPARROW,
			MATERIAL_TRANSPARENT = 99
		};

		static bool init( const boost::filesystem::path& );
		static void freeAll();

		static Material* get( const std::string& );

		~Material();

		const std::string& getName() const { return this->m_name; };
		MaterialType getType() const { return this->m_type; };

		bool activate( UniformBlock*, Program* );
		bool deactivate();

	private:
		static std::map<std::string, Material*> allMaterials;

		Material( const std::string&, MaterialType );

		const std::string m_name;
		const MaterialType m_type;

		glm::vec4 m_color;
		glm::vec4 m_genericParams1;
		glm::vec4 m_genericParams2;

		Texture* m_normalMap;
		Texture* m_diffuseTexture;
};

#endif /* MATERIAL_H_ */
