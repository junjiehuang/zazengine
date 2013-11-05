/*
 * MaterialClassic.h
 *
 *  Created on: May 09, 2013
 *      Author: jonathan
 */

#ifndef MATERIALCLASSIC_H_
#define MATERIALCLASSIC_H_

#include "Material.h"

#include "../Texture/Texture.h"

class MaterialClassic : public Material
{
	public:
		friend class MaterialFactory;

		MaterialClassic( const std::string&, MaterialType );
		virtual ~MaterialClassic();

		bool activate( Program* );

	private:
		UniformBlock* m_materialConfig;

		glm::vec4 m_color;

		Texture* m_diffuseTexture;
};

#endif /* MATERIAL_H_ */
