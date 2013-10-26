/*
 * MaterialDoom3.h
 *
 *  Created on: Oct 22, 2013
 *      Author: jonathan
 */

#ifndef MATERIALDOOM3_H
#define MATERIALDOOM3_H

#include "Material.h"

#include "../Program/UniformBlock.h"
#include "../Texture/Texture.h"

class MaterialDoom3 : public Material
{
	public:
		friend class MaterialFactory;

		MaterialDoom3( const std::string& );
		virtual ~MaterialDoom3();

		bool activate( Program* );

	private:
		UniformBlock* m_materialConfig;

		Texture* m_diffuseTexture;
		Texture* m_specularTexture;
		Texture* m_normalMap;
};

#endif /* MATERIAL_H_ */
