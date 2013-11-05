/*
 * MaterialTransparent.h
 *
 *  Created on: May 09, 2013
 *      Author: jonathan
 */


#ifndef MATERIALTRANSPARENT_H_
#define MATERIALTRANSPARENT_H_

#include "Material.h"

#include "../Texture/Texture.h"

class MaterialTransparent : public Material
{
	public:
		friend class MaterialFactory;

		MaterialTransparent( const std::string& );
		virtual ~MaterialTransparent();

		bool activate( Program* );

	private:
		UniformBlock* m_materialConfig;

		float m_blendingFactor;
		float m_refractionFactor;

		Texture* m_diffuseTexture;
		Texture* m_normalTexture;
};

#endif /* MATERIAL_H_ */
