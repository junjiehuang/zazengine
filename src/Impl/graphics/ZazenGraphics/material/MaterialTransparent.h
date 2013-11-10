#ifndef _MATERIAL_TRANSPARENT_H_
#define _MATERIAL_TRANSPARENT_H_

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

#endif /* _MATERIAL_TRANSPARENT_H_ */
