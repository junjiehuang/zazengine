#ifndef _MATERIAL_TRANSPARENT_REFRACTIVE_H_
#define _MATERIAL_TRANSPARENT_REFRACTIVE_H_

#include "Material.h"

#include "../Texture/Texture.h"

class MaterialTransparentRefractive : public Material
{
	public:
		friend class MaterialFactory;

		MaterialTransparentRefractive( const std::string& );
		virtual ~MaterialTransparentRefractive();

		bool activate( Program* );

	private:
		UniformBlock* m_materialParamsBlock;

		glm::vec2 m_materialParams;

		Texture* m_diffuseTexture;
		Texture* m_normalTexture;
};

#endif /* _MATERIAL_TRANSPARENT_H_ */
