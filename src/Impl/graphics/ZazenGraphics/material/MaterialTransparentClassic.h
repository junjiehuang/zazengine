#ifndef _MATERIAL_TRANSPARENT_CLASSIC_H_
#define _MATERIAL_TRANSPARENT_CLASSIC_H_

#include "Material.h"

#include "../Texture/Texture.h"

class MaterialTransparentClassic : public Material
{
	public:
		friend class MaterialFactory;

		MaterialTransparentClassic( const std::string& );
		virtual ~MaterialTransparentClassic();

		bool activate( Program* );

	private:
		UniformBlock* m_materialParamsBlock;

		Texture* m_diffuseTexture;

		glm::vec2 m_materialParams;

};

#endif /* _MATERIAL_TRANSPARENT_H_ */
