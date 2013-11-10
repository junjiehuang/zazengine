#ifndef _MATERIAL_CLASSIC_H_
#define _MATERIAL_CLASSIC_H_

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

#endif /* _MATERIAL_CLASSIC_H_ */
