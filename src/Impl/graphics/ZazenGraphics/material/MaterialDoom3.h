#ifndef _MATERIAL_DOOM3_H_
#define _MATERIAL_DOOM3_H_

#include "Material.h"

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

#endif /* _MATERIAL_DOOM3_H_ */
