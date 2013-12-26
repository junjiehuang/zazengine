#ifndef _MATERIAL_ENVIRONMENTAL_H_
#define _MATERIAL_ENVIRONMENTAL_H_

#include "Material.h"

class MaterialEnvironmental : public Material
{
	public:
		friend class MaterialFactory;

		MaterialEnvironmental( const std::string& );
		virtual ~MaterialEnvironmental();

		bool activate( Program* );

	private:
		UniformBlock* m_materialParamsBlock;

		float m_blendingFactor;
		float m_refractionFactor;
};

#endif /* _MATERIAL_TRANSPARENT_H_ */
