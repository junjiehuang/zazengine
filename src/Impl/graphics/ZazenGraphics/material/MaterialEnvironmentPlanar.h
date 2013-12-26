#ifndef _MATERIAL_ENVIRONMENT_PLANAR_H_
#define _MATERIAL_ENVIRONMENT_PLANAR_H_

#include "Material.h"

class MaterialEnvironmentPlanar : public Material
{
	public:
		friend class MaterialFactory;

		MaterialEnvironmentPlanar( const std::string& );
		virtual ~MaterialEnvironmentPlanar();

		bool activate( Program* );

	private:
		UniformBlock* m_materialParamsBlock;

};

#endif /* _MATERIAL_ENVIRONMENT_PLANAR_H_ */
