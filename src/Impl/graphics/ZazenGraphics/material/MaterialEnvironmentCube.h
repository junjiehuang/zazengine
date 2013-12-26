#ifndef _MATERIAL_ENVIRONMENT_CUBE_H_
#define _MATERIAL_ENVIRONMENT_CUBE_H_

#include "Material.h"

class MaterialEnvironmentCube : public Material
{
	public:
		friend class MaterialFactory;

		MaterialEnvironmentCube( const std::string& );
		virtual ~MaterialEnvironmentCube();

		bool activate( Program* );

	private:
		UniformBlock* m_materialParamsBlock;

};

#endif /* _MATERIAL_ENVIRONMENT_CUBE_H_ */
