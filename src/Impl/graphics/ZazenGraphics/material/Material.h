#ifndef _MATERIAL_H_
#define _MATERIAL_H_

#include "../Program/Program.h"

#include <glm/glm.hpp>

#include <string>

class Material
{
	public:
		enum MaterialType
		{
			MATERIAL_LAMBERTIAN = 1,
			MATERIAL_PHONG,
			MATERIAL_DOOM3,
			MATERIAL_POST_PROCESS = 100, // all materials beyond this marker will be processed in post-process
			MATERIAL_TRANSPARENT_CLASSIC,
			MATERIAL_TRANSPARENT_REFRACTIVE,
			MATERIAL_ENVIRONMENT_CUBE,
			MATERIAL_ENVIRONMENT_PLANAR
		};

		Material( const std::string&, MaterialType );
		virtual ~Material();

		const std::string& getName() const { return this->m_name; };
		MaterialType getType() const { return this->m_type; };

		virtual bool activate( Program* ) = 0;

	private:
		const std::string m_name;
		const MaterialType m_type;
};

#endif /* _MATERIAL_H_ */
