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
			// NOTE: start with 1, material 0 is reserved for sky-box/background
			MATERIAL_LAMBERTIAN = 1,
			MATERIAL_PHONG,
			MATERIAL_DOOM3,
			MATERIAL_TRANSPARENT = 100,
			MATERIAL_TRANSPARENT_PLANAR,
			MATERIAL_TRANSPARENT_ENVIRONMENT
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
