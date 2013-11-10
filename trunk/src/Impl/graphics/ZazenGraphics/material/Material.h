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
			MATERIAL_DIFFUSE = 0,
			MATERIAL_LAMBERTIAN,
			MATERIAL_PHONG,
			MATERIAL_DOOM3,
			MATERIAL_TRANSPARENT = 99
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
