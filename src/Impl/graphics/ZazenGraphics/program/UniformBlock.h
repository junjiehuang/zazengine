#ifndef _UNIFORMBLOCK_H_
#define _UNIFORMBLOCK_H_

#include <GL/glew.h>
#include <glm/glm.hpp>

#include <map>
#include <string>

class UniformBlock
{
	public:
		friend class UniformManagement;

		static UniformBlock* createBlock( const std::string& name );

		const std::string& getName() const { return this->m_name; };
		GLuint getID() const { return this->m_id; };

		GLuint getBinding() const { return this->m_binding; };

		bool bindBase();
		bool bindBuffer();

		bool updateData( const void* data, int size );
		bool updateData( const void* data, int offset, int size );

		bool updateField( const std::string&, const glm::mat4& );
		bool updateField( const std::string&, const glm::vec4& );
		bool updateField( const std::string&, const glm::vec3& );
		bool updateField( const std::string&, const glm::vec2& );

		~UniformBlock();

	private:
		struct UniformField {
			GLint m_offset;
			GLenum m_type;
			GLint m_size;
			std::string m_name;
		};

		UniformBlock( GLuint, GLuint, const std::string& name );

		static GLuint m_nextBinding;

		//static int m_currentBoundId;

		GLuint m_id;
		GLuint m_binding;

		const std::string m_name;

		std::map<std::string, UniformField*> m_fields;

		UniformField* getUniformField( const std::string& name );

};

#endif /* _UNIFORMBLOCK_H_ */
