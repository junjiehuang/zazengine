#ifndef _SHADER_H_
#define _SHADER_H_

#include <GL/glew.h>

#include <string>

class Shader
{
	public:
		enum ShaderType {
			GEOMETRY_SHADER,
			VERTEX_SHADER,
			FRAGMENT_SHADER
		};

		static Shader* createShader( Shader::ShaderType type, const std::string& file );

		bool compile();

		void printInfoLog();

		const std::string& getFileName() { return this->m_fileName; };

		GLuint getObject() const { return this->m_shaderObject; };

		~Shader();

	private:
		Shader( GLuint shaderObject, const std::string& );

		std::string m_fileName;

		GLuint m_shaderObject;

		static bool readShaderSource( const std::string&, std::string& );
};

#endif /* _SHADER_H_ */
