/*
 * Shader.h
 *
 *  Created on: Jan 23, 2011
 *      Author: jonathan
 */

#ifndef SHADER_H_
#define SHADER_H_

#include <GL/glew.h>

#include <string>

class Shader
{
 public:
	enum ShaderType {
		VERTEX_SHADER,
		FRAGMENT_SHADER,
		GEOMETRY_SHADER
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

#endif /* SHADER_H_ */
