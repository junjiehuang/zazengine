/*
 * Program.h
 *
 *  Created on: Jan 16, 2011
 *      Author: jonathan
 */

#ifndef PROGRAM_H_
#define PROGRAM_H_

#include "Shader.h"
#include "UniformBlock.h"

#include <map>

class Program
{
 public:
	~Program();

	static Program* createProgram( const std::string& );

	void printInfoLog();

	bool attachShader( Shader* );
	bool detachShader( Shader* );

	bool setUniformMatrix4( const std::string& name, const float* );
	bool setUniform4( const std::string& name, const float* );
	bool setUniformInt( const std::string& name, int );

	bool bindUniformBlock( UniformBlock* );

	bool bindAttribLocation( GLuint index, const std::string& name );
	GLint getAttribLocation( const std::string& name );

	bool bindFragDataLocation( GLuint colorNumber, const std::string& name );
	GLint getFragDataLocation( const std::string& name );

	bool link();

	bool use();
	static bool unuse();

 private:
	Program( GLuint programObject, const std::string& programName );

	std::string m_programName;
	GLuint m_programObject;
	std::map<std::string, GLint> m_uniformLocations;

	GLuint getUniformBlockIndex( const std::string& name );
	GLint getUniformLocation( const std::string& name );
	static void printInfoLog( GLuint obj );
};

#endif /* PROGRAM_H_ */
