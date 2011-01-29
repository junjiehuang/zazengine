/*
 * Program.h
 *
 *  Created on: Jan 16, 2011
 *      Author: jonathan
 */

#ifndef PROGRAM_H_
#define PROGRAM_H_

#include "Shader.h"

class Program
{
 public:
	~Program();

	static Program* createProgram();

	void printInfoLog();

	bool attachShader( Shader* );
	bool detachShader( Shader* );

	bool setUniformMatrix4( const std::string& name, const float* );
	bool setUniform4( const std::string& name, const float* );

	bool bindAttribLocation( GLuint index, const std::string& name );
	bool bindFragDataLocation( GLuint colorNumber, const std::string& name );

	bool link();

	bool use();

 private:
	Program( GLuint programObject );

	GLuint programObject;

	GLint getUniformLocation( const std::string& name );
	static void printInfoLog( GLuint obj );
};

#endif /* PROGRAM_H_ */
