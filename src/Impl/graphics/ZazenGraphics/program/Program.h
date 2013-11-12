#ifndef _PROGRAM_H_
#define _PROGRAM_H_

#include "Shader.h"
#include "UniformBlock.h"

#include <map>
#include <vector>

class Program
{
	public:
		friend class UniformManagement;

		static Program* createProgram( const std::string& );

		void printInfoLog();

		GLuint getId() const { return this->m_programObject; };

		bool attachShader( Shader* );
		bool detachShader( Shader* );

		bool activateSubroutine( const std::string&, Shader::ShaderType );

		bool setUniformInt( const std::string& name, int );
		bool setUniformMatrices( const std::string& name, const std::vector<glm::mat4>& );

		bool bindUniformBlock( UniformBlock* );

		bool bindAttribLocation( GLuint index, const std::string& name );

		bool bindFragDataLocation( GLuint colorNumber, const std::string& name );

		bool link();

		bool use();
		static bool unuse();

		~Program();

	private:
		struct UniformField {
			GLuint m_index;
			GLenum m_type;
			GLint m_size;
			std::string m_name;
		};

		struct Subroutine {
			std::string m_name;
			GLint m_index;
			GLint m_uniformIndex;
		};

		Program( GLuint programObject, const std::string& programName );

		std::string m_programName;
		GLuint m_programObject;

		std::map<std::string, UniformField> m_uniforms;
		std::map<GLenum, std::vector<Subroutine>> m_subroutines;
		std::map<GLenum, std::vector<Subroutine>> m_activeSubroutines;
		std::map<GLenum, std::vector<GLuint>> m_activeSubroutineConfig;

		GLuint getUniformBlockIndex( const std::string& name );
		UniformField* getUniformField( const std::string& name );

		static void printInfoLog( GLuint obj );
};

#endif /* _PROGRAM_H_ */
