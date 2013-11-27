#ifndef _TEXTURE_H_
#define _TEXTURE_H_

#include <GL/glew.h>

class Texture
{
	public:
		enum TextureType {
			TEXTURE_2D,
			TEXTURE_CUBE
		};

		Texture( GLuint, TextureType );
		virtual ~Texture();

		GLuint getId() const { return this->m_id; };

		bool bind( int textureUnit );
		bool unbind();

	private:
		static GLint m_currentTextureUnit;

		GLuint m_id;
		GLint m_boundToUnit;
		TextureType m_textureType;

};

#endif /* _TEXTURE_H_ */
