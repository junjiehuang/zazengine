/*
 * RenderTarget.h
 *
 *  Created on: 17.03.2013
 *      Author: Jonathan Thaler
 */

#ifndef RENDERTARGET_H_
#define RENDERTARGET_H_

#include <GL/glew.h>

class RenderTarget
{
	public:
		enum RenderTargetType
		{
			RT_COLOR = 0,
			RT_DEPTH,
			RT_SHADOW
		};

		static RenderTarget* create( GLsizei, GLsizei, RenderTargetType );
		static bool destroy( RenderTarget* );

		GLuint getId() { return this->m_id; };

		bool bind( unsigned int );
		bool unbind();

		RenderTargetType getType() { return this->m_targetType; };

	private:
		RenderTarget( GLuint, RenderTargetType );
		~RenderTarget();

		GLuint m_id;
		GLuint m_boundIndex;
		RenderTargetType m_targetType;
};

#endif /* RENDERTARGET_H_ */