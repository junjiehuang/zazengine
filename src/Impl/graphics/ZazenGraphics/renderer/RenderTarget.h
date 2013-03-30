/*
 * RenderTarget.h
 *
 *  Created on: 17.03.2013
 *      Author: Jonathan Thaler
 */

#ifndef RENDERTARGET_H_
#define RENDERTARGET_H_

#include <GL/glew.h>

#include <vector>

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
		// TODO need a cleanup of shadow-map pool

		GLuint getId() { return this->m_id; };

		GLsizei getWidth() { return this->m_width; };
		GLsizei getHeight() { return this->m_height; };

		bool bind( unsigned int );
		bool unbind();

		RenderTargetType getType() { return this->m_targetType; };

	private:
		RenderTarget( GLuint, GLsizei, GLsizei, RenderTargetType );
		~RenderTarget();

		GLuint m_id;
		GLuint m_boundIndex;

		GLsizei m_width;
		GLsizei m_height;

		RenderTargetType m_targetType;

		static std::vector<RenderTarget*> m_shadowMapPool;

		static RenderTarget* findShadowMapInPool( GLsizei, GLsizei );
};

#endif /* RENDERTARGET_H_ */