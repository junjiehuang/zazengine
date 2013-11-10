#ifndef _RENDERTARGET_H_
#define _RENDERTARGET_H_

#include "../Texture/Texture.h"

#include <GL/glew.h>

#include <vector>

class RenderTarget : public Texture
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
		static void cleanup();

		GLsizei getWidth() const { return this->m_width; };
		GLsizei getHeight() const { return this->m_height; };

		RenderTargetType getType() { return this->m_targetType; };

	private:
		RenderTarget( GLuint, GLsizei, GLsizei, RenderTargetType );
		~RenderTarget();

		GLsizei m_width;
		GLsizei m_height;

		RenderTargetType m_targetType;

		static std::vector<RenderTarget*> m_shadowMapPool;

		static RenderTarget* findShadowMapInPool( GLsizei, GLsizei );
};

#endif /* _RENDERTARGET_H_ */