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
			RT_COLOR_CUBE,
			RT_DEPTH,
			RT_DEPTH_STENCIL,
			RT_SHADOW_PLANAR,
			RT_SHADOW_CUBE
		};

		static RenderTarget* create( GLsizei, GLsizei, RenderTargetType );
		static bool destroy( RenderTarget* );
		static void cleanup();

		GLsizei getWidth() const { return this->m_width; };
		GLsizei getHeight() const { return this->m_height; };

		RenderTargetType getType() { return this->m_targetType; };

	private:
		RenderTarget( GLuint, GLsizei, GLsizei, RenderTargetType, TextureType );
		~RenderTarget();

		GLsizei m_width;
		GLsizei m_height;

		RenderTargetType m_targetType;

		static std::vector<RenderTarget*> m_allTargets;

		static RenderTarget* findShadowMapInPool( RenderTargetType, GLsizei, GLsizei );
};

#endif /* _RENDERTARGET_H_ */