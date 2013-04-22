/*
 * FrameBufferObject.h
 *
 *  Created on: 17.03.2013
 *      Author: Jonathan Thaler
 */

#ifndef FRAMEBUFFEROBJECT_H_
#define FRAMEBUFFEROBJECT_H_

#include "RenderTarget.h"

#include <GL/glew.h>

#include <vector>

class FrameBufferObject
{
	public:
		static FrameBufferObject* create();
		static bool destroy( FrameBufferObject* );

		bool attachTarget( RenderTarget* );
		bool attachTargetTemp( RenderTarget* );

		bool drawAllBuffers();
		bool drawBuffer( unsigned int );
		bool drawBuffers( std::vector<unsigned int> );
		bool drawNone();

		bool bind();
		bool unbind();

		bool bindAllTargets();
		bool bindTargets( std::vector<unsigned int> );
		bool bindTarget( unsigned int );

		bool clearAll();

		bool checkStatus();

		const std::vector<RenderTarget*>& getAttachedTargets() { return this->m_attachedTargets; };
		RenderTarget* getAttachedDepthTarget() { return this->m_depthTarget; };

	private:
		FrameBufferObject( GLuint );
		~FrameBufferObject();

		GLuint m_id;

		GLuint m_depthBuffer;	

		RenderTarget* m_depthTarget;

		std::vector<GLenum> m_colorBufferTargets;
		std::vector<GLuint> m_colorBuffers;
		
		std::vector<RenderTarget*> m_attachedTargets;
};

#endif /* FRAMEBUFFEROBJECT_H_ */