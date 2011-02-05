/*
 * DRRenderer.cpp
 *
 *  Created on: Jan 16, 2011
 *      Author: jonathan
 */

#include <GL/glew.h>
#include "SDL/SDL.h"
#include <glm/glm.hpp>

#include "DRRenderer.h"
#include "../Material/UniformBlock.h"

#include <iostream>
#include <algorithm>

using namespace std;

DRRenderer::DRRenderer(Camera& camera, std::string& skyBoxFolder)
	: Renderer(camera, skyBoxFolder)
{
	this->m_drFB = 0;
	memset( this->m_mrt, sizeof( this->m_mrt), 0 );

	this->m_progGeomStage = 0;
	this->m_vertGeomStage = 0;
	this->m_fragGeomStage = 0;

	this->m_progLightingStage = 0;
	this->m_vertLightingStage = 0;
	this->m_fragLightingStage = 0;

	this->m_progShadowMapping = 0;
	this->m_vertShadowMapping = 0;
	this->m_fragShadowMapping = 0;

	this->m_shadowMap = 0;
	this->m_shadowMappingFB = 0;

	this->m_transformBlock = 0;

	this->m_light = 0;
}

DRRenderer::~DRRenderer()
{
}

bool
DRRenderer::renderFrame( std::list<Instance*>& instances )
{
	GLint status;

	// calculate the model-view-projection matrix
	this->m_modelViewProjection = this->camera.modelView.data;
	this->m_modelViewProjection.multiply( this->camera.projection  );

	// update the transform-uniforms block with the new mvp matrix
	if ( false == this->m_transformBlock->updateData( this->m_modelViewProjection.data, 0, 64) )
		return false;

	glUseProgram( 0 );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "ERROR in DRRenderer::renderFrame: glUseProgram( 0 ) failed with " << gluErrorString( status ) << endl;
		return false;
	}

	glLoadMatrixf( this->camera.modelView.data );

	//Rendering offscreen
	glBindFramebufferEXT( GL_FRAMEBUFFER_EXT, this->m_shadowMappingFB );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "ERROR in DRRenderer::renderFrame: glBindFramebufferEXT failed with " << gluErrorString( status ) << endl;
		return false;
	}

	glClear( GL_DEPTH_BUFFER_BIT );
	glColorMask( GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE );

	if ( false == this->renderInstances( instances ) )
		return false;

	glBindFramebufferEXT( GL_FRAMEBUFFER_EXT,0 );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "ERROR in DRRenderer::renderFrame: glBindFramebufferEXT( 0 ) failed with " << gluErrorString( status ) << endl;
		return false;
	}

	glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

	// clear window
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	/*
	if ( false == this->m_progGeomStage->use() )
	{
		cout << "ERROR in DRRenderer::renderFrame: using shadow mapping program failed - exit" << endl;
		return false;
	}

	if ( false == this->m_transformBlock->bind( 0 ) )
	{
		cout << "ERROR in DRRenderer::renderFrame: binding transform uniform-block failed - exit" << endl;
		return false;
	}

	//GLenum buffers[MRT_COUNT];

	// drawing the cross in the origin
	glColor4f(1, 0, 0, 0);
	glBegin(GL_LINES);
		glVertex3f(100, 0, 0);
		glVertex3f(-100, 0, 0);
	glEnd();

	glColor4f(0, 1, 0, 0);
	glBegin(GL_LINES);
		glVertex3f(0, 100, 0);
		glVertex3f(0, -100, 0);
	glEnd();

	glColor4f(0, 0, 1, 0);
	glBegin(GL_LINES);
		glVertex3f(0, 0, 100);
		glVertex3f(0, 0, -100);
	glEnd();

	//  start geometry pass
	// glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, this->m_drFB);

	// clear fbo
	//glClear(GL_COLOR_BUFFER_BIT);

	// activate drawing to targets
	//for ( int i = 0; i < MRT_COUNT; i++)
	//	buffers[ i ] = GL_COLOR_ATTACHMENT0_EXT + i;
	//glDrawBuffers(MRT_COUNT, buffers);


	// draw all geometry
	if ( false == this->renderInstances( instances ) )
		return false;
*/
	this->showShadowMap();

	/*
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

	// start lighting stage
	// bind rendertargets as textures
	for ( int i = 0; i < MRT_COUNT; i++ )
	{
		glActiveTexture( GL_TEXTURE0 + i );
		glBindTexture( GL_TEXTURE_2D, this->m_mrt[ i ]);
	}

	// activate lighting-stage shader
	//this->m_lightStageProg->activate();
*/

	// finish lighting stage
	//this->m_lightStageProg->deactivate();

	// swap buffers
	SDL_GL_SwapBuffers();

	this->frame++;

	return true;
}

bool
DRRenderer::initialize()
{
	cout << "Initializing Deferred Renderer..." << endl;

	//if ( false == this->initFBO() )
	//	return false;

	if ( false == this->initGeomStage() )
		return false;

	if ( false == this->initLightingStage() )
		return false;

	if ( false == this->initShadowMapping() )
		return false;

	if ( false == this->initUniformBlocks() )
		return false;

	cout << "Initializing Deferred Renderer finished" << endl;

	return true;
}

bool
DRRenderer::shutdown()
{
	cout << "Shutting down Deferred Renderer..." << endl;

	// cleaning up uniform blocks
	if ( this->m_transformBlock )
		delete this->m_transformBlock;

	if ( this->m_shadowMap )
	{
		glDeleteTextures( 1, &this->m_shadowMap );
	}

	if ( this->m_shadowMappingFB )
	{
		glDeleteFramebuffers( 1, &this->m_shadowMappingFB );
	}

	// cleaning up shadow mapping
	if ( this->m_progShadowMapping )
	{
		if ( this->m_vertShadowMapping )
		{
			this->m_progShadowMapping->detachShader( this->m_vertShadowMapping );

			delete this->m_vertShadowMapping;
			this->m_vertShadowMapping = NULL;
		}

		if ( this->m_fragShadowMapping )
		{
			this->m_progShadowMapping->detachShader( this->m_fragShadowMapping );

			delete this->m_fragShadowMapping;
			this->m_fragShadowMapping = NULL;
		}

		delete this->m_progShadowMapping;
		this->m_progShadowMapping = NULL;
	}

	// cleaning up lighting stage
	if ( this->m_progLightingStage )
	{
		if ( this->m_vertLightingStage )
		{
			this->m_progLightingStage->detachShader( this->m_vertLightingStage );

			delete this->m_vertLightingStage;
			this->m_vertLightingStage = NULL;
		}

		if ( this->m_fragLightingStage )
		{
			this->m_progLightingStage->detachShader( this->m_fragLightingStage );

			delete this->m_fragLightingStage;
			this->m_fragLightingStage = NULL;
		}

		delete this->m_progLightingStage;
		this->m_progLightingStage = NULL;
	}

	// cleaning up geometry-stage
	if ( this->m_progGeomStage )
	{
		if ( this->m_vertLightingStage )
		{
			this->m_progGeomStage->detachShader( this->m_vertLightingStage );

			delete this->m_vertLightingStage;
			this->m_vertLightingStage = NULL;
		}

		if ( this->m_fragGeomStage )
		{
			this->m_progGeomStage->detachShader( this->m_fragGeomStage );

			delete this->m_fragGeomStage;
			this->m_fragGeomStage = NULL;
		}

		delete this->m_progGeomStage;
		this->m_progGeomStage = NULL;
	}

	// cleaning up framebuffer
	if ( this->m_drFB )
	{
		glDeleteFramebuffers( 1, &this->m_drFB );
	}

	// cleaning up mrts
	for ( int i = 0; i < MRT_COUNT; i++ )
	{
		if ( this->m_mrt[ i ] )
		{
			glDeleteTextures( 1, &this->m_mrt[ i ] );
			this->m_mrt[ i ] = 0;
		}
	}

	cout << "Shutting down Deferred Renderer finished" << endl;

	return true;
}

bool
DRRenderer::initFBO()
{
	GLenum status;

	glGenFramebuffersEXT(1, &this->m_drFB);
	if ( GL_NO_ERROR != ( status = glGetError() )  )
	{
		cout << "ERROR in DRRenderer::initFBO: glGenFramebuffersEXT failed with " << gluErrorString( status ) << " - exit" << endl;
		return false;
	}

	for ( int i = 0; i < MRT_COUNT; i++ )
	{
		glGenTextures( 1, &this->m_mrt[ i ] );
		if ( GL_NO_ERROR != ( status = glGetError() )  )
		{
			cout << "ERROR in DRRenderer::initFBO: glGenTextures failed with " << gluErrorString( status ) << " - exit" << endl;
			return false;
		}

		glBindFramebufferEXT( GL_FRAMEBUFFER_EXT, this->m_drFB );
		if ( GL_NO_ERROR != ( status = glGetError() )  )
		{
			cout << "ERROR in DRRenderer::initFBO: glBindFramebufferEXT failed with " << gluErrorString( status ) << " - exit" << endl;
			return false;
		}
		glBindTexture( GL_TEXTURE_RECTANGLE_ARB, this->m_mrt[i] );
		if ( GL_NO_ERROR != ( status = glGetError() )  )
		{
			cout << "ERROR in DRRenderer::initFBO: glBindTexture failed with " << gluErrorString( status ) << " - exit" << endl;
			return false;
		}
		glTexImage2D( GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGBA8, this->camera.getWidth(), this->camera.getHeight(), 0, GL_RGBA, GL_UNSIGNED_BYTE, 0 );
		if ( GL_NO_ERROR != ( status = glGetError() )  )
		{
			cout << "ERROR in DRRenderer::initFBO: glTexImage2D failed with " << gluErrorString( status ) << " - exit" << endl;
			return false;
		}

		glClearColor(0, 0, 0, 0);
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT + i, GL_TEXTURE_RECTANGLE_ARB, this->m_mrt[i], 0);
		CHECK_FRAMEBUFFER_STATUS( status );
		if ( GL_FRAMEBUFFER_COMPLETE_EXT != status )
		{
			cout << "ERROR in DRRenderer::initFBO: framebuffer error: " << gluErrorString( status ) << " - exit" << endl;
			return false;
		}

		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
	}

	return true;
}

bool
DRRenderer::initGeomStage()
{
	cout << "Initializing Deferred Rendering Geometry-Stage..." << endl;

	this->m_progGeomStage = Program::createProgram( );
	if ( 0 == this->m_progGeomStage )
	{
		cout << "ERROR in DRRenderer::initGeomStage: coulnd't create program - exit" << endl;
		return false;
	}

	this->m_vertLightingStage = Shader::createShader( Shader::VERTEX_SHADER, "media/graphics/dr/stages/geom/geomVert.glsl" );
	if ( 0 == this->m_vertLightingStage )
	{
		cout << "ERROR in DRRenderer::initGeomStage: coulnd't create vertex-shader - exit" << endl;
		return false;
	}

	this->m_fragGeomStage = Shader::createShader( Shader::FRAGMENT_SHADER, "media/graphics/dr/stages/geom/geomFrag.glsl" );
	if ( 0 == this->m_fragGeomStage )
	{
		cout << "ERROR in DRRenderer::initGeomStage: coulnd't create fragment-shader - exit" << endl;
		return false;
	}

	if ( false == this->m_vertLightingStage->compile() )
	{
		cout << "ERROR in DRRenderer::initGeomStage: vertex shader compilation failed - exit" << endl;
		return false;
	}

	if ( false == this->m_fragGeomStage->compile() )
	{
		cout << "ERROR in DRRenderer::initGeomStage: fragment shader compilation failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progGeomStage->attachShader( this->m_vertLightingStage ) )
	{
		cout << "ERROR in DRRenderer::initGeomStage: attaching vertex shader to program failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progGeomStage->attachShader( this->m_fragGeomStage ) )
	{
		cout << "ERROR in DRRenderer::initGeomStage: attaching fragment shader to program failed - exit" << endl;
		return false;
	}

	// setting frag-data location is done bevore linking
	/*
	this->m_progGeomStage->bindFragDataLocation( 0, "out_diffuse" );
	this->m_progGeomStage->bindFragDataLocation( 1, "out_normal" );
	this->m_progGeomStage->bindFragDataLocation( 2, "out_depth" );
	this->m_progGeomStage->bindFragDataLocation( 3, "out_generic" );
	*/

	if ( false == this->m_progGeomStage->bindAttribLocation( 0, "in_vertPos" ) )
	{
		cout << "ERROR in DRRenderer::initGeomStage: binding attribute location to program failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progGeomStage->bindAttribLocation( 1, "in_vertNorm" ) )
	{
		cout << "ERROR in DRRenderer::initGeomStage: binding attribute location to program failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progGeomStage->link() )
	{
		cout << "ERROR in DRRenderer::initGeomStage: linking program failed - exit" << endl;
		return false;
	}

	cout << "Initializing Deferred Rendering Geometry-Stage finished" << endl;

	return true;
}

bool
DRRenderer::initLightingStage()
{
	cout << "Initializing Deferred Rendering Lighting-Stage..." << endl;

	this->m_light = new Light();

	this->m_light->viewingTransf.setPosition( 0, 500, 0 );

	cout << "Initializing Deferred Rendering Lighting-Stage finished" << endl;

	return true;
}

bool
DRRenderer::initShadowMapping()
{
	cout << "Initializing Deferred Rendering Shadow-Mapping..." << endl;

	GLenum status;

	// Try to use a texture depth component
	glGenTextures( 1, &this->m_shadowMap );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: glGenTextures failed with " << gluErrorString( status ) << " - exit" << endl;
		return false;
	}

	glBindTexture( GL_TEXTURE_2D, this->m_shadowMap );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: glBindTexture failed with " << gluErrorString( status ) << " - exit" << endl;
		return false;
	}

	// GL_LINEAR does not make sense for depth texture. However, next tutorial shows usage of GL_LINEAR and PCF
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );

	// Remove artifact on the edges of the shadowmap
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP );
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP );

	// No need to force GL_DEPTH_COMPONENT24, drivers usually give you the max precision if available
	glTexImage2D( GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, SHADOW_MAP_WIDTH, SHADOW_MAP_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, 0 );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "ERROR in DRRenderer::glTexImage2D: glGenTextures failed with " << gluErrorString( status ) << " - exit" << endl;
		return false;
	}

	glBindTexture( GL_TEXTURE_2D, 0 );

	// create a framebuffer object
	glGenFramebuffersEXT( 1, &this->m_shadowMappingFB );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: glGenFramebuffersEXT failed with " << gluErrorString( status ) << " - exit" << endl;
		return false;
	}

	glBindFramebufferEXT( GL_FRAMEBUFFER_EXT, this->m_shadowMappingFB );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: glBindFramebufferEXT failed with " << gluErrorString( status ) << " - exit" << endl;
		return false;
	}

	// Instruct openGL that we won't bind a color texture with the currently binded FBO
	glDrawBuffer( GL_NONE );
	glReadBuffer( GL_NONE );

	// attach the texture to FBO depth attachment point
	glFramebufferTexture2DEXT( GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,GL_TEXTURE_2D, this->m_shadowMap, 0 );
	CHECK_FRAMEBUFFER_STATUS( status );
	if ( GL_FRAMEBUFFER_COMPLETE_EXT != status )
	{
		cout << "ERROR in DRRenderer::initFBO: glFramebufferTexture2DEXT error: " << gluErrorString( status ) << " - exit" << endl;
		return false;
	}

	// switch back to window-system-provided framebuffer
	glBindFramebufferEXT( GL_FRAMEBUFFER_EXT, 0 );

	this->m_progShadowMapping = Program::createProgram( );
	if ( 0 == this->m_progShadowMapping )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: coulnd't create program - exit" << endl;
		return false;
	}

	this->m_vertShadowMapping = Shader::createShader( Shader::VERTEX_SHADER, "media/graphics/dr/stages/geom/shadow/shadowVert.glsl" );
	if ( 0 == this->m_vertShadowMapping )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: coulnd't create vertex shader - exit" << endl;
		return false;
	}

	this->m_fragShadowMapping = Shader::createShader( Shader::FRAGMENT_SHADER, "media/graphics/dr/stages/geom/shadow/shadowFrag.glsl" );
	if ( 0 == this->m_fragShadowMapping )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: coulnd't create fragment shader - exit" << endl;
		return false;
	}

	if ( false == this->m_vertShadowMapping->compile() )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: vertex shader compilation failed - exit" << endl;
		return false;
	}

	if ( false == this->m_fragShadowMapping->compile() )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: fragment shader compilation failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progShadowMapping->attachShader( this->m_vertShadowMapping ) )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: attaching vertex shader to program failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progShadowMapping->attachShader( this->m_fragShadowMapping ) )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: attaching fragment shader to program failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progShadowMapping->link() )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: linking program failed - exit" << endl;
		return false;
	}

	cout << "Initializing Deferred Rendering Shadow-Mapping finished" << endl;

	return true;
}

bool
DRRenderer::initUniformBlocks()
{
	this->m_transformBlock = UniformBlock::createBlock( "transform" );
	if ( 0 == this->m_transformBlock )
	{
		cout << "ERROR in DRRenderer::initGeomStage: creating uniform block failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progGeomStage->bindUniformBlock( this->m_transformBlock ) )
	{
		cout << "ERROR in DRRenderer::initGeomStage: failed binding uniform block - exit" << endl;
		return false;
	}

	return true;
}

bool
DRRenderer::renderInstances( std::list<Instance*>& instances )
{
	list<Instance*>::iterator iter = instances.begin();
	while ( iter != instances.end() )
	{
		Instance* instance = *iter++;

		Matrix transf( instance->transform.matrix.data );
		transf.multiply( this->m_modelViewProjection );

		if ( false == this->renderGeom( transf, instance->geom ) )
			return false;
	}

	return true;
}

bool
DRRenderer::renderGeom( Matrix& transf, GeomType* geom )
{
	if ( geom->children.size() )
	{
		for ( unsigned int i = 0; i < geom->children.size(); i++ )
		{
			return this->renderGeom( transf, geom->children[ i ] );
		}
	}
	else
	{
		Matrix mat( geom->model_transf );
		mat.multiply( transf );

		if ( false == this->m_transformBlock->updateData( mat.data, 0, 64) )
			return false;

		return geom->render();
	}

	return true;
}

bool
DRRenderer::showShadowMap()
{
	GLint status;

	glUseProgram( 0 );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "ERROR in DRRenderer::renderFrame: glUseProgram( 0 ) failed with " << gluErrorString( status ) << endl;
		return false;
	}

	// set up orthogonal projection to render quad
	this->camera.setupOrtho();

	glEnable(GL_TEXTURE_2D);
	glActiveTexture( GL_TEXTURE0 );
	glBindTexture( GL_TEXTURE_2D, this->m_shadowMap );

	// render quad
	glBegin( GL_QUADS );
		glTexCoord2f( 0.0f, 0.0f ); glVertex2f( 0, 0 );
		glTexCoord2f( 0.0f, 1.0f ); glVertex2f( 0, this->camera.getHeight() );
		glTexCoord2f( 1.0f, 1.0f ); glVertex2f( this->camera.getWidth(), this->camera.getHeight() );
		glTexCoord2f( 1.0f, 0.0f ); glVertex2f( this->camera.getWidth(), 0 );
	glEnd();

	glBindTexture( GL_TEXTURE_2D, 0 );
	glDisable(GL_TEXTURE_2D);

	// switch back to perspective projection
	this->camera.setupPerspective();

	return true;
}
