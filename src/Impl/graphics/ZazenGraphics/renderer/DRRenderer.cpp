/*
 * DRRenderer.cpp
 *
 *  Created on: Jan 16, 2011
 *      Author: jonathan
 */

#include <GL/glew.h>

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "DRRenderer.h"

#include "../Geometry/GeomSkyBox.h"
#include "../Material/UniformBlock.h"

#include <iostream>
#include <algorithm>

using namespace std;

DRRenderer::DRRenderer()
	: Renderer( )
{
	this->m_fbo = 0;

	this->m_progGeomStage = 0;
	this->m_vertGeomStage = 0;
	this->m_fragGeomStage = 0;

	this->m_progLightingStage = 0;
	this->m_vertLightingStage = 0;
	this->m_fragLightingStage = 0;

	this->m_progShadowMapping = 0;
	this->m_vertShadowMapping = 0;
	this->m_fragShadowMapping = 0;

	this->m_shadowMappingFB = 0;

	this->m_transformsBlock = 0;
	this->m_cameraBlock = 0;
	this->m_lightBlock = 0;
	this->m_materialBlock = 0;

	this->m_unitCubeMatrix = glm::mat4(
		0.5, 0.0, 0.0, 0.0, 
		0.0, 0.5, 0.0, 0.0,
		0.0, 0.0, 0.5, 0.0,
		0.5, 0.5, 0.5, 1.0
	);

	this->m_displayMRT = false;
}

DRRenderer::~DRRenderer()
{
}

bool
DRRenderer::renderFrame( std::list<Instance*>& instances, std::list<Light*>& lights )
{
	if ( false == this->renderShadowMap( instances, lights ) )
	{
		return false;
	}
	
	if ( false == this->m_fbo->bind() )
	{
		return false;
	}

	// IMPORTANT: need to set the viewport for each FBO
	glViewport( 0, 0, this->m_camera->getWidth(), this->m_camera->getHeight() );

	if ( false == this->m_fbo->clearAll() )
	{
		return false;
	}

	if ( false == this->m_fbo->unbind() )
	{
		return false;
	}

	if ( false == this->renderSkyBox() )
	{
		return false;
	}

	if ( false == this->renderGeometryStage( instances, lights ) )
	{
		return false;
	}

	if ( false == this->renderLightingStage( instances, lights ) )
	{
		return false;
	}

	if ( this->m_displayMRT )
	{
		if ( false == this->showTexture( this->m_fbo->getAttachedTargets()[ 0 ]->getId(), 0 ) )
		{
			return false;
		}
		if ( false == this->showTexture( this->m_fbo->getAttachedTargets()[ 1 ]->getId(), 1 ) )
		{
			return false;
		}
		if ( false == this->showTexture( this->m_fbo->getAttachedTargets()[ 2 ]->getId(), 2 ) )
		{
			return false;
		}
		if ( false == this->showTexture( this->m_fbo->getAttachedDepthTarget()->getId(), 3 ) )
		{
			return false;
		}
	}
	

	this->frame++;

	return true;
}

bool
DRRenderer::toggleDisplay()
{
	this->m_displayMRT = !this->m_displayMRT;

	return true;
}

bool
DRRenderer::initialize( const boost::filesystem::path& pipelinePath )
{
	cout << "Initializing Deferred Renderer..." << endl;

	glShadeModel( GL_SMOOTH );								// Enable Smooth Shading
	glClearColor( 0.0f, 0.0f, 0.0f, 1.0f );					// Black Background
	glClearDepth( 1.0f );									// Depth Buffer Setup

	glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );	// Really Nice Perspective Calculations

	glEnable( GL_DEPTH_TEST );								// Enables Depth Testing
	glDepthFunc( GL_LESS );								// The Type Of Depth Testing To Do
	glDepthMask( GL_TRUE );

	// Cull triangles which normal is not towards the camera
	glEnable(GL_CULL_FACE);

	glEnable( GL_TEXTURE_2D );

	if ( false == this->initFBO() )
	{
		return false;
	}

	if ( false == this->initGeomStage( pipelinePath ) )
	{
		return false;
	}

	if ( false == this->initLightingStage( pipelinePath ) )
	{
		return false;
	}

	if ( false == this->initShadowMapping( pipelinePath ) )
	{
		return false;
	}

	if ( false == this->initUniformBlocks() )
	{
		return false;
	}

	cout << "Initializing Deferred Renderer finished" << endl;

	return true;
}

bool
DRRenderer::shutdown()
{
	cout << "Shutting down Deferred Renderer..." << endl;

	Program::unuse();

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
		if ( this->m_vertGeomStage )
		{
			this->m_progGeomStage->detachShader( this->m_vertGeomStage );

			delete this->m_vertGeomStage;
			this->m_vertGeomStage = NULL;
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

	// cleaning up uniform blocks
	if ( this->m_transformsBlock )
	{
		delete this->m_transformsBlock;
		this->m_transformsBlock = NULL;
	}

	if ( this->m_cameraBlock )
	{
		delete this->m_cameraBlock;
		this->m_cameraBlock = NULL;
	}

	if ( this->m_lightBlock )
	{
		delete this->m_lightBlock;
		this->m_lightBlock = NULL;
	}

	if ( this->m_materialBlock )
	{
		delete this->m_materialBlock;
		this->m_materialBlock = NULL;
	}

	FrameBufferObject::destroy( this->m_shadowMappingFB );

	// cleaning up framebuffer
	FrameBufferObject::destroy( this->m_fbo );
	
	GeomSkyBox::shutdown();

	cout << "Shutting down Deferred Renderer finished" << endl;

	return true;
}

bool
DRRenderer::initFBO()
{
	this->m_fbo = FrameBufferObject::create();
	if ( NULL == this->m_fbo )
	{
		return false;
	}

	if ( false == this->m_fbo->bind() )
	{
		return false;
	}
	
	// index 0: diffuse color
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR ) )		
	{
		return false;
	}

	// index 1: normals
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR ) )		
	{
		return false;
	}

	// index 2: position data
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR ) )		
	{
		return false;
	}

	// index 3: generic attributes
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR ) )		
	{
		return false;
	}

	// index 4: depth-buffer
	if ( false == this->createMrtBuffer( RenderTarget::RT_DEPTH ) )		
	{
		return false;
	}

	if ( false == this->m_fbo->checkStatus() )
	{
		return false;
	}

	if ( false == this->m_fbo->unbind() )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::initGeomStage( const boost::filesystem::path& pipelinePath )
{
	cout << "Initializing Deferred Rendering Geometry-Stage..." << endl;

	this->m_progGeomStage = Program::createProgram( "GeometryStageProgramm" );
	if ( 0 == this->m_progGeomStage )
	{
		cout << "ERROR in DRRenderer::initGeomStage: coulnd't create program - exit" << endl;
		return false;
	}

	this->m_vertGeomStage = Shader::createShader( Shader::VERTEX_SHADER, pipelinePath.generic_string() + "/dr/stages/geom/geomVert.glsl" );
	if ( 0 == this->m_vertGeomStage )
	{
		cout << "ERROR in DRRenderer::initGeomStage: coulnd't create vertex-shader - exit" << endl;
		return false;
	}

	this->m_fragGeomStage = Shader::createShader( Shader::FRAGMENT_SHADER, pipelinePath.generic_string() + "/dr/stages/geom/geomFrag.glsl" );
	if ( 0 == this->m_fragGeomStage )
	{
		cout << "ERROR in DRRenderer::initGeomStage: coulnd't create fragment-shader - exit" << endl;
		return false;
	}

	if ( false == this->m_vertGeomStage->compile() )
	{
		cout << "ERROR in DRRenderer::initGeomStage: vertex shader compilation failed - exit" << endl;
		return false;
	}

	if ( false == this->m_fragGeomStage->compile() )
	{
		cout << "ERROR in DRRenderer::initGeomStage: fragment shader compilation failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progGeomStage->attachShader( this->m_vertGeomStage ) )
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
	if ( false == this->m_progGeomStage->bindFragDataLocation( 0, "out_diffuse" ) )
	{
		cout << "ERROR in DRRenderer::initGeomStage: binding fragment-data location failed - exit" << endl;
		return false;
	}
	if ( false == this->m_progGeomStage->bindFragDataLocation( 1, "out_normal" ) )
	{
		cout << "ERROR in DRRenderer::initGeomStage: binding fragment-data location failed - exit" << endl;
		return false;
	}
	if ( false == this->m_progGeomStage->bindFragDataLocation( 2, "out_generic1" ) )
	{
		cout << "ERROR in DRRenderer::initGeomStage: binding fragment-data location failed - exit" << endl;
		return false;
	}
	if ( false == this->m_progGeomStage->bindFragDataLocation( 3, "out_generic2" ) )
	{
		cout << "ERROR in DRRenderer::initGeomStage: binding fragment-data location failed - exit" << endl;
		return false;
	}

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
	if ( false == this->m_progGeomStage->bindAttribLocation( 2, "in_texCoord" ) )
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
DRRenderer::initLightingStage( const boost::filesystem::path& pipelinePath )
{
	cout << "Initializing Deferred Rendering Lighting-Stage..." << endl;

	this->m_progLightingStage = Program::createProgram( "LightingStageProgramm" );
	if ( 0 == this->m_progLightingStage )
	{
		cout << "ERROR in DRRenderer::initLightingStage: coulnd't create program - exit" << endl;
		return false;
	}

	this->m_vertLightingStage = Shader::createShader( Shader::VERTEX_SHADER, pipelinePath.generic_string() + "/dr/stages/lighting/lightVert.glsl" );
	if ( 0 == this->m_vertLightingStage )
	{
		cout << "ERROR in DRRenderer::initLightingStage: coulnd't create vertex-shader - exit" << endl;
		return false;
	}

	this->m_fragLightingStage = Shader::createShader( Shader::FRAGMENT_SHADER, pipelinePath.generic_string() + "/dr/stages/lighting/lightFrag.glsl" );
	if ( 0 == this->m_fragGeomStage )
	{
		cout << "ERROR in DRRenderer::initLightingStage: coulnd't create fragment-shader - exit" << endl;
		return false;
	}

	if ( false == this->m_vertLightingStage->compile() )
	{
		cout << "ERROR in DRRenderer::initLightingStage: vertex shader compilation failed - exit" << endl;
		return false;
	}

	if ( false == this->m_fragLightingStage->compile() )
	{
		cout << "ERROR in DRRenderer::initLightingStage: fragment shader compilation failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progLightingStage->attachShader( this->m_vertLightingStage ) )
	{
		cout << "ERROR in DRRenderer::initLightingStage: attaching vertex shader to program failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progLightingStage->attachShader( this->m_fragLightingStage ) )
	{
		cout << "ERROR in DRRenderer::initLightingStage: attaching fragment shader to program failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progLightingStage->bindAttribLocation( 0, "in_vertPos" ) )
	{
		cout << "ERROR in DRRenderer::initLightingStage: binding attribute location to program failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progLightingStage->link() )
	{
		cout << "ERROR in DRRenderer::initLightingStage: linking program failed - exit" << endl;
		return false;
	}

	cout << "Initializing Deferred Rendering Lighting-Stage finished" << endl;

	return true;
}

bool
DRRenderer::initShadowMapping( const boost::filesystem::path& pipelinePath )
{
	cout << "Initializing Deferred Rendering Shadow-Mapping..." << endl;

	this->m_shadowMappingFB = FrameBufferObject::create();
	if ( NULL == this->m_shadowMappingFB )
	{
		return false;
	}

	this->m_progShadowMapping = Program::createProgram( "ShadowingStageProgramm" );
	if ( 0 == this->m_progShadowMapping )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: coulnd't create program - exit" << endl;
		return false;
	}

	this->m_vertShadowMapping = Shader::createShader( Shader::VERTEX_SHADER, pipelinePath.generic_string() + "/dr/stages/shadowing/shadowVert.glsl" );
	if ( 0 == this->m_vertShadowMapping )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: coulnd't create vertex shader - exit" << endl;
		return false;
	}

	this->m_fragShadowMapping = Shader::createShader( Shader::FRAGMENT_SHADER, pipelinePath.generic_string() + "/dr/stages/shadowing/shadowFrag.glsl" );
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

	// setting frag-data location is done bevore linking
	if ( false == this->m_progShadowMapping->bindFragDataLocation( 0, "fragmentdepth" ) )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: binding fragment-data location failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progShadowMapping->bindAttribLocation( 0, "in_vertPos" ) )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: binding attribute location to program failed - exit" << endl;
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
	this->m_transformsBlock = UniformBlock::createBlock( "transforms" );
	if ( 0 == this->m_transformsBlock )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: creating uniform block failed - exit" << endl;
		return false;
	}

	this->m_cameraBlock = UniformBlock::createBlock( "camera" );
	if ( 0 == this->m_cameraBlock )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: creating uniform block failed - exit" << endl;
		return false;
	}

	this->m_lightBlock = UniformBlock::createBlock( "light" );
	if ( 0 == this->m_lightBlock )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: creating uniform block failed - exit" << endl;
		return false;
	}

	this->m_materialBlock = UniformBlock::createBlock( "material" );
	if ( 0 == this->m_materialBlock )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: creating uniform block failed - exit" << endl;
		return false;
	}

	// bind transformation-data to all programs
	if ( false == this->m_progShadowMapping->bindUniformBlock( this->m_transformsBlock ) )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: failed binding uniform block - exit" << endl;
		return false;
	}
	if ( false == this->m_progGeomStage->bindUniformBlock( this->m_transformsBlock ) )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: failed binding uniform block - exit" << endl;
		return false;
	}
	if ( false == this->m_progLightingStage->bindUniformBlock( this->m_transformsBlock ) )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: failed binding uniform block - exit" << endl;
		return false;
	}

	// lighting-data & camera-data just to lighting stage program
	if ( false == this->m_progLightingStage->bindUniformBlock( this->m_lightBlock ) )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: failed binding uniform block - exit" << endl;
		return false;
	}
	if ( false == this->m_progLightingStage->bindUniformBlock( this->m_cameraBlock ) )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: failed binding uniform block - exit" << endl;
		return false;
	}

	// material-data just go to geometry-stage program
	if ( false == this->m_progGeomStage->bindUniformBlock( this->m_materialBlock ) )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: failed binding uniform block - exit" << endl;
		return false;
	}

	/* IMPORTANT: found this in forums: 
		On ATI hardware, you have to call

		glGetUniformBlockIndex( program, blockName )
		glUniformBlockBinding( program, blockIndex, slot )

		BEFORE

		glBindBufferBase( GL_UNIFORM_BUFFER, slot, UBO )

		On NVIDIA hardware, the other way around is fine.

		THIS IMPLIES: call bind() method on UniformBlock AFTER calling bindUniformBlock() on programm
	*/
	// bind uniform blocks
	if ( false == this->m_transformsBlock->bind() )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: binding transform uniform-block failed - exit" << endl;
		return false;
	}
	if ( false == this->m_cameraBlock->bind() )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: binding transform uniform-block failed - exit" << endl;
		return false;
	}
	if ( false == this->m_lightBlock->bind() )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: binding transform uniform-block failed - exit" << endl;
		return false;
	}
	if ( false == this->m_materialBlock->bind() )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: binding transform uniform-block failed - exit" << endl;
		return false;
	}

	return true;
}

bool
DRRenderer::createMrtBuffer( RenderTarget::RenderTargetType targetType )
{
	RenderTarget* renderTarget = RenderTarget::create( ( GLsizei ) this->m_camera->getWidth(), ( GLsizei ) this->m_camera->getHeight(), targetType );
	if ( NULL == renderTarget )
	{
		return false;
	}

	if ( false == this->m_fbo->attachTarget( renderTarget ) )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::renderShadowMap( std::list<Instance*>& instances, std::list<Light*>& lights )
{
	// Rendering offscreen
	if ( false == this->m_shadowMappingFB->bind() )
	{
		return false;
	}

	// don't check yet because cannot be complete yet
	this->m_shadowMappingFB->drawNone();

	// render the depth-map for each light
	std::list<Light*>::iterator iter = lights.begin();
	while ( iter != lights.end() )
	{
		Light* light = *iter++;

		if ( false == this->m_shadowMappingFB->attachTarget( light->getShadowMap() ) )
		{
			return false;
		}
	}

	// no all set up for checking completeness
	if ( false == this->m_shadowMappingFB->checkStatus() )
	{
		return false;
	}

	// back to window-system framebuffer
	if ( false == this->m_shadowMappingFB->unbind() )
	{
		return false;
	}

	// Rendering offscreen
	if ( false == this->m_shadowMappingFB->bind() )
	{
		return false;
	}

	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	if ( false == this->m_progShadowMapping->use() )
	{
		cout << "ERROR in DRRenderer::renderShadowMap: using program failed - exit" << endl;
		return false;
	}

	// render the depth-map for each light
	iter = lights.begin();
	while ( iter != lights.end() )
	{
		Light* light = *iter++;

		// IMPORTANT: need to set the viewport for each FBO
		glViewport( 0, 0, light->getWidth(), light->getHeight() );

		// render scene from view of camera - don't apply material, don't render transparency
		if ( false == this->renderInstances( light, instances, this->m_progShadowMapping, false, false ) )
		{
			return false;
		}
	}
	
	// back to window-system framebuffer
	if ( false == this->m_shadowMappingFB->unbind() )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::renderSkyBox()
{
	if ( false == GeomSkyBox::isPresent() )
	{
		return true;
	}

	if ( false == Program::unuse() )
	{
		cout << "ERROR in DRRenderer::renderSkyBox: failed deactivating program " << endl;
		return false;
	}

	if ( false == this->m_fbo->bind() )
	{
		return false;
	}

	if ( false == this->m_fbo->drawBuffer( 0 ) )
	{
		return false;
	}

	if ( false == this->m_fbo->checkStatus() )
	{
		return false;
	}

	GeomSkyBox::getRef().render();

	if ( false == this->m_fbo->unbind() )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::renderGeometryStage( std::list<Instance*>& instances, std::list<Light*>& lights )
{
	// activate geometry-stage program
	if ( false == this->m_progGeomStage->use() )
	{
		cout << "ERROR in DRRenderer::renderGeometryStage: using shadow mapping program failed - exit" << endl;
		return false;
	}

	if ( false == this->m_fbo->bind() )
	{
		return false;
	}

	if ( false == this->m_fbo->drawAllBuffers() )
	{
		return false;
	}

	if ( false == this->m_fbo->checkStatus() )
	{
		return false;
	}

	// draw all geometry - apply materials but don't render transparency
	if ( false == this->renderInstances( this->m_camera, instances, this->m_progGeomStage, true, false ) )
	{
		return false;
	}

	if ( false == this->m_fbo->unbind() )
	{
		return false;
	}

	return true;
}

// TODO break this method up into smaller submethods, its too long
bool
DRRenderer::renderLightingStage( std::list<Instance*>& instances, std::list<Light*>& lights )
{
	// activate lighting-stage shader
	if ( false == this->m_progLightingStage->use() )
	{
		cout << "ERROR in DRRenderer::renderLightingStage: using program failed - exit" << endl;
		return false;
	}
	
	glColorMask( GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE );
	glClearColor( 0.0, 0.0, 0.0, 1.0 );
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	this->m_fbo->bindAllTargets();

	glm::vec4 cameraRectangle;
	cameraRectangle[ 0 ] = ( float ) this->m_camera->getWidth();
	cameraRectangle[ 1 ] = ( float ) this->m_camera->getHeight();

	// upload world-orientation of camera ( its model-matrix )
	if ( false == this->m_cameraBlock->updateData( glm::value_ptr( this->m_camera->getMatrix() ), 0, 64 ) )
	{
		return false;
	}
	// upload view-matrix of camera (need to transform e.g. light-world position in EyeCoords/Viewspace)
	if ( false == this->m_cameraBlock->updateData( glm::value_ptr( this->m_camera->m_viewMatrix ), 64, 64 ) )
	{
		return false;
	}
	// upload camera-rectangel
	if ( false == this->m_cameraBlock->updateData( glm::value_ptr( cameraRectangle ), 128, 16 ) )
	{
		return false;
	}

	// TODO remove: can use layout in shader to bind
	// tell lighting program that diffusemap is bound to texture-unit 0
	this->m_progLightingStage->setUniformInt( "DiffuseMap", 0 );
	// tell lighting program that normalmap is bound to texture-unit 1
	this->m_progLightingStage->setUniformInt( "NormalMap", 1 );
	// tell lighting program that generic map is bound to texture-unit 2
	this->m_progLightingStage->setUniformInt( "GenericMap1", 2 );
	// tell lighting program that generic map is bound to texture-unit 3
	this->m_progLightingStage->setUniformInt( "GenericMap2", 3 );
	// tell lighting program that depth-map of scene is bound to texture-unit MRT_COUNT
	this->m_progLightingStage->setUniformInt( "DepthMap", MRT_COUNT );
	// tell program that the shadowmap of spot/directional-light will be available at texture unit MRT_COUNT + 1
	this->m_progLightingStage->setUniformInt( "ShadowMap", MRT_COUNT + 1 );

	// update projection-matrix because changed to orthogonal-projection
	glm::mat4 orthoMat = this->m_camera->createOrthoProj( false, true );
	if ( false == this->m_transformsBlock->updateData( glm::value_ptr( orthoMat ), 64, 64 ) )
	{
		return false;
	}

	glm::vec4 lightConfig;
	glm::mat4 lightSpaceUnit;

	// render the contribution of each light to the scene
	std::list<Light*>::iterator iter = lights.begin();
	while ( iter != lights.end() )
	{
		Light* light = *iter++;

		// TODO: need to set opengl to additiveley blend light-passes
		// alpha? color modulate?...

		// bind the shadowmap of the light to texture unit MRT_COUNT + 1
		if ( Light::SPOT == light->getType() || Light::DIRECTIONAL == light->getType() )
		{
			light->getShadowMap()->bind( MRT_COUNT + 1 );
		}
		else if ( Light::POINT == light->getType() )
		{
			// TODO: bind shadow cubemap
		}

		lightConfig[ 0 ] = ( float ) light->getType();
		lightConfig[ 1 ] = light->getFalloff();
		lightConfig[ 2 ] = light->isShadowCaster();

		// calculate the light-space projection matrix
		// multiplication with unit-cube is first because has to be carried out the last
		lightSpaceUnit = this->m_unitCubeMatrix * light->m_VPMatrix;

		if ( false == this->m_lightBlock->updateData( glm::value_ptr( lightConfig ), 0, 16 ) )
		{
			return false;
		}
		if ( false == this->m_lightBlock->updateData( glm::value_ptr( light->getColor() ), 16, 16 ) )
		{
			return false;
		}
		if ( false == this->m_lightBlock->updateData( glm::value_ptr( light->m_modelMatrix ), 32, 64 ) )
		{
			return false;
		}
		if ( false == this->m_lightBlock->updateData( glm::value_ptr( lightSpaceUnit ), 96, 64 ) )
		{
			return false;
		}

		// render quad
		glBegin( GL_QUADS );
			glVertex2f( 0, 0 );
			glVertex2f( 0, ( float ) this->m_camera->getHeight() );
			glVertex2f( ( float ) this->m_camera->getWidth(), ( float ) this->m_camera->getHeight() );
			glVertex2f( ( float ) this->m_camera->getWidth(), 0 );
		glEnd();

		// unbind shadow-map texture
		if ( Light::SPOT == light->getType() || Light::DIRECTIONAL == light->getType() )
		{
			light->getShadowMap()->unbind();
		}
		else if ( Light::POINT == light->getType() )
		{
			// TODO: unbind shadow cubemap
		}
	}

	this->m_fbo->unbindAllTargets();

	return true;
}

bool
DRRenderer::renderInstances( Viewer* viewer, list<Instance*>& instances, Program* currentProgramm, 
		bool applyMaterial, bool transparencyPass )
{
	if ( false == this->m_transformsBlock->updateData( glm::value_ptr( viewer->m_projectionMatrix ), 64, 64 ) )
	{
		return false;
	}

	list<Instance*>::iterator iter = instances.begin();
	while ( iter != instances.end() )
	{
		Instance* instance = *iter++;
		Material* activeMaterial = NULL;

		if ( instance->material && applyMaterial )
		{
			if ( transparencyPass )
			{
				if ( instance->material->getType() != Material::MATERIAL_TRANSPARENT )
				{
					continue;
				}
			}
			else
			{
				if ( instance->material->getType() == Material::MATERIAL_TRANSPARENT )
				{
					continue;
				}
			}

			activeMaterial = instance->material;
		}

		if ( activeMaterial )
		{
			if ( false == instance->material->activate( this->m_materialBlock, currentProgramm ) )
			{
				return false;
			}
		}

		if ( false == this->renderGeom( viewer, instance->geom, instance->getMatrix() ) )
		{
			return false;
		}

		if ( activeMaterial )
		{
			instance->material->deactivate();
		}
	}

	return true;
}

// NOTE OpenGL applies last matrix in multiplication as first transformation to object
bool
DRRenderer::renderGeom( Viewer* viewer, GeomType* geom, const glm::mat4& rootModelMatrix )
{
	glm::mat4 modelMatrix = rootModelMatrix * geom->m_modelMatrix;

	if ( geom->children.size() )
	{
		for ( unsigned int i = 0; i < geom->children.size(); i++ )
		{
			GeomType* child = geom->children[ i ];
			if ( false == this->renderGeom( viewer, geom->children[ i ], modelMatrix ) )
			{
				return false;
			}
		}
	}
	else
	{
		Viewer::CullResult cullResult = viewer->cullBB( geom->getBBMin(), geom->getBBMax() );
		if ( Viewer::OUTSIDE != cullResult )
		{
			// calculate modelView-Matrix
			glm::mat4 modelViewMatrix = viewer->m_viewMatrix * modelMatrix;
			// normal-vectors are transformed different than vertices
			// take the transpose of the inverse modelView or simply reset the translation vector in the modelview-matrix
			// in other words: only the rotations are applied to normals and they are guaranteed to leave
			// normalized normals at unit length. THIS METHOD ONLY WORKS WHEN NO NON UNIFORM SCALING IS APPLIED
			glm::mat4 normalModelViewMatrix = glm::transpose( glm::inverse( modelViewMatrix ) );

			if ( false == this->m_transformsBlock->updateData( glm::value_ptr( modelViewMatrix ), 0, 64 ) )
			{
				return false;
			}
			if ( false == this->m_transformsBlock->updateData( glm::value_ptr( normalModelViewMatrix ), 128, 64 ) )
			{
				return false;
			}

			return geom->render();
		}
	}

	return true;
}

bool
DRRenderer::showTexture( GLuint texID, int quarter )
{
	GLint status;

	glUseProgram( 0 );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "ERROR in DRRenderer::showTexture: glUseProgram( 0 ) failed with " << gluErrorString( status ) << endl;
		return false;
	}

	glm::mat4 orthoMat = this->m_camera->createOrthoProj( false, true );
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();

	glLoadMatrixf( glm::value_ptr( orthoMat ) );

	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();

	glActiveTexture( GL_TEXTURE0 );
	glBindTexture( GL_TEXTURE_2D, texID );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "ERROR in DRRenderer::showTexture: glBindTexture failed with " << gluErrorString( status ) << endl;
		return false;
	}

	int counter = 0;
	float height = ( float ) this->m_camera->getHeight() / 2.0f;
	float width = ( float ) this->m_camera->getWidth() / 2.0f;

	for ( int i = 0; i < 2; i++ )
	{
		for ( int j = 0; j < 2; j++ )
		{
			if ( counter == quarter )
			{
				// render quad
				glBegin( GL_QUADS );
					glTexCoord2f( 0.0f, 1.0f ); glVertex2f( width * j, height * i );
					glTexCoord2f( 0.0f, 0.0f ); glVertex2f( width * j, height * i + height );
					glTexCoord2f( 1.0f, 0.0f ); glVertex2f( width * j + width, height * i + height );
					glTexCoord2f( 1.0f, 1.0f ); glVertex2f( width * j + width, height * i );
				glEnd();
			}

			counter++;
		}
	}

	return true;
}
