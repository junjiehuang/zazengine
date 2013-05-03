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

#include "../Util/GLUtils.h"
#include "../ZazenGraphics.h"
#include "../Geometry/GeomSkyBox.h"
#include "../Geometry/GeometryFactory.h"
#include "../Program/ProgramManagement.h"
#include "../Program/UniformManagement.h"

#include <iostream>
#include <algorithm>

using namespace std;

DRRenderer::DRRenderer()
	: Renderer( )
{
	this->m_gBufferFbo = NULL;
	this->m_intermediateDepthFB = NULL;

	this->m_depthCopy = NULL;

	this->m_progGeomStage = NULL;
	this->m_progSkyBox = NULL;
	this->m_progLightingStage = NULL;
	this->m_progLightingNoShadowStage = NULL;
	this->m_progShadowMapping = NULL;
	this->m_progTransparency = NULL;
	this->m_progBlendTransparency = NULL;

	this->m_transformsBlock = NULL;
	this->m_cameraBlock = NULL;
	this->m_lightBlock = NULL;
	this->m_materialBlock = NULL;

	this->m_fullScreenQuad = NULL;

	this->m_unitCubeMatrix = glm::mat4(
		0.5, 0.0, 0.0, 0.0, 
		0.0, 0.5, 0.0, 0.0,
		0.0, 0.0, 0.5, 0.0,
		0.5, 0.5, 0.5, 1.0
	);
}

DRRenderer::~DRRenderer()
{
}

bool
DRRenderer::initialize()
{
	ZazenGraphics::getInstance().getLogger().logInfo( "Initializing Deferred Renderer..." );

	glEnable( GL_DEPTH_TEST );								// Enables Depth Testing

	// Cull triangles which normal is not towards the camera
	glEnable( GL_CULL_FACE );

	glColorMask( GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE );
	glClearColor( 0.0, 0.0, 0.0, 1.0 );

	if ( false == this->initFBOs() )
	{
		return false;
	}

	if ( false == this->initGeomStage() )
	{
		return false;
	}

	if ( false == this->initLightingStage() )
	{
		return false;
	}

	if ( false == this->initShadowMapping() )
	{
		return false;
	}

	if ( false == this->initTransparency() )
	{
		return false;
	}

	if ( false == this->initUniformBlocks() )
	{
		return false;
	}

	ZazenGraphics::getInstance().getLogger().logInfo( "Initializing Deferred Renderer finished" );

	return true;
}

bool
DRRenderer::shutdown()
{
	ZazenGraphics::getInstance().getLogger().logInfo( "Shutting down Deferred Renderer..." );

	Program::unuse();

	FrameBufferObject::destroy( this->m_intermediateDepthFB );

	// cleaning up framebuffer
	FrameBufferObject::destroy( this->m_gBufferFbo );
	
	// deleting shadow-map pool
	RenderTarget::cleanup();

	ZazenGraphics::getInstance().getLogger().logInfo( "Shutting down Deferred Renderer finished" );

	return true;
}

bool
DRRenderer::initFBOs()
{
	if ( false == this->initGBuffer() )
	{
		return false;
	}

	if ( false == this->initIntermediateDepthBuffer() )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::initGBuffer()
{
	this->m_gBufferFbo = FrameBufferObject::create();
	if ( NULL == this->m_gBufferFbo )
	{
		return false;
	}

	// needs to be bound to attach render-targets
	// the order of the following calls is important and depermines the index
	if ( false == this->m_gBufferFbo->bind() )
	{
		return false;
	}
	
	glColorMask( GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE );
	glClearColor( 0.0, 0.0, 0.0, 1.0 );

	// render-target at index 0: diffuse color
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR, this->m_gBufferFbo ) )		
	{
		return false;
	}

	// render-target at index 1: normals
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR, this->m_gBufferFbo ) )		
	{
		return false;
	}

	// render-target at index 2: positions in viewing-coords (Eye-Space)
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR, this->m_gBufferFbo ) )		
	{
		return false;
	}

	// render-target at index 3: intermediate lighting-result
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR, this->m_gBufferFbo ) )		
	{
		return false;
	}

	// render-target at index 4: depth-buffer
	if ( false == this->createMrtBuffer( RenderTarget::RT_DEPTH, this->m_gBufferFbo ) )		
	{
		return false;
	}

	// IMPORTANT: don't check status until all necessary buffers are created
	// and attached to the according fbo (e.g. after each attach) otherwise
	// it will be incomplete at some point and will fail.
	// check status of FBO AFTER all buffers are set-up and attached
	if ( false == this->m_gBufferFbo->checkStatus() )
	{
		return false;
	}

	// unbind => switch back to default framebuffer
	if ( false == this->m_gBufferFbo->unbind() )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::initIntermediateDepthBuffer()
{
	// shadow-mapping is done by rendering the depth of the current processing light into an own FBO
	this->m_intermediateDepthFB = FrameBufferObject::create();
	if ( NULL == this->m_intermediateDepthFB )
	{
		return false;
	}

	// bind the shadow FBO to init stuff
	if ( false == this->m_intermediateDepthFB->bind() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initIntermediateDepthBuffer: coulnd't bind shadow FBO - exit" );
		return false;
	}

	// IMPORTANT: disable drawing&reading from this fbo is important, otherwise will fail as incomplete with depth-only attachment
	// no drawing & reading in shadow-fbo, set initial, fbo will keep this state, no need to set it every bind
	this->m_intermediateDepthFB->drawNone();

	// back to default frame-buffer
	if ( false == this->m_intermediateDepthFB->unbind() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initIntermediateDepthBuffer: coulnd't unbind shadow FBO - exit" );
		return false;
	}

	return true;
}

bool
DRRenderer::createMrtBuffer( RenderTarget::RenderTargetType targetType, FrameBufferObject* fbo )
{
	RenderTarget* renderTarget = RenderTarget::create( ( GLsizei ) this->m_camera->getWidth(), 
		( GLsizei ) this->m_camera->getHeight(), targetType );
	if ( NULL == renderTarget )
	{
		return false;
	}

	if ( false == fbo->attachTarget( renderTarget ) )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::initGeomStage()
{
	ZazenGraphics::getInstance().getLogger().logInfo( "Initializing Deferred Rendering Geometry-Stage..." );

	this->m_progGeomStage = ProgramManagement::get( "GeometryStageProgramm" );
	if ( 0 == this->m_progGeomStage )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initGeomStage: coulnd't get program - exit" );
		return false;
	}

	this->m_progSkyBox = ProgramManagement::get( "SkyBoxProgram" );
	if ( 0 == this->m_progSkyBox )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initGeomStage: coulnd't create program - exit" );
		return false;
	}

	ZazenGraphics::getInstance().getLogger().logInfo( "Initializing Deferred Rendering Geometry-Stage finished" );

	return true;
}

bool
DRRenderer::initLightingStage()
{
	ZazenGraphics::getInstance().getLogger().logInfo( "Initializing Deferred Rendering Lighting-Stage..." );

	this->m_progLightingStage = ProgramManagement::get( "LightingStageProgramm" );
	if ( 0 == this->m_progLightingStage )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initLightingStage: coulnd't create program - exit" );
		return false;
	}

	this->m_progLightingNoShadowStage = ProgramManagement::get( "LightingNoShadowStageProgramm" );
	if ( 0 == this->m_progLightingNoShadowStage )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initLightingStage: coulnd't create program - exit" );
		return false;
	}

	ZazenGraphics::getInstance().getLogger().logInfo( "Initializing Deferred Rendering Lighting-Stage finished" );

	return true;
}

bool
DRRenderer::initShadowMapping()
{
	ZazenGraphics::getInstance().getLogger().logInfo( "Initializing Deferred Rendering Shadow-Mapping..." );

	this->m_progShadowMapping = ProgramManagement::get( "ShadowingStageProgramm" );
	if ( 0 == this->m_progShadowMapping )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initShadowMapping: coulnd't create program - exit" );
		return false;
	}

	ZazenGraphics::getInstance().getLogger().logInfo( "Initializing Deferred Rendering Shadow-Mapping finished" );

	return true;
}

bool
DRRenderer::initTransparency()
{
	ZazenGraphics::getInstance().getLogger().logInfo( "Initializing Deferred Rendering Transparency-Stage..." );

	this->m_progTransparency = ProgramManagement::get( "TransparencyProgram" );
	if ( 0 == this->m_progTransparency )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initTransparency: coulnd't create program - exit" );
		return false;
	}

	this->m_progBlendTransparency = ProgramManagement::get( "BlendTransparencyProgram" );
	if ( 0 == this->m_progBlendTransparency )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initTransparency: coulnd't create program - exit" );
		return false;
	}
	
	this->m_fullScreenQuad = GeometryFactory::createQuad( ( float ) this->m_camera->getWidth(), ( float ) this->m_camera->getHeight() );
	if ( NULL == this->m_fullScreenQuad )
	{
		return false;
	}

	this->m_depthCopy = RenderTarget::create( ( GLsizei ) this->m_camera->getWidth(), ( GLsizei ) this->m_camera->getHeight(), RenderTarget::RT_SHADOW );
	if ( NULL == this->m_depthCopy )
	{
		return false;
	}

	ZazenGraphics::getInstance().getLogger().logInfo( "Initializing Deferred Rendering Transparency-Stage finished" );

	return true;
}

bool
DRRenderer::initUniformBlocks()
{
	this->m_transformsBlock = UniformManagement::getBlock( "TransformUniforms" );
	if ( NULL == this->m_transformsBlock )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initUniformBlocks: couldn't find transforms uniform-block - exit" );
		return false;
	}

	this->m_cameraBlock = UniformManagement::getBlock( "CameraUniforms" );
	if ( 0 == this->m_cameraBlock )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initUniformBlocks: couldn't find camera uniform-block - exit" );
		return false;
	}

	this->m_lightBlock = UniformManagement::getBlock( "LightUniforms" );
	if ( 0 == this->m_lightBlock )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initUniformBlocks: couldn't find light uniform-block - exit" );
		return false;
	}

	this->m_materialBlock = UniformManagement::getBlock( "MaterialUniforms" );
	if ( 0 == this->m_materialBlock )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initUniformBlocks: couldn't find material uniform-block - exit" );
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
	if ( false == this->m_transformsBlock->bindBase() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initUniformBlocks: binding transform uniform-block failed - exit" );
		return false;
	}
	if ( false == this->m_cameraBlock->bindBase() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initUniformBlocks: binding camera uniform-block failed - exit" );
		return false;
	}
	if ( false == this->m_lightBlock->bindBase() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initUniformBlocks: binding light uniform-block failed - exit" );
		return false;
	}
	if ( false == this->m_materialBlock->bindBase() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initUniformBlocks: binding material uniform-block failed - exit" );
		return false;
	}

	// TODO clean-up: different approach in future
	GeomSkyBox::getRef().setTransformBlock( this->m_transformsBlock );

	return true;
}

bool
DRRenderer::renderFrame( std::list<Instance*>& instances, std::list<Light*>& lights )
{
	if ( false == this->doGeometryStage( instances, lights ) )
	{
		return false;
	}

	if ( false == this->doLightingStage( instances, lights ) )
	{
		return false;
	}

	if ( false == this->doTransparencyStage( instances, lights ) )
	{
		return false;
	}
	
#ifndef CHECK_GL_ERRORS
	// if checking of gl-errors is deactivated check once per frame for ALL errors
	if ( false == GLUtils::peekErrors() )
	{
		return false;
	}
#endif

	this->frame++;

	return true;
}

bool
DRRenderer::doGeometryStage( std::list<Instance*>& instances, std::list<Light*>& lights )
{
	// render to g-buffer FBO
	if ( false == this->m_gBufferFbo->bind() )
	{
		return false;
	}

	// IMPORTANT: need to re-set the viewport for each FBO
	// could have changed due to shadow-map or other rendering happend in the frame before
	this->m_camera->restoreViewport();

	// clear all targets for the new frame
	if ( false == this->m_gBufferFbo->clearAll() )
	{
		return false;
	}

	// render sky-box first with disabled depth-write => will be behind all other objects
	if ( false == this->renderSkyBox() )
	{
		return false;
	}

	// activate geometry-stage program
	if ( false == this->m_progGeomStage->use() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::renderGeometryStage: using shadow mapping program failed - exit" );
		return false;
	}

	vector<unsigned int> indices;
	indices.push_back( 0 ); // diffuse
	indices.push_back( 1 );	// normal
	indices.push_back( 2 );	// position

	// enable rendering to all render-targets in geometry-stage
	if ( false == this->m_gBufferFbo->drawBuffers( indices ) )
	{
		return false;
	}

	// check status of FBO, IMPORANT: not before, would have failed
	if ( false == this->m_gBufferFbo->checkStatus() )
	{
		return false;
	}

	// draw all geometry from cameras viewpoint AND apply materials but ignore transparent material
	if ( false == this->renderInstances( this->m_camera, instances, this->m_progGeomStage, true, false ) )
	{
		return false;
	}

	// switch back to default framebuffer
	if ( false == this->m_gBufferFbo->unbind() )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::renderSkyBox()
{
	// skip sky-box rendering if not present
	if ( false == GeomSkyBox::isPresent() )
	{
		return true;
	}

	// only render to buffer with index 0 which is diffuse-color only
	// IMPORANT: sky-box doesn't render normals, depth and other stuff
	//		because lighting and other effects won't be applied, only diffuse color matters
	if ( false == this->m_gBufferFbo->drawBuffer( 0 ) )
	{
		return false;
	}

	// check FBO-status
	if ( false == this->m_gBufferFbo->checkStatus() )
	{
		return false;
	}

	// sky-box rendering uses its own program
	if ( false == this->m_progSkyBox->use() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::renderSkyBox: using program failed - exit" );
		return false;
	}

	this->m_progSkyBox->setUniformInt( "SkyBoxCubeMap", 0 );

	// render the geometry
	GeomSkyBox::getRef().render();

	return true;
}

bool
DRRenderer::doLightingStage( std::list<Instance*>& instances, std::list<Light*>& lights )
{
	glm::vec4 cameraRectangle;
	cameraRectangle[ 0 ] = ( float ) this->m_camera->getWidth();
	cameraRectangle[ 1 ] = ( float ) this->m_camera->getHeight();

	// bind camera uniform-block to update data
	if ( false == this->m_cameraBlock->bindBuffer() )
	{
		return false;
	}

	// upload world-orientation of camera ( its model-matrix )
	if ( false == this->m_cameraBlock->updateField( "Camera.modelMatrix", this->m_camera->getModelMatrix() ) )
	{
		return false;
	}
	// upload view-matrix of camera (need to transform e.g. light-world position in EyeCoords/Viewspace)
	if ( false == this->m_cameraBlock->updateField( "Camera.viewMatrix", this->m_camera->getViewMatrix() ) )
	{
		return false;
	}
	// upload camera-rectangle
	if ( false == this->m_cameraBlock->updateField( "Camera.rectangle", cameraRectangle ) )
	{
		return false;
	}

	// render the contribution of each light to the scene
	std::list<Light*>::iterator iter = lights.begin();
	while ( iter != lights.end() )
	{
		Light* light = *iter++;

		if ( false == this->renderLight( instances, light ) )
		{
			return false;
		}
	}

	// switch back to default framebuffer
	if ( false == this->m_intermediateDepthFB->unbind() )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::renderLight( std::list<Instance*>& instances, Light* light )
{
	// different programs could be active according whether light is shadow-caster or not
	// IMPORANT: unbinding textures will fail when shadowmap is not used within program due to no shadow-mapping
	Program* activeLightingProgram = NULL;

	// light is shadow-caster: render shadow map for this light
	if ( light->isShadowCaster() )
	{
		if ( false == this->renderShadowMap( instances, light ) )
		{
			return false;
		}

		activeLightingProgram = this->m_progLightingStage;
	}
	// light is no shadow-caster => choose different lighting-stage program
	else
	{
		activeLightingProgram = this->m_progLightingNoShadowStage;
	}

	// render to g-buffer
	if ( false == this->m_gBufferFbo->bind() )
	{
		return false;
	}

	// IMPORTANT: need to re-set the viewport for each FBO
	// could have changed due to shadow-map or other rendering happend in the frame before
	this->m_camera->restoreViewport();

	// enable rendering to 3rd target
	if ( false == this->m_gBufferFbo->drawBuffer( 3 ) )
	{
		return false;
	}

	// check status of FBO, IMPORANT: not before, would have failed
	if ( false == this->m_gBufferFbo->checkStatus() )
	{
		return false;
	}

	// activate lighting-stage shader
	if ( false == activeLightingProgram->use() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::renderLight: using program failed - exit" );
		return false;
	}

	vector<unsigned int> indices;
	indices.push_back( 0 ); // diffuse
	indices.push_back( 1 );	// normals
	indices.push_back( 2 );	// positions
	indices.push_back( 4 );	// depth

	// OPTIMIZE: no need to do for every light!
	// lighting stage program need all buffers of g-buffer bound as textures
	if ( false == this->m_gBufferFbo->bindTargets( indices ) )
	{
		return false;
	}

	// tell lighting program that diffusemap is bound to texture-unit 0
	activeLightingProgram->setUniformInt( "DiffuseMap", 0 );
	// tell lighting program that normalmap is bound to texture-unit 1
	activeLightingProgram->setUniformInt( "NormalMap", 1 );
	// tell lighting program that generic map is bound to texture-unit 2
	activeLightingProgram->setUniformInt( "PositionMap", 2 );
	// tell lighting program that depth-map of scene is bound to texture-unit 3 
	activeLightingProgram->setUniformInt( "DepthMap", 3 );
	
	glm::vec4 lightConfig;
	glm::mat4 lightSpaceUnit;

	lightConfig[ 0 ] = ( float ) light->getType();
	lightConfig[ 1 ] = light->getFalloff();
	lightConfig[ 2 ] = light->isShadowCaster();
		
	// bind light uniform-block to update data of this light
	if ( false == this->m_lightBlock->bindBuffer() )
	{
		return false;
	}

	// upload light-config
	if ( false == this->m_lightBlock->updateField( "Light.config", lightConfig ) )
	{
		return false;
	}
	// upload light-model matrix = orientation of the light in the world
	if ( false == this->m_lightBlock->updateField( "Light.modelMatrix", light->getModelMatrix() ) )
	{
		return false;
	}

	// bind shadow-map when light is shadow-caster
	if ( light->isShadowCaster() )
	{
		// tell program that the shadowmap of spot/directional-light will be available at texture unit 4
		activeLightingProgram->setUniformInt( "ShadowMap", 4 );

		if ( false == light->getShadowMap()->bind( 4 ) )
		{
			return false;
		}

		// calculate the light-space projection matrix
		// multiplication with unit-cube is first because has to be carried out the last
		lightSpaceUnit = this->m_unitCubeMatrix * light->getVPMatrix();

		if ( false == this->m_lightBlock->updateField( "Light.spaceUniformMatrix", lightSpaceUnit ) )
		{
			return false;
		}
	}

	// QUESTION: due to a but only bind was called instead of bindBuffer but it worked!! why?
	// update projection-matrix because need ortho-projection for full-screen quad
	this->m_transformsBlock->bindBuffer();
	// OPTIMIZE: store in light once, and only update when change
	glm::mat4 orthoMat = this->m_camera->createOrthoProj( true, true );
	if ( false == this->m_transformsBlock->updateField( "Transforms.projectionMatrix", orthoMat ) )
	{
		return false;
	}

	// TODO: need to set opengl to additiveley blend light-passes
	// alpha? color modulate?...

	// disable writing to depth: light-boundaries should not update depth
	// we also need to READ from depth so no update 
	glDepthMask( GL_FALSE );

	// render light boundary (for now only full screen-quad)
	light->getBoundingGeometry()->render();
	
	// enable depth-writing again
	glDepthMask( GL_TRUE );

	return true;
}

bool
DRRenderer::renderShadowMap( std::list<Instance*>& instances, Light* light )
{
	// use shadow-mapping fbo to render depth of light view-point to
	if ( false == this->m_intermediateDepthFB->bind() )
	{
		return false;
	}

	// attach this shadow-map temporarily to the fbo (other light will use the same fbo)
	if ( false == this->m_intermediateDepthFB->attachTargetTemp( light->getShadowMap() ) )
	{
		return false;
	}

	// check status now
	// IMPORANT: don't check too early
	if ( false == this->m_intermediateDepthFB->checkStatus() )
	{
		return false;
	}

	// clear bound buffer
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	// use shadow-mapping program
	if ( false == this->m_progShadowMapping->use() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::renderShadowMap: using program failed - exit" );
		return false;
	}

	// IMPORTANT: need to set the viewport for each shadow-map, because resolution can be different for each
	light->restoreViewport();

	// render scene from view of camera - don't apply material, we need only depth, render transparent materials
	if ( false == this->renderInstances( light, instances, this->m_progShadowMapping, false, true ) )
	{
		return false;
	}

	// render scene from view of camera - don't apply material, we need only depth, render opaque materials
	if ( false == this->renderInstances( light, instances, this->m_progShadowMapping, false, false ) )
	{
		return false;
	}

	// back to default framebuffer
	if ( false == this->m_intermediateDepthFB->unbind() )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::doTransparencyStage( std::list<Instance*>& instances, std::list<Light*>& lights )
{
	if ( false == this->m_progTransparency->use() )
	{
		return false;
	}

	// bind to index 2 because DiffuseColor of Material is at index 0 and NormalMap of Material is at index 1
	this->m_gBufferFbo->getAttachedTargets()[ 3 ]->bind( 2 );
	this->m_depthCopy->bind( 3 );

	this->m_progTransparency->setUniformInt( "Background", 2 );
	this->m_progTransparency->setUniformInt( "BackgroundDepth", 3 );

	this->m_gBufferFbo->bind();

	if ( false == this->m_gBufferFbo->copyDepthToTarget( this->m_depthCopy ) )
	{
		return false;
	}

	this->m_gBufferFbo->drawBuffer( 0 );

	glClear( GL_COLOR_BUFFER_BIT );

	if ( false == this->renderInstances( this->m_camera, instances, this->m_progTransparency, true, true ) )
	{
		return false;
	}

	// back to default-framebuffer
	this->m_gBufferFbo->unbind();

	// clear default-framebuffer color & depth
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	if ( false == this->m_progBlendTransparency->use() )
	{
		return false;
	}

	this->m_gBufferFbo->getAttachedTargets()[ 0 ]->bind( 0 );

	this->m_progBlendTransparency->setUniformInt( "Background", 2 );
	this->m_progBlendTransparency->setUniformInt( "Transparent", 0 );
	
	// QUESTION: due to a but only bind was called instead of bindBuffer but it worked!! why?
	// update projection-matrix because need ortho-projection for full-screen quad
	this->m_transformsBlock->bindBuffer();
	// OPTIMIZE: store in light once, and only update when change
	glm::mat4 orthoMat = this->m_camera->createOrthoProj( true, true );
	if ( false == this->m_transformsBlock->updateField( "Transforms.projectionMatrix", orthoMat ) )
	{
		return false;
	}

	this->m_fullScreenQuad->render();

	return true;
}

bool
DRRenderer::renderInstances( Viewer* viewer, list<Instance*>& instances, Program* currentProgramm, bool applyMaterial, bool renderTransparency )
{
	// bind transform uniform-block to update model-, view & projection transforms
	if ( false == this->m_transformsBlock->bindBuffer() )
	{
		return false;
	}

	// update projection because each viewer can have different projection-transform
	if ( false == this->m_transformsBlock->updateField( "Transforms.projectionMatrix", viewer->getProjMatrix() ) )
	{
		return false;
	}

	list<Instance*>::iterator iter = instances.begin();
	while ( iter != instances.end() )
	{
		Instance* instance = *iter++;

		if ( instance->material )
		{

			if ( ( Material::MATERIAL_TRANSPARENT == instance->material->getType() && ! renderTransparency ) ||
				( Material::MATERIAL_TRANSPARENT != instance->material->getType() && renderTransparency ) )
			{
				continue;
			}

			// activate material only when there is one present & render-pass enforces usage of materials
			if ( applyMaterial )
			{
				// need to bind material uniform-block to update data
				if ( false == this->m_materialBlock->bindBuffer() )
				{
					return false;
				}

				// activate material
				if ( false == instance->material->activate( this->m_materialBlock, currentProgramm ) )
				{
					return false;
				}

				// back to transform uniform-block because is needed in the actual rendering of geometry (modelview calculation)
				if ( false == this->m_transformsBlock->bindBuffer() )
				{
					return false;
				}
			}
		}

		// render geometry of this instance
		if ( false == this->renderGeom( viewer, instance->geom, instance->getModelMatrix() ) )
		{
			return false;
		}
	}

	return true;
}

bool
DRRenderer::renderGeom( Viewer* viewer, GeomType* geom, const glm::mat4& rootModelMatrix )
{
	// IMPORANT: OpenGL applies last matrix in multiplication as first transformation to object
	// apply model-transformations recursive
	glm::mat4 modelMatrix = rootModelMatrix * geom->getModelMatrix();
	const std::vector<GeomType*>& children = geom->getChildren();

	if ( children.size() )
	{
		for ( unsigned int i = 0; i < children.size(); i++ )
		{
			GeomType* child = children[ i ];
			// recursively process children
			if ( false == this->renderGeom( viewer, children[ i ], modelMatrix ) )
			{
				return false;
			}
		}
	}
	else
	{
		// cull objects using the viewers point of view
		Viewer::CullResult cullResult = viewer->cullBB( geom->getBBMin(), geom->getBBMax() );
		if ( Viewer::OUTSIDE != cullResult )
		{
			// calculate modelView-Matrix
			glm::mat4 modelViewMatrix = viewer->getViewMatrix() * modelMatrix;
			// IMPORTANT: normal-vectors are transformed different than vertices
			// take the transpose of the inverse modelView or simply reset the translation vector in the modelview-matrix
			// in other words: only the rotations are applied to normals and they are guaranteed to leave
			// normalized normals at unit length. THIS METHOD ONLY WORKS WHEN NO NON UNIFORM SCALING IS APPLIED
			glm::mat4 normalModelViewMatrix = glm::transpose( glm::inverse( modelViewMatrix ) );

			// update model-view matrix
			if ( false == this->m_transformsBlock->updateField( "Transforms.modelViewMatrix", modelViewMatrix) )
			{
				return false;
			}
			// update model-view matrix for normals
			if ( false == this->m_transformsBlock->updateField( "Transforms.normalsModelViewMatrix", normalModelViewMatrix ) )
			{
				return false;
			}

			// render geometry
			return geom->render();
		}
	}

	return true;
}
