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
#include "SkyBox.h"

#include "../Util/GLUtils.h"
#include "../ZazenGraphics.h"
#include "../Geometry/GeometryFactory.h"
#include "../Program/ProgramManagement.h"
#include "../Program/UniformManagement.h"

#include "../context/RenderingContext.h"

#include <iostream>
#include <algorithm>

using namespace std;

DRRenderer::DRRenderer()
{
	this->m_gBufferFbo = NULL;
	this->m_intermediateDepthFB = NULL;

	this->m_depthCopy = NULL;

	this->m_progGeomStaticStage = NULL;
	this->m_progGeomAnimStage = NULL;
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
	this->m_transparentMaterialBlock = NULL;

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

	glEnable( GL_DEPTH_TEST );

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

	// render-target at index 3: tangents
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR, this->m_gBufferFbo ) )		
	{
		return false;
	}

	// render-target at index 4: bi-tangents
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR, this->m_gBufferFbo ) )		
	{
		return false;
	}

	// render-target at index 5: intermediate lighting-result
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR, this->m_gBufferFbo ) )		
	{
		return false;
	}

	// render-target at index 6: depth-buffer
	if ( false == this->createMrtBuffer( RenderTarget::RT_DEPTH, this->m_gBufferFbo ) )		
	{
		return false;
	}

	// IMPORTANT: don't check status until all necessary buffers are created
	// and attached to the according fbo (e.g. after each attach) otherwise
	// it will be incomplete at some point and will fail.
	// check status of FBO AFTER all buffers are set-up and attached
	if ( false == FrameBufferObject::checkStatus() )
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
	RenderTarget* renderTarget = RenderTarget::create( RenderingContext::getRef().getWidth(), RenderingContext::getRef().getHeight(), targetType );
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

	this->m_progGeomStaticStage = ProgramManagement::get( "GeometryStaticStageProgramm" );
	if ( 0 == this->m_progGeomStaticStage )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initGeomStage: coulnd't get static geometry program - exit" );
		return false;
	}

	this->m_progGeomAnimStage = ProgramManagement::get( "GeometryAnimStageProgramm" );
	if ( 0 == this->m_progGeomAnimStage )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initGeomStage: coulnd't get animated geometry program - exit" );
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
	
	this->m_fullScreenQuad = GeometryFactory::createQuad( ( float ) RenderingContext::getRef().getWidth(), ( float ) RenderingContext::getRef().getHeight() );
	if ( NULL == this->m_fullScreenQuad )
	{
		return false;
	}

	this->m_depthCopy = RenderTarget::create( RenderingContext::getRef().getWidth(), RenderingContext::getRef().getHeight(), RenderTarget::RT_SHADOW );
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

	this->m_transparentMaterialBlock = UniformManagement::getBlock( "TransparentMaterialUniforms" );
	if ( 0 == this->m_transparentMaterialBlock )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initUniformBlocks: couldn't find transparent material uniform-block - exit" );
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
	if ( false == this->m_transparentMaterialBlock->bindBase() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initUniformBlocks: binding transparent material uniform-block failed - exit" );
		return false;
	}

	return true;
}

bool
DRRenderer::renderFrame( std::list<ZazenGraphicsEntity*>& entities )
{
	// iterate over all cameras in scene
	list<ZazenGraphicsEntity*>::iterator iter = entities.begin();
	while ( iter != entities.end() )
	{
		ZazenGraphicsEntity* entity = *iter++;

		if ( entity->getCamera() )
		{
			this->m_mainCamera = entity->getCamera();
			
			// TODO: need to perform some culling by this camera

			this->preProcessTransparency( entities );

			if ( false == this->doGeometryStage( entities ) )
			{
				return false;
			}

			if ( false == this->doLightingStage( entities ) )
			{
				return false;
			}

			if ( false == this->doTransparencyStage( entities ) )
			{
				return false;
			}
		}
	}

	
	// check once per frame for ALL errors regardless if we are in _DEBUG or not
	if ( GL_PEEK_ERRORS )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::renderFrame: pending OpenGL-Errors - run the DEBUG-build to narrow and determine the call responsible for the error - exit" );
		return false;
	}

	return true;
}

void
DRRenderer::preProcessTransparency( std::list<ZazenGraphicsEntity*>& entities )
{
	this->m_transparentEntities.clear();

	list<ZazenGraphicsEntity*>::iterator iter = entities.begin();
	while ( iter != entities.end() )
	{
		ZazenGraphicsEntity* entity = *iter++;

		if ( entity->getMaterial() )
		{
			if ( Material::MATERIAL_TRANSPARENT == entity->getMaterial()->getType() )
			{
				// need to relcalculate distance from viewer for depth-sorting
				entity->recalculateDistance( this->m_mainCamera->getViewMatrix() );

				this->m_transparentEntities.push_back( entity );
			}
		}
	}

	// do depth-sorting
	std::sort( this->m_transparentEntities.begin(), this->m_transparentEntities.end(), DRRenderer::depthSortingFunc );

}

bool
DRRenderer::doGeometryStage( std::list<ZazenGraphicsEntity*>& entities )
{
	list<ZazenGraphicsEntity*> staticInstances;
	list<ZazenGraphicsEntity*> animatedInstances;

	// render to g-buffer FBO
	if ( false == this->m_gBufferFbo->bind() )
	{
		return false;
	}

	// IMPORTANT: need to re-set the viewport for each FBO
	// could have changed due to shadow-map or other rendering happend in the frame before
	this->m_mainCamera->restoreViewport();

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

	list<ZazenGraphicsEntity*>::iterator iter = entities.begin();
	while ( iter != entities.end() )
	{
		ZazenGraphicsEntity* entity = *iter++;
		Animation* animation = entity->getAnimation();

		if ( 0 == entity->getMesh() )
		{
			continue;
		}

		if ( animation )
		{
			animatedInstances.push_back( entity );
		}
		else
		{
			staticInstances.push_back( entity );
		}
	}

	if ( false == this->renderGeometry( staticInstances, this->m_progGeomStaticStage ) )
	{
		return false;
	}

	if ( false == this->renderGeometry( animatedInstances, this->m_progGeomAnimStage ) )
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
	if ( false == SkyBox::isPresent() )
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
	CHECK_FRAMEBUFFER_DEBUG

	// sky-box rendering uses its own program
	if ( false == this->m_progSkyBox->use() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::renderSkyBox: using program failed - exit" );
		return false;
	}

	this->m_progSkyBox->setUniformInt( "SkyBoxCubeMap", 0 );

	// render the geometry
	SkyBox::getRef().render( *this->m_mainCamera, this->m_transformsBlock );

	return true;
}

bool
DRRenderer::renderGeometry( std::list<ZazenGraphicsEntity*>& entities, Program* program )
{
	// activate geometry-stage program
	if ( false == program->use() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::renderGeometry: using program failed - exit" );
		return false;
	}

	vector<unsigned int> indices;
	indices.push_back( 0 ); // diffuse
	indices.push_back( 1 );	// normal
	indices.push_back( 2 );	// position
	indices.push_back( 3 );	// tangents
	indices.push_back( 4 );	// bi-tangents

	// enable rendering to all render-targets in geometry-stage
	if ( false == this->m_gBufferFbo->drawBuffers( indices ) )
	{
		return false;
	}

	// check status of FBO, IMPORANT: not before, would have failed
	CHECK_FRAMEBUFFER_DEBUG

	// draw all geometry from cameras viewpoint AND apply materials but ignore transparent material
	if ( false == this->renderEntities( this->m_mainCamera, entities, program, true, false ) )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::doLightingStage( std::list<ZazenGraphicsEntity*>& entities )
{
	glm::vec4 cameraRectangle;
	cameraRectangle[ 0 ] = ( float ) this->m_mainCamera->getWidth();
	cameraRectangle[ 1 ] = ( float ) this->m_mainCamera->getHeight();

	// bind camera uniform-block to update data
	if ( false == this->m_cameraBlock->bindBuffer() )
	{
		return false;
	}

	// upload world-orientation of camera ( its model-matrix )
	this->m_cameraBlock->updateField( "CameraUniforms.modelMatrix", this->m_mainCamera->getModelMatrix() );
	// upload view-matrix of camera (need to transform e.g. light-world position in EyeCoords/Viewspace)
	this->m_cameraBlock->updateField( "CameraUniforms.viewMatrix", this->m_mainCamera->getViewMatrix() );
	// upload camera-rectangle
	this->m_cameraBlock->updateField( "CameraUniforms.rectangle", cameraRectangle );

	// no transparent objects in scene, render lighting to default framebuffer
	if ( 0 == this->m_transparentEntities.size() )
	{
		// clear frame-buffer
		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	}

	// render the contribution of each light-entiy to the scene
	std::list<ZazenGraphicsEntity*>::iterator iter = entities.begin();
	while ( iter != entities.end() )
	{
		ZazenGraphicsEntity* entiy = *iter++;
		Light* light = entiy->getLight();
		if ( light )
		{
			if ( false == this->renderLight( entities, light ) )
			{
				return false;
			}
		}
	}

	if ( 0 < this->m_transparentEntities.size() )
	{
		this->m_gBufferFbo->unbind();
	}

	return true;
}

bool
DRRenderer::renderLight( std::list<ZazenGraphicsEntity*>& entities, Light* light )
{
	// different programs could be active according whether light is shadow-caster or not
	// IMPORANT: unbinding textures will fail when shadowmap is not used within program due to no shadow-mapping
	Program* activeLightingProgram = NULL;

	// light is shadow-caster: render shadow map for this light
	if ( light->isShadowCaster() )
	{
		if ( false == this->renderShadowMap( entities, light ) )
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

	// if there are some transparent objects in scene, render lighting result to intermediate 
	// 5th g-buffer target because needed as background for transparency blending
	if ( 0 < this->m_transparentEntities.size() )
	{
		// render to g-buffer
		if ( false == this->m_gBufferFbo->bind() )
		{
			return false;
		}

		// enable rendering to 5th target
		if ( false == this->m_gBufferFbo->drawBuffer( 5 ) )
		{
			return false;
		}

		// check status of FBO, IMPORANT: not before, would have failed
		CHECK_FRAMEBUFFER_DEBUG
	}

	// IMPORTANT: need to re-set the viewport for each FBO
	// could have changed due to shadow-map or other rendering happend in the frame before
	this->m_mainCamera->restoreViewport();

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
	indices.push_back( 3 );	// tangents
	indices.push_back( 4 );	// bi-tangents
	indices.push_back( 6 );	// depth

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
	// tell lighting program that tangents-map of scene is bound to texture-unit 3 
	activeLightingProgram->setUniformInt( "TangentMap", 3 );
	// tell lighting program that bi-tangents-map of scene is bound to texture-unit 4 
	activeLightingProgram->setUniformInt( "BiTangentMap", 4 );
	// tell lighting program that depth-map of scene is bound to texture-unit 5 
	activeLightingProgram->setUniformInt( "DepthMap", 5 );
	
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
	this->m_lightBlock->updateField( "LightUniforms.config", lightConfig );
	// upload light-model matrix = orientation of the light in the world
	this->m_lightBlock->updateField( "LightUniforms.modelMatrix", light->getModelMatrix() );

	// bind shadow-map when light is shadow-caster
	if ( light->isShadowCaster() )
	{
		// tell program that the shadowmap of spot/directional-light will be available at texture unit 6
		activeLightingProgram->setUniformInt( "ShadowMap", 6 );

		if ( false == light->getShadowMap()->bind( 6 ) )
		{
			return false;
		}

		// calculate the light-space projection matrix
		// multiplication with unit-cube is first because has to be carried out the last
		lightSpaceUnit = this->m_unitCubeMatrix * light->getVPMatrix();

		this->m_lightBlock->updateField( "LightUniforms.spaceUniformMatrix", lightSpaceUnit );
	}

	// QUESTION: due to a but only bind was called instead of bindBuffer but it worked!! why?
	// update projection-matrix because need ortho-projection for full-screen quad
	this->m_transformsBlock->bindBuffer();
	// OPTIMIZE: store in light once, and only update when change
	glm::mat4 orthoMat = this->m_mainCamera->createOrthoProj( true, true );
	this->m_transformsBlock->updateField( "TransformUniforms.projectionMatrix", orthoMat );

	// TODO: need to set opengl to additiveley blend light-passes
	// alpha? color modulate?...

	// disable writing to depth: light-boundaries should not update depth
	// we also need to READ from depth so no update 
	glDepthMask( GL_FALSE );

	// render light boundary (for now only full screen-quad)
	light->getBoundingMesh()->render();
	
	// enable depth-writing again
	glDepthMask( GL_TRUE );

	return true;
}

bool
DRRenderer::renderShadowMap( std::list<ZazenGraphicsEntity*>& entities, Light* light )
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
	CHECK_FRAMEBUFFER_DEBUG

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
	if ( false == this->renderEntities( light, entities, this->m_progShadowMapping, false, true ) )
	{
		return false;
	}

	// render scene from view of camera - don't apply material, we need only depth, render opaque materials
	if ( false == this->renderEntities( light, entities, this->m_progShadowMapping, false, false ) )
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
DRRenderer::doTransparencyStage( std::list<ZazenGraphicsEntity*>& entities )
{
	unsigned int combinationTarget = 1;
	unsigned int backgroundIndex = 5;

	for ( unsigned int i = 0; i < this->m_transparentEntities.size(); i++ )
	{
		ZazenGraphicsEntity* entity = this->m_transparentEntities[ i ];

		if ( false == this->renderTransparentInstance( entity, backgroundIndex, combinationTarget,
			i == this->m_transparentEntities.size() - 1 ) )
		{
			return false;
		}

		std::swap( combinationTarget, backgroundIndex );
	}

	return true;
}

bool
DRRenderer::renderTransparentInstance( ZazenGraphicsEntity* entity, unsigned int backgroundIndex, unsigned int combinationTarget, bool lastInstance )
{
	if ( false == this->m_progTransparency->use() )
	{
		return false;
	}

	// bind background to index 2 because DiffuseColor of Material is at index 0 and NormalMap of Material is at index 1
	this->m_gBufferFbo->getAttachedTargets()[ backgroundIndex ]->bind( 2 );
	// bind depth-copy to index 3
	this->m_depthCopy->bind( 3 );

	this->m_progTransparency->setUniformInt( "Background", 2 );
	this->m_progTransparency->setUniformInt( "BackgroundDepth", 3 );

	this->m_gBufferFbo->bind();
	
	// copy depth of g-buffer to target, because we need to access the depth in transparency-rendering
	// to prevent artifacts. at the same time we write depth when rendering transparency so
	// we cannot bind the g-buffer depth => need to copy
	if ( false == this->m_gBufferFbo->copyDepthToTarget( this->m_depthCopy ) )
	{
		return false;
	}

	// transparent objects are always rendered intermediate to g-buffer color target 0
	this->m_gBufferFbo->drawBuffer( 0 );

	// clear buffer 
	glClear( GL_COLOR_BUFFER_BIT );

	if ( false == this->renderEntity( this->m_mainCamera, entity, this->m_progTransparency ) )
	{
		return false;
	}

	// after rendering last instance we will combine to default-framebuffer for final result
	if ( lastInstance )
	{
		// back to default-framebuffer
		this->m_gBufferFbo->unbind();

		// clear default-framebuffer color & depth
		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	}
	else 
	{
		// draw combination to new target (swap-rendering)
		this->m_gBufferFbo->drawBuffer( combinationTarget );

		glClear( GL_COLOR_BUFFER_BIT );
	}

	if ( false == this->m_progBlendTransparency->use() )
	{
		return false;
	}

	// transparent object was rendered to g-buffer color target 0
	this->m_gBufferFbo->getAttachedTargets()[ 0 ]->bind( 0 );

	this->m_progBlendTransparency->setUniformInt( "Background", 2 );
	this->m_progBlendTransparency->setUniformInt( "Transparent", 0 );
	
	// QUESTION: due to a but only bind was called instead of bindBuffer but it worked!! why?
	// update projection-matrix because need ortho-projection for full-screen quad
	this->m_transformsBlock->bindBuffer();
	// OPTIMIZE: store in light once, and only update when change
	glm::mat4 orthoMat = this->m_mainCamera->createOrthoProj( true, true );
	this->m_transformsBlock->updateField( "TransformUniforms.projectionMatrix", orthoMat );

	// disable writing to depth when not last instance because would destroy our depth-buffer
	// when last instance it doesnt matter because we render to screen-buffer
	if ( ! lastInstance )
	{
		glDepthMask( GL_FALSE );
	}

	this->m_fullScreenQuad->render();

	// enable depth-writing again
	if ( ! lastInstance )
	{
		glDepthMask( GL_TRUE );
	}

	return true;
}

bool
DRRenderer::renderEntities( Viewer* viewer, list<ZazenGraphicsEntity*>& entities, Program* currentProgramm, bool applyMaterial, bool renderTransparency )
{
	// bind transform uniform-block to update model-, view & projection transforms
	if ( false == this->m_transformsBlock->bindBuffer() )
	{
		return false;
	}

	// update projection because each viewer can have different projection-transform
	this->m_transformsBlock->updateField( "TransformUniforms.projectionMatrix", viewer->getProjMatrix() );

	list<ZazenGraphicsEntity*>::iterator iter = entities.begin();
	while ( iter != entities.end() )
	{
		ZazenGraphicsEntity* entity = *iter++;
		Material* material = entity->getMaterial();
		Animation* animation = entity->getAnimation();

		if ( 0 == entity->getMesh() )
		{
			continue;
		}

		if ( animation )
		{
			animation->updateToProgram( currentProgramm );
		}

		if ( material )
		{

			if ( ( Material::MATERIAL_TRANSPARENT == material->getType() && ! renderTransparency ) ||
				( Material::MATERIAL_TRANSPARENT != material->getType() && renderTransparency ) )
			{
				continue;
			}

			// activate material only when there is one present & render-pass enforces usage of materials
			if ( applyMaterial )
			{
				// activate material
				if ( false == material->activate( currentProgramm ) )
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
		if ( false == this->renderMesh( viewer, entity->getMesh(), entity->getModelMatrix() ) )
		{
			return false;
		}
	}

	return true;
}

bool
DRRenderer::renderEntity( Viewer* viewer, ZazenGraphicsEntity* entity, Program* currentProgramm )
{
	// bind transform uniform-block to update model-, view & projection transforms
	if ( false == this->m_transformsBlock->bindBuffer() )
	{
		return false;
	}

	// update projection because each viewer can have different projection-transform
	this->m_transformsBlock->updateField( "TransformUniforms.projectionMatrix", viewer->getProjMatrix() );

	// activate material
	if ( false == entity->getMaterial()->activate( currentProgramm ) )
	{
		return false;
	}

	// back to transform uniform-block because is needed in the actual rendering of geometry (modelview calculation)
	if ( false == this->m_transformsBlock->bindBuffer() )
	{
		return false;
	}
		
	// render geometry of this instance
	if ( false == this->renderMesh( viewer, entity->getMesh(), entity->getModelMatrix() ) )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::renderMesh( Viewer* viewer, Mesh* mesh, const glm::mat4& rootModelMatrix )
{
	// IMPORANT: OpenGL applies last matrix in multiplication as first transformation to object
	// apply model-transformations recursive
	glm::mat4 modelMatrix = rootModelMatrix * mesh->getModelMatrix();
	const std::vector<Mesh*>& children = mesh->getChildren();

	if ( children.size() )
	{
		for ( unsigned int i = 0; i < children.size(); i++ )
		{
			Mesh* child = children[ i ];
			// recursively process children
			if ( false == this->renderMesh( viewer, children[ i ], modelMatrix ) )
			{
				return false;
			}
		}
	}
	else
	{
		// cull objects using the viewers point of view
		Viewer::CullResult cullResult = viewer->cullBB( mesh->getBBMin(), mesh->getBBMax() );
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
			this->m_transformsBlock->updateField( "TransformUniforms.modelViewMatrix", modelViewMatrix );
			// update model-view matrix for normals
			this->m_transformsBlock->updateField( "TransformUniforms.normalsModelViewMatrix", normalModelViewMatrix );

			// render geometry
			return mesh->render();
		}
	}

	return true;
}

bool
DRRenderer::depthSortingFunc( ZazenGraphicsEntity* a, ZazenGraphicsEntity* b )
{
	// depth sorting: distance holds the z-value of the center in view-space
	// the larger the negative values the farther way
	return a->getDistance() < b->getDistance(); 
}
