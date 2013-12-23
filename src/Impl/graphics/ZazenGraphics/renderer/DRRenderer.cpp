#include "DRRenderer.h"

#include "SkyBox.h"

#include "../Util/GLUtils.h"
#include "../ZazenGraphics.h"
#include "../Geometry/GeometryFactory.h"
#include "../Program/ProgramManagement.h"
#include "../Program/UniformManagement.h"

#include "../context/RenderingContext.h"

#include <GL/glew.h>

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>
#include <algorithm>

using namespace std;

DRRenderer::DRRenderer()
{
	this->m_fbo = NULL;
	this->m_intermediateDepthFB = NULL;

	this->m_progGeomStage = NULL;
	this->m_progSkyBox = NULL;
	this->m_progLightingStage = NULL;
	this->m_progLightingStageStencilVolume = NULL;
	this->m_progShadowMappingPlanar = NULL;
	this->m_progShadowMappingCubeSinglePass = NULL;
	this->m_progShadowMappingCubeMultiPass = NULL;
	this->m_progTransparency = NULL;
	this->m_progBlendTransparency = NULL;

	this->m_transformsBlock = NULL;
	this->m_cameraBlock = NULL;
	this->m_lightBlock = NULL;
	this->m_materialBlock = NULL;
	this->m_transparentMaterialBlock = NULL;

	this->m_fsq = NULL;

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

	// by default depth-testing is disabled
	glEnable( GL_DEPTH_TEST );
	// culling of faces is disabled by default
	glEnable( GL_CULL_FACE );

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

	this->initializeStaticData();

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
	FrameBufferObject::destroy( this->m_fbo );
	
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
	this->m_fbo = FrameBufferObject::create();
	if ( NULL == this->m_fbo )
	{
		return false;
	}

	// needs to be bound to attach render-targets
	// the order of the following calls is important and depermines the index
	if ( false == this->m_fbo->bind() )
	{
		return false;
	}
	
	// by default alpha is cleared to 0.0 too -> set to 1.0
	glClearColor( 0.0, 0.0, 0.0, 1.0 );

	// render-target at index 0: diffuse color
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR, this->m_fbo ) )		
	{
		return false;
	}

	// render-target at index 1: normals
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR, this->m_fbo ) )		
	{
		return false;
	}

	// render-target at index 2: tangents
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR, this->m_fbo ) )		
	{
		return false;
	}

	// render-target at index 3: bi-tangents
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR, this->m_fbo ) )		
	{
		return false;
	}

	// render-target at index 4: intermediate lighting-result
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR, this->m_fbo ) )		
	{
		return false;
	}

	// render-target at index 5: depth-stencil-buffer
	if ( false == this->createMrtBuffer( RenderTarget::RT_DEPTH_STENCIL, this->m_fbo ) )		
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
	if ( false == this->m_fbo->unbind() )
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

	if ( false == this->createMrtBuffer( RenderTarget::RT_DEPTH, this->m_intermediateDepthFB ) )		
	{
		return false;
	}

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

	this->m_progGeomStage = ProgramManagement::get( "GeometryStageProgramm" );
	if ( 0 == this->m_progGeomStage )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initGeomStage: coulnd't get static geometry program - exit" );
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
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initLightingStage: coulnd't create lighting-stage program - exit" );
		return false;
	}

	this->m_progLightingStageStencilVolume = ProgramManagement::get( "LightingStageStencilVolumeProgramm" );
	if ( 0 == this->m_progLightingStageStencilVolume )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initLightingStage: coulnd't create lighting-stage stencil volume program - exit" );
		return false;
	}

	ZazenGraphics::getInstance().getLogger().logInfo( "Initializing Deferred Rendering Lighting-Stage finished" );

	return true;
}

bool
DRRenderer::initShadowMapping()
{
	ZazenGraphics::getInstance().getLogger().logInfo( "Initializing Deferred Rendering Shadow-Mapping..." );

	this->m_progShadowMappingPlanar = ProgramManagement::get( "ShadowingStagePlanarProgramm" );
	if ( 0 == this->m_progShadowMappingPlanar )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initShadowMapping: coulnd't create planar-shadow program - exit" );
		return false;
	}

	this->m_progShadowMappingCubeSinglePass = ProgramManagement::get( "ShadowingStageCubeSinglePassProgramm" );
	this->m_progShadowMappingCubeMultiPass = ProgramManagement::get( "ShadowingStageCubeMultiPassProgramm" );
	
	if ( NULL == this->m_progShadowMappingCubeSinglePass && NULL == this->m_progShadowMappingCubeMultiPass )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initShadowMapping: no cube-shadow-mapping program found - exit" );
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
	
	this->m_fsq = GeometryFactory::getRef().getNDCQuad();
	if ( NULL == this->m_fsq )
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

void 
DRRenderer::initializeStaticData()
{
	// insertion-order is very important: first x, then y and last z
	this->m_cubeViewDirections.clear();
	this->m_cubeViewDirections.push_back( glm::lookAt( glm::vec3( 0 ), glm::vec3( 1, 0, 0 ), glm::vec3( 0, -1, 0 ) ) );		// POS X
	this->m_cubeViewDirections.push_back( glm::lookAt( glm::vec3( 0 ), glm::vec3( -1, 0, 0 ), glm::vec3( 0, -1, 0 ) ) );	// NEG X
	this->m_cubeViewDirections.push_back( glm::lookAt( glm::vec3( 0 ), glm::vec3( 0, 1, 0 ), glm::vec3( 0, 0, 1 ) ) );		// POS Y
	this->m_cubeViewDirections.push_back( glm::lookAt( glm::vec3( 0 ), glm::vec3( 0, -1, 0 ), glm::vec3( 0, 0, -1 ) ) ); 	// NEG Y
	this->m_cubeViewDirections.push_back( glm::lookAt( glm::vec3( 0 ), glm::vec3( 0, 0, 1 ), glm::vec3( 0, -1, 0 ) ) );		// POS Z 
	this->m_cubeViewDirections.push_back( glm::lookAt( glm::vec3( 0 ), glm::vec3( 0, 0, -1 ),glm::vec3( 0, -1, 0 ) ) );		// NEG Z

	this->m_gBufferDrawBufferIndices.clear();
	this->m_gBufferDrawBufferIndices.push_back( 0 );	// diffuse
	this->m_gBufferDrawBufferIndices.push_back( 1 );	// normal
	this->m_gBufferDrawBufferIndices.push_back( 2 );	// tangents
	this->m_gBufferDrawBufferIndices.push_back( 3 );	// bi-tangents

	this->m_gBufferBindTargetIndices.clear();
	this->m_gBufferBindTargetIndices.push_back( 0 );	// diffuse
	this->m_gBufferBindTargetIndices.push_back( 1 );	// normal
	this->m_gBufferBindTargetIndices.push_back( 2 );	// tangents
	this->m_gBufferBindTargetIndices.push_back( 3 );	// bi-tangents
	this->m_gBufferBindTargetIndices.push_back( 5 );	// depth-map

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
			
			// culling is done outside renderer in special visibility-detection, renderer just issues commands to GPU

			NVTX_RANGE_PUSH( "Frame" );

			// NOTE: there is no GPU work going on thus no RANGE_PUSH
			this->preProcessTransparency( entities );

			NVTX_RANGE_PUSH( "G-Stage" )
			if ( false == this->doGeometryStage( entities ) )
			{
				return false;
			}
			NVTX_RANGE_POP

			NVTX_RANGE_PUSH( "L-Stage" )
			if ( false == this->doLightingStage( entities ) )
			{
				return false;
			}
			NVTX_RANGE_POP

			NVTX_RANGE_PUSH( "T-Stage" )
			if ( false == this->doTransparencyStage( entities ) )
			{
				return false;
			}
			NVTX_RANGE_POP

			NVTX_RANGE_POP

			break;
		}
	}
	
	// NOTE: in release-build gl-errors are detected through GL ARB debug output, see RenderingContext

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
	// render to g-buffer FBO
	if ( false == this->m_fbo->bind() )
	{
		return false;
	}

	// IMPORTANT: need to re-set the viewport for each FBO
	// could have changed due to shadow-map or other rendering happend in the frame before
	this->m_mainCamera->restoreViewport();

	// clear all targets for the new frame
	if ( false == this->m_fbo->clearAll() )
	{
		return false;
	}

	// activate geometry-stage program
	if ( false == this->m_progGeomStage->use() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::doGeometryStage: using program failed - exit" );
		return false;
	}

	// enable rendering to all render-targets in geometry-stage
	if ( false == this->m_fbo->drawBuffers( this->m_gBufferDrawBufferIndices ) )
	{
		return false;
	}

	// check status of FBO, IMPORANT: not before, would have failed
	CHECK_FRAMEBUFFER_DEBUG

	// update all parameters of the camera to render geometry-stage
	this->updateCameraBlock( this->m_mainCamera );

	// enable stencil-test to mark geometry-pixels to distinguish them from background and thus prevent the lighting-shader to run on them
	glEnable( GL_STENCIL_TEST );
	// test always passes and will thus write 1 for each fragment generated by any mesh
	glStencilFunc( GL_ALWAYS, 0x1, 0x1 );
	// replace always so we really get all the geometry marked
	glStencilOp( GL_REPLACE, GL_REPLACE, GL_REPLACE );

	// draw all geometry from cameras viewpoint AND apply materials but ignore transparent material
	if ( false == this->renderEntities( this->m_mainCamera, entities, this->m_progGeomStage, true, false ) )
	{
		return false;
	}

	NVTX_RANGE_PUSH( "Skybox" )
	// render sky-box 
	if ( false == this->renderSkyBox() )
	{
		return false;
	}
	NVTX_RANGE_POP

	// disable stencil-test so not to interfere with lightin-stage (it will handle the stencil-test by itself)
	glDisable( GL_STENCIL_TEST );

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

	// render to target 4 which gathers the final result before post-processing
	if ( false == this->m_fbo->drawBuffer( 4 ) )
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

	// TODO move into skybox
	// NOTE: need to bind cube-map to unit at Texture::CUBE_RANGE_START because unit 0 is already occupied by 2D-textures during light-rendering - 
	// it is not allowed to bind different types of textures to the same unit
	this->m_progSkyBox->setUniformInt( "SkyBoxCubeMap", Texture::CUBE_RANGE_START );

	// stencil-test will pass only when a 0 is found in stencil-buffer - mesh-fragments were marked with 1 in geometry-stage
	glStencilFunc( GL_NOTEQUAL, 0x1, 0x1 );
	// don't touch the stencil-buffer, keep it for all fail-scenarios the same
	glStencilOp( GL_KEEP, GL_KEEP, GL_KEEP );

	// render the geometry
	SkyBox::getRef().render( *this->m_mainCamera, this->m_cameraBlock, this->m_transformsBlock );

	return true;
}

bool
DRRenderer::doLightingStage( std::list<ZazenGraphicsEntity*>& entities )
{
	// start light marker with 2 because meshes increased the stencil-buffer already to 1
	unsigned int lightMarker = 2;

	// render the contribution of each light-entiy to the scene
	std::list<ZazenGraphicsEntity*>::iterator iter = entities.begin();
	while ( iter != entities.end() )
	{
		ZazenGraphicsEntity* entiy = *iter++;
		Light* light = entiy->getLight();
		if ( light )
		{
			if ( false == this->processLight( entities, light, lightMarker ) )
			{
				return false;
			}

			lightMarker++;
		}
	}

	return true;
}

bool
DRRenderer::processLight( std::list<ZazenGraphicsEntity*>& entities, Light* light, unsigned int lightMarker )
{
	// render the shadow-map of the light (if it is a shadow-caster)
	if ( false == this->renderShadowMap( entities, light ) )
	{
		return false;
	}
		
	// IMPORTANT: need to re-set the viewport, could have changed due to shadow-map rendering happend before
	this->m_mainCamera->restoreViewport();

	// light uses stencil-test to discard pixels
	glEnable( GL_STENCIL_TEST );
	// disable writing to depth: light-boundaries should not update depth
	// we also need to READ from depth so no update 
	glDepthMask( GL_FALSE );

	// mark all pixels inside the light-volume of spot- and point-lights
	if ( false == this->markLightVolume( light, lightMarker ) )
	{
		return false;
	}

	// finally render the light
	if ( false == this->renderLight( light, lightMarker ) )
	{
		return false;
	}

	// disable stencil test for next step in pipeline
	glDisable( GL_STENCIL_TEST );
	// enable depth-testing for next step in pipeline
	glDepthMask( GL_TRUE );

	return true;
}

bool
DRRenderer::markLightVolume( Light* light, unsigned int lightMarker )
{
	// NOTE: won't process directional-lights with this as they do a full-screen pass anway
	// thus it would be a waste of this stencil pass
	if ( Light::DIRECTIONAL == light->getType() ) 
	{
		return true;
	}

	// only update stencil-buffer => glDrawBuffer( GL_NONE )
	this->m_fbo->drawNone();

	// check status of FBO, IMPORANT: not before, would have failed
	CHECK_FRAMEBUFFER_DEBUG

	if ( false == this->m_progLightingStageStencilVolume->use() )
	{
		return false;
	}

	float scaleRadius = light->getAttenuation().x;
	glm::mat4 lightBoundingMeshMVP = this->m_mainCamera->getVPMatrix() * light->getModelMatrix() * glm::scale( glm::vec3( scaleRadius, scaleRadius, scaleRadius ) );
	this->m_progLightingStageStencilVolume->setUniformMatrix( "LightBoundingMeshMVP", lightBoundingMeshMVP );

	// no culling of faces because we need to mark the volume by 
	// incrementing/decrementing the stencil-values when entering/exiting
	glDisable( GL_CULL_FACE );

	// set ref to the current lightMarker
    glStencilFuncSeparate( GL_BACK, GL_ALWAYS, lightMarker, 0 );
	// when back-face depth-test fails (e.g. something is in front of the back-face) then this pixel is inside the volume
	// mark this pixel with the current lightmarker
    glStencilOpSeparate( GL_BACK, GL_KEEP, GL_REPLACE, GL_KEEP );
	// set ref to 1 - mesh
	glStencilFuncSeparate( GL_FRONT, GL_ALWAYS, 1, 0 );
	// front-face depth-test failed, the pixel is not inside the volume, mark as normal mesh
    glStencilOpSeparate( GL_FRONT, GL_KEEP, GL_REPLACE, GL_KEEP );

	// render the bounding-mesh of the light
	light->getBoundingMesh()->render();

	// enable culling of faces again
	glEnable( GL_CULL_FACE );

	return true;
}

bool
DRRenderer::renderLight( Light* light, unsigned int lightMarker )
{
	// render to target 4 which gathers the final result before post-processing
	if ( false == this->m_fbo->drawBuffer( 4 ) )
	{
		return false;
	}

	// check status of FBO, IMPORANT: not before, would have failed
	CHECK_FRAMEBUFFER_DEBUG

	// activate lighting-stage shader
	if ( false == this->m_progLightingStage->use() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::renderLight: using program failed - exit" );
		return false;
	}

	// OPTIMIZE: no need to do for every light! try it
	// lighting stage program need all buffers of g-buffer bound as textures
	if ( false == this->m_fbo->bindTargets( this->m_gBufferBindTargetIndices ) )
	{
		return false;
	}

	// tell lighting program that diffusemap is bound to texture-unit 0
	this->m_progLightingStage->setUniformInt( "DiffuseMap", 0 );
	// tell lighting program that normalmap is bound to texture-unit 1
	this->m_progLightingStage->setUniformInt( "NormalMap", 1 );
	// tell lighting program that tangents-map of scene is bound to texture-unit 2
	this->m_progLightingStage->setUniformInt( "TangentMap", 2 );
	// tell lighting program that bi-tangents-map of scene is bound to texture-unit 3 
	this->m_progLightingStage->setUniformInt( "BiTangentMap", 3 );
	// tell lighting program that depth-map of scene is bound to texture-unit 4 
	this->m_progLightingStage->setUniformInt( "DepthMap", 4 );

	// activate the corresponding subroutines for the light-type
	if ( Light::LightType::DIRECTIONAL == light->getType() )
	{
		this->m_progLightingStage->activateSubroutine( "directionalLight", Shader::FRAGMENT_SHADER );
	}
	else if ( Light::LightType::SPOT == light->getType() )
	{
		this->m_progLightingStage->activateSubroutine( "spotLight", Shader::FRAGMENT_SHADER );
	}
	else if ( Light::LightType::POINT == light->getType() )
	{
		this->m_progLightingStage->activateSubroutine( "pointLight", Shader::FRAGMENT_SHADER );
	}

	// bind shadow-map when light is shadow-caster
	if ( light->isShadowCaster() )
	{
		// WARNING: you can't bind two different texture-types to the same unit in the same program - will result in INVALID OPERATION on draw-call
		// thus we need to bind the shadow cube-map to a different unit than the normal 2d shadow-map
		int textureUnit = 7;

		if ( Light::LightType::POINT == light->getType() )
		{
			textureUnit = Texture::CUBE_RANGE_START;

			// tell program that the shadowmap of point-light will be available at texture unit Texture::CUBE_RANGE_START
			this->m_progLightingStage->setUniformInt( "ShadowCubeMap", textureUnit );

			this->m_progLightingStage->activateSubroutine( "cubeShadow", Shader::FRAGMENT_SHADER );
		}
		else
		{
			// tell program that the shadowmap of spot/directional-light will be available at texture unit 7
			this->m_progLightingStage->setUniformInt( "ShadowPlanarMap", textureUnit );

			if ( Light::LightType::DIRECTIONAL == light->getType() )
			{
				this->m_progLightingStage->activateSubroutine( "directionalShadow", Shader::FRAGMENT_SHADER );
			}
			else if ( Light::LightType::SPOT == light->getType() )
			{
				this->m_progLightingStage->activateSubroutine( "projectiveShadow", Shader::FRAGMENT_SHADER );
			}
		}

		if ( false == light->getShadowMap()->bind( textureUnit ) )
		{
			return false;
		}
	}
	else
	{
		this->m_progLightingStage->activateSubroutine( "noShadow", Shader::FRAGMENT_SHADER );
	}

	// update configuration of the current light to its uniform-block
	if ( false == this->updateLightBlock( light, this->m_mainCamera ) )
	{
		return false;
	}

	// need to re-set the configuration of the camera uniform-block to the main-camera because need the information within the lighting-shader
	if ( false == this->updateCameraBlock( this->m_mainCamera ) )
	{
		return false;
	}

	// blend lighting-results, defaults to GL_FUNC_ADD, exactly what we need
	glEnable( GL_BLEND );
	// the source (from lighting-shader) is weighted by the alpha-value (constant 1.0 for now)
	// the destination is weighted by the inverse of the source-color which creates a saturation effect with multiple lights
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_COLOR );

	// need a different stencil-func for directional-lights because we don't render their light-volume as a FSQ doesn't have
	// one and it would make no sense as they reach every mesh anyway thus it would be a waste
	if ( Light::DIRECTIONAL == light->getType() ) 
	{
		// stencil-test will pass if stencil value is not 0 => directional light affects all mesh-pixels regardless if they
		// are affected by other lights already or not
		glStencilFunc( GL_NOTEQUAL, 0, 0xFF );
	}
	// point- and spot-lights render their light-volume and will write the lightMarker to the stencil-buffer
	// this is necessary because in the final light-pass we render a FSQ (see below)
	else
	{
		// pass only for pixels marked with the lightMarker value, which stays constant for one light and gets incremented
		// thus each light has a unique lightmarker. this is necessary because otherwise the FSQ pass would affect
		// pixels which belong to other lights too which would waste part of the optimization 
		glStencilFunc( GL_LEQUAL, lightMarker, 0xFF );
	}

	// don't touch the stencil-buffer, keep it for all fail-scenarios the same
	glStencilOp( GL_KEEP, GL_KEEP, GL_KEEP );

	// now render the FSQ
	// NOTE: we don't render the light bounding-mesh because for two reasons
	// 1. the mesh could have quite high complexity 
	// 2. with the FSQ we don't have to distinguish if we are inside the light bounding-mesh or outside
	//		based upon this we would have to change glCullFace to FRONT or BACK
	this->m_fsq->render();
	
	// turn off blending otherwise would lead to artifacts because everything would be blended (depth maps,...)
	glDisable( GL_BLEND );

	return true;
}

bool
DRRenderer::renderShadowMap( std::list<ZazenGraphicsEntity*>& entities, Light* light )
{
	// check if this light is a shadow-caster
	if ( false == light->isShadowCaster() )
	{
		return true;
	}

	NVTX_RANGE_PUSH( "ShadowMapping" )

	// update the camera for rendering the shadow-cube - the light is the camera because we render the scene from the perspective of the camera
	this->updateCameraBlock( light );

	// use shadow-mapping fbo to render depth of light view-point to
	if ( false == this->m_intermediateDepthFB->bind() )
	{
		return false;
	}

	// point-light has a cube-shadowmap
	if ( Light::LightType::POINT == light->getType() )
	{
		if ( false == this->renderShadowCube( entities, light ) )
		{
			return false;
		}
	}
	// directional- and spot-light have a planar (2D) shadow-map 
	else
	{
		if ( false == this->renderShadowPlanar( entities, light ) )
		{
			return false;
		}
	}

	if ( false == this->m_fbo->bind() )
	{
		return false;
	}

	NVTX_RANGE_POP

	return true;
}

bool
DRRenderer::renderShadowPlanar( std::list<ZazenGraphicsEntity*>& entities, Light* light )
{
	if ( false == this->m_intermediateDepthFB->attachTargetTemp( light->getShadowMap() ) )
	{
		return false;
	}

	if ( false == this->m_progShadowMappingPlanar->use() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::renderShadowMap: using planar program failed - exit" );
		return false;
	}

	if ( false == this->renderShadowPass( entities, light, this->m_progShadowMappingPlanar ) )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::renderShadowCube( std::list<ZazenGraphicsEntity*>& entities, Light* light )
{
	// single-pass program for shadow-mapping is in use
	// uses geometry-shader to render to layers of the cube-map
	if ( this->m_progShadowMappingCubeSinglePass )
	{
		if ( false == this->renderShadowCubeSinglePass( entities, light ) )
		{
			return false;
		}
	}
	// multi-pass program for shadow-mapping is in use
	// do a pass for each cube-layer
	else
	{
		if ( false == this->renderShadowCubeMultiPass( entities, light ) )
		{
			return false;
		}
	}

	return true;
}

bool
DRRenderer::renderShadowCubeSinglePass( std::list<ZazenGraphicsEntity*>& entities, Light* light )
{
	std::vector<glm::mat4> cubeMVPTransforms( 6 );
	glm::vec3 lightPosWorld = light->getPosition();
	glm::mat4 invLightPosTransf = glm::translate( -lightPosWorld );
		
	// calculate model-view-projection matrices for each cube-face
	for ( unsigned int i = 0; i < 6; ++i )
	{
		cubeMVPTransforms[ i ] = light->getProjMatrix() * this->m_cubeViewDirections[ i ] * invLightPosTransf;
	}

	if ( false == this->m_progShadowMappingCubeSinglePass->use() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::renderShadowCubeSinglePass: using cube single-pass program failed - exit" );
		return false;
	}

	// upload model-view-projection transformations for each face (used in geometry-shader)
	this->m_progShadowMappingCubeSinglePass->setUniformMatrices( "u_cubeMVPTransforms[0]", cubeMVPTransforms );

	// attach cube-map faces to frame-buffer object
	if ( false == this->m_intermediateDepthFB->attachTargetTemp( light->getShadowMap() ) )
	{
		return false;
	}

	// do single shadow-pass - geometry-shader will render to each layer of the cube-map
	if ( false == this->renderShadowPass( entities, light, this->m_progShadowMappingCubeSinglePass ) )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::renderShadowCubeMultiPass( std::list<ZazenGraphicsEntity*>& entities, Light* light )
{
	std::vector<glm::mat4> cubeViewTransforms( 6 );
	glm::vec3 lightPosWorld = light->getPosition();
	glm::mat4 invLightPosTransf = glm::translate( -lightPosWorld );
		
	// calculate model-view matrices for each cube-face - leave out projection, will be applied in shader!
	for ( unsigned int i = 0; i < 6; ++i )
	{
		cubeViewTransforms[ i ] = this->m_cubeViewDirections[ i ] * invLightPosTransf;
	}

	if ( false == this->m_progShadowMappingCubeMultiPass->use() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::renderShadowCubeMultiPass: using cube multi-pass program failed - exit" );
		return false;
	}

	// do 6 passes, one for each cube-map face
	for ( unsigned int face = 0; face < 6; ++face )
	{
		if ( false == this->m_cameraBlock->bindBuffer() )
		{
			return false;
		}

		// update viewing-matrix for the according cube-face
		this->m_cameraBlock->updateField( "CameraUniforms.viewMatrix", cubeViewTransforms[ face ] );

		// attach cube-map faces to frame-buffer object
		if ( false == this->m_intermediateDepthFB->attachTargetTempCubeFace( light->getShadowMap(), face ) )
		{
			return false;
		}

		if ( false == this->renderShadowPass( entities, light, this->m_progShadowMappingCubeMultiPass ) )
		{
			return false;
		}
	}

	return true;
}

bool
DRRenderer::renderShadowPass( list<ZazenGraphicsEntity*>& entities, Light* light, Program* currentShadowProgram )
{
	// check status now
	// IMPORANT: don't check too early
	CHECK_FRAMEBUFFER_DEBUG

	// clear bound buffer
	glClear( GL_DEPTH_BUFFER_BIT );

	// IMPORTANT: need to set the viewport for each shadow-map, because resolution can be different for each
	light->restoreViewport();

	// render scene from view of camera - don't apply material, we need only depth, render transparent materials
	if ( false == this->renderEntities( light, entities, currentShadowProgram, false, true ) )
	{
		return false;
	}

	// render scene from view of camera - don't apply material, we need only depth, render opaque materials
	if ( false == this->renderEntities( light, entities, currentShadowProgram, false, false ) )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::doTransparencyStage( std::list<ZazenGraphicsEntity*>& entities )
{
	// when no transparent entities are present, just blit the final gathering to the system framebuffer
	if ( 0 == this->m_transparentEntities.size() )
	{
		// final light-gather is accumulated in render-target with index 4
		this->m_fbo->blitToSystemFB( 4 );
	}
	// if transparent entities present, ping-pong render them
	else
	{
		unsigned int combinationTarget = 1;
		unsigned int backgroundIndex = 4;

		this->m_intermediateDepthFB->bind();
		// could have been changed bevore during shadow-map rendering
		this->m_intermediateDepthFB->restoreDepthTarget();

		CHECK_FRAMEBUFFER_DEBUG

		this->m_fbo->bind();

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
	}

	return true;
}

bool
DRRenderer::renderTransparentInstance( ZazenGraphicsEntity* entity, unsigned int backgroundIndex, unsigned int combinationTarget, bool lastInstance )
{
	NVTX_RANGE_PUSH( "T Render" );

	if ( false == this->m_progTransparency->use() )
	{
		return false;
	}

	// bind background to index 2 because DiffuseColor of Material is at index 0 and NormalMap of Material is at index 1
	this->m_fbo->getAttachedTargets()[ backgroundIndex ]->bind( 2 );
	// bind depth of intermediate depth-FBO to target 3 - will act as background-depth
	this->m_intermediateDepthFB->getAttachedDepthTarget()->bind( 3 );

	this->m_progTransparency->setUniformInt( "Background", 2 );
	this->m_progTransparency->setUniformInt( "BackgroundDepth", 3 );
	
	// copy depth of g-buffer to intermediate depth-fbo, because we need to access the depth in transparency-rendering
	// to prevent artifacts. at the same time we write depth when rendering transparency so we cannot bind the g-buffer depth => need to copy
	this->m_fbo->blitDepthToFBO( this->m_intermediateDepthFB );

	// transparent objects are always rendered intermediate to g-buffer color target 0
	this->m_fbo->drawBuffer( 0 );

	// clear buffer
	// need to set alpha-clearing to 1.0 because was set to 0.0 during geometry-stage to mark bits as background/sky-box material (alpha-channel of diffuse-texture stores material-id)
	// need to set alpha-channel to 1.0 now because will mark the background-pixels
	glClearColor( 0.0, 0.0, 0.0, 1.0 );
	glClear( GL_COLOR_BUFFER_BIT );

	if ( false == this->updateCameraBlock( this->m_mainCamera ) )
	{
		return false;
	}

	if ( false == this->renderTransparentEntity( this->m_mainCamera, entity, this->m_progTransparency ) )
	{
		return false;
	}

	NVTX_RANGE_POP

	NVTX_RANGE_PUSH( "T Blend" );

	// after rendering last instance we will combine to default-framebuffer for final result
	if ( lastInstance )
	{
		// back to default-framebuffer
		this->m_fbo->unbind();

		// clear default-framebuffer color & depth
		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	}
	else 
	{
		// draw combination to new target (ping-pong rendering)
		this->m_fbo->drawBuffer( combinationTarget );

		glClear( GL_COLOR_BUFFER_BIT );
	}


	if ( false == this->m_progBlendTransparency->use() )
	{
		return false;
	}

	// transparent object was rendered to g-buffer color target 0
	this->m_fbo->getAttachedTargets()[ 0 ]->bind( 0 );

	this->m_progBlendTransparency->setUniformInt( "Background", 2 );
	this->m_progBlendTransparency->setUniformInt( "Transparent", 0 );
	this->m_progBlendTransparency->setUniformMatrix( "projectionMatrix", this->m_mainCamera->createOrthoProj( true, true ) );

	// disable writing to depth when not last instance because would destroy our depth-buffer
	// when last instance it doesnt matter because we render to screen-buffer
	if ( ! lastInstance )
	{
		glDepthMask( GL_FALSE );
	}

	this->m_fsq->render();

	// enable depth-writing again
	if ( ! lastInstance )
	{
		glDepthMask( GL_TRUE );
	}

	NVTX_RANGE_POP

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

	list<ZazenGraphicsEntity*>::iterator iter = entities.begin();
	while ( iter != entities.end() )
	{
		ZazenGraphicsEntity* entity = *iter++;
		Material* material = entity->getMaterial();
		Animation* animation = entity->getAnimation();

		if ( 0 == entity->getMeshNode() )
		{
			continue;
		}

		if ( animation )
		{
			// upload bones for animation
			currentProgramm->setUniformMatrices( "u_bones[0]", animation->getBoneTransforms() );
			currentProgramm->activateSubroutine( "processInputsAnimated", Shader::VERTEX_SHADER );
		}
		else
		{
			// switch subroutine in vertex-shader of geometry-stage because need skinning for animated instances
			currentProgramm->activateSubroutine( "processInputsStatic", Shader::VERTEX_SHADER );
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

		// calculate model-view and mvp matrix once for this entity, hierarchical node-transforms are applied during mesh-rendering
		glm::mat4 modelView = viewer->getViewMatrix() * entity->getModelMatrix();
		glm::mat4 modelViewProj = viewer->getVPMatrix() * entity->getModelMatrix();

		// render geometry of this instance
		if ( false == this->renderMeshNode( entity->getMeshNode(), modelViewProj, modelView, entity->getModelMatrix() ) )
		{
			return false;
		}
	}

	return true;
}

bool
DRRenderer::renderTransparentEntity( Viewer* viewer, ZazenGraphicsEntity* entity, Program* currentProgramm )
{
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
		
	// calculate model-view and mvp matrix once for this entity, hierarchical node-transforms are applied during mesh-rendering
	glm::mat4 modelView = viewer->getViewMatrix() * entity->getModelMatrix();
	glm::mat4 modelViewProj = viewer->getVPMatrix() * entity->getModelMatrix();

	// render geometry of this instance
	if ( false == this->renderMeshNode( entity->getMeshNode(), modelViewProj, modelView, entity->getModelMatrix() ) )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::renderMeshNode( MeshNode* meshNode, const glm::mat4& entityMVPMatrix, const glm::mat4& entityMVMatrix, const glm::mat4& entitiyModelMatrix )
{
	// no meshes in this node or any sub-children, nothing to render
	// because we pre-calculate the global-transformation for each mesh-node during loading
	// we can ommit non-meshed nodes and don't need to walk them just because of re-calculating
	// the (always static) hierarchical global-transformations
	if ( ! meshNode->hasMeshes() )
	{
		return true;
	}

	const std::vector<Mesh*>& meshes = meshNode->getMeshes();
	const std::vector<MeshNode*>& children = meshNode->getChildren();
	glm::mat4 localMVPMatrix = entityMVPMatrix * meshNode->getGlobalTransform();
	glm::mat4 localMVMatrix = entityMVMatrix * meshNode->getGlobalTransform();
	glm::mat4 localModelMatrix = entitiyModelMatrix * meshNode->getGlobalTransform();

	for ( unsigned int i = 0; i < children.size(); i++ )
	{
		MeshNode* child = children[ i ];
		if ( false == this->renderMeshNode( children[ i ], entityMVPMatrix, entityMVMatrix, entitiyModelMatrix ) )
		{
			return false;
		}
	}

	for ( unsigned int i = 0; i < meshes.size(); i++ )
	{
		Mesh* mesh = meshes[ i ];

		// update model matrix
		this->m_transformsBlock->updateField( "TransformUniforms.modelMatrix", localModelMatrix );
		// update model-view matrix
		this->m_transformsBlock->updateField( "TransformUniforms.modelViewMatrix", localMVMatrix );
		// update model-view-projection matrix
		this->m_transformsBlock->updateField( "TransformUniforms.modelViewProjMatrix", localMVPMatrix );
		
		// THIS EXISTS FOR LEARNING AND INFORMATION REATIONS!
		// IMPORTANT: normal-vectors are transformed different than vertices
		// take the transpose of the inverse modelView or simply reset the translation vector in the modelview-matrix
		// in other words: only the rotations are applied to normals and they are guaranteed to leave
		// normalized normals at unit length. THIS METHOD WORKS ALSO WHEN NO NON UNIFORM SCALING IS APPLIED
		// TODO: scrap this transform because we forbid non-uniform-scaling in our engine for performance reasons
		// glm::mat4 normalModelViewMatrix = glm::transpose( glm::inverse( localModelViewMatrix ) );
		// update model-view matrix for normals
		// this->m_transformsBlock->updateField( "TransformUniforms.normalsModelViewMatrix", normalModelViewMatrix );

		// render geometry
		return mesh->render();
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

bool
DRRenderer::updateCameraBlock( Viewer* viewer )
{
	glm::vec2 cameraRectangle;
	glm::vec2 cameraNearFar;
	glm::vec2 cameraFrustum;

	cameraRectangle[ 0 ] = ( float ) viewer->getWidth();
	cameraRectangle[ 1 ] = ( float ) viewer->getHeight();

	cameraNearFar[ 0 ] = viewer->getNear();
	cameraNearFar[ 1 ] = viewer->getFar();

	cameraFrustum[ 0 ] = viewer->getRightFrustum();
	cameraFrustum[ 1 ] = viewer->getTopFrustum();

	// bind camera uniform-block to update data
	if ( false == this->m_cameraBlock->bindBuffer() )
	{
		return false;
	}

	// upload cameras window size
	this->m_cameraBlock->updateField( "CameraUniforms.window", cameraRectangle );
	// upload cameras near and far distances
	this->m_cameraBlock->updateField( "CameraUniforms.nearFar", cameraNearFar );
	// upload cameras right and top frustum (symetric!)
	this->m_cameraBlock->updateField( "CameraUniforms.frustum", cameraFrustum );

	// upload world-orientation of camera ( its model-matrix )
	this->m_cameraBlock->updateField( "CameraUniforms.modelMatrix", viewer->getModelMatrix() );
	// upload view-matrix of camera ( = its inverse model-matrix)
	this->m_cameraBlock->updateField( "CameraUniforms.viewMatrix", viewer->getViewMatrix() );
	// upload projection-matrix of the camera
	this->m_cameraBlock->updateField( "CameraUniforms.projectionMatrix", viewer->getProjMatrix() );
	
	return true;
}

bool
DRRenderer::updateLightBlock( Light* light, Viewer* camera )
{
	glm::vec2 shadowResolution;
	const glm::vec3& color = light->getColor();
	const glm::vec2& specular = light->getSpecular();
	const glm::vec3& attenuation = light->getAttenuation();
	glm::mat4 lightMVMatrix = camera->getViewMatrix() * light->getModelMatrix();

	// shadow-map resolution corresponds to the width and height
	shadowResolution.x = ( float ) light->getWidth();
	shadowResolution.y = ( float )  light->getHeight();
	
	// bind light uniform-block to update data of this light
	if ( false == this->m_lightBlock->bindBuffer() )
	{
		return false;
	}

	this->m_lightBlock->updateField( "LightUniforms.shadowRes", shadowResolution );
	this->m_lightBlock->updateField( "LightUniforms.color", color );
	this->m_lightBlock->updateField( "LightUniforms.specular", specular );
	this->m_lightBlock->updateField( "LightUniforms.modelViewMatrix", lightMVMatrix );

	// upload light-model matrix = orientation of the light in the world
	this->m_lightBlock->updateField( "LightUniforms.modelMatrix", light->getModelMatrix() );

	if ( Light::LightType::POINT == light->getType() )
	{
		glm::vec2 nearFar;
		nearFar.x = light->getNear();
		nearFar.y = light->getFar();

		this->m_lightBlock->updateField( "LightUniforms.nearFar", nearFar );
		this->m_lightBlock->updateField( "LightUniforms.attenuation", attenuation );
	}
	else
	{
		if ( Light::LightType::SPOT == light->getType() )
		{
			glm::vec2 spot( 1.0 );
			spot.x = ( float ) cos( light->getFov() / 2.0 );
			spot.y = light->getAttenuation().z;

			this->m_lightBlock->updateField( "LightUniforms.attenuation", attenuation );
			this->m_lightBlock->updateField( "LightUniforms.spot", spot );
		}

		// update unit-cube matrix only when its NOT a point light AND shadow-caster
		// this unit-cube matrix is only needed in case of planar shadow rendering
		if ( light->isShadowCaster() )
		{
			glm::mat4 lightSpaceUnit = this->m_unitCubeMatrix * light->getVPMatrix();
			this->m_lightBlock->updateField( "LightUniforms.spaceUniformMatrix", lightSpaceUnit );
		}
	}

	return true;
}
