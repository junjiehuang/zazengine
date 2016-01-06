The renderer is implemented using the deferred rendering technique. See docs/papers/rendering/deferred rendering for much information about this technique.

For the G-Buffer four RGBA8 Color-Attachments and one 32Bit Floating Depth-Attachment are used to deliver the necessary properties to the Lighting-Stage.

**Color-Attachment layouts**
  * 0.DiffuseR, DiffuseG, DiffuseB, MaterialType
  * 1.NormalX, NormalY, NormalZ, Specular-Component
  * 2.Generic
  * 3.Generic

## Material System ##
Each material uses a special BRDF-model to simulate the lighting reflection.
Each material can specify a RGB triple as base-color ( can be used as single color without texturing over the whole object or as an ambient term when applying textures).
Each material can have applied none, one or up to four diffuse-color textures.
Each material can have a normal-map. When a normal-map is specified this will override the normals from the object.
Each material can have a specular.map. Some BRDFs will ignore this.
Transparent material is available but not implemented in the deferred rendering pass but in a separate one. Seee lighting & shadowing.

The material-type is also the BRDF-type. The following BRDFs are supported:
  * DIFFUSE – won't apply any lighting-calculations and use just the diffuse color. Looks shit and is very very cheap.
  * LAMBERTIAN – use the lambertian law to apply lighting. Looks better and is well suited for diffuse material which needs to have lighting applied. Very cheap
  * PHONG – uses the phong-model to apply lighting. Introduces specular highlights onto lambertian base color. Used for plastic or material with specular effects. Cheap.
  * ORENNAYAR – a more phyiscally plausible retro-diffuse BRDF than lambertian. Expensive.
  * SSS – does an approximation of subsurface scattering which can be used to render jade or porcelain. Very expensive.
  * WARDS – an implementation of wards brdf-model. Can simulate a huge amount of materials when the correct parameters are used. Very expensive.
  * TORRANCESPARROW – an implementation of the torrance-sparrow BRDF which uses a microfacet model. Can simulate a huge amount of materials when the correct parameters are used. Expensive.

Materialproperties of an object which change over the object need to be encoded in a texture e.g. normalmaps, diffuse color, specular component. Properties which stay constant for an entire object can be passed through the material uniform-block to the geometry-stage.

**Material-Definition examples**
Some woody floor. It's lambertian because no specular highligh needed and wood is diffuse but we want the lambertian light-model applied. No base-color specified because using textures as diffuse color.
```
<material name='WoodFloor'>
	<type='LAMBERTIAN'/>
		<diffuseColor1 = 'path/woodFloorBase.tga'/>
		<diffuseColor2 = 'path/woodFloorFine.tga'/>
		<normalMap = 'path/woodFloorNormals.tga'/>
	</type>
</material>
```

Some stone floor. It's phong because it has some specular sparkles in it. No base color specified because using textures as diffuse color.
```
<material name='StoneFloor'>
	<type='PHONG'/>
		<diffuseColor1 = 'path/stoneFloorBase.tga'/>
		<diffuseColor2 = 'path/stoneFloorFine.tga'/>
		<diffuseColor3 = 'path/stoneFloorGrain.tga'/>
		<specularMap = 'path/stoneFloorSpecularMap.tga'/>
		<normalMap = 'path/stoneFloorNormals.tga'/>
	</type>
</material>
```

Transparent Glass.
```
<material name='ChurchGlass'>
	<type='TRANSPARENT'/>
		<diffuseColor1 = 'path/churchWindowColor.tga'/>
		<normalMap = 'path/churchWindowNormals.tga'/>
	</type>
</material>
```

Phong BRDF. Doesn't use any texture but just a base-color.
```
<material name='BluePlastic'>
	<type='PHONG'/>
		<color r='0.0' g='0.0' b='1.0'/>

		TODO: add parameters of this BRDF
	</type>
</material>
```

Oren-Nayar BRDF
```
<material name=“MatePaper“>
	<type=“ORENNAYAR“/>
		TODO: add parameters of this BRDF
	</type>
</material>
```

Subsurface Scattering Approximation BRDF
```
<material name=“Jade“>
	<type=“SSS“/>
		TODO: add parameters of this BRDF
	</type>
</material>
```

Wards BRDF
```
<material name=“BrushedMetal“>
	<type=“WARDS“/>
		TODO: add parameters of this BRDF
	</type>
</material>
```

Torrance-Sparrow Microfacet BRDF
```
<material name=“SnowMetal“>
	<type=“TORRANCESPARROW“/>
		TODO: add parameters of this BRDF
	</type>
</material>
```

The following Material-Types are defined (transparency not rendered with this mechanism)
  * DIFFUSE			0
  * LAMBERTIAN		1
  * PHONG			2
  * ORENNAYAR		3
  * SSS				4
  * WARDS			5
  * TORRANCESPARROW		6
  * TRANSPARENT		99 (special-case, won't be handled with deferred rendering directly)

The geometry-stage program could look like:
Fragment-Shader

```
#define MAX_DIFFUSE_TEXT 4

uniform block
{
	vec4 materialConfig; // x=materialtype, y=diffuseTextureCounter, z=normal map 0/1, w=specular map0/1
	vec4 genericMaterialAttrib1;	// written to ColorAttachment 3
	vec4 genericMaterialAttrib2;	// written to ColorAttachment 4

	vec4 materialColor;	// base-color of material

	sampler diffuseTextures[MAX_DIFFUSE_TEXT];
	sampler normalMap;
	sampler specularMap;
}

in vec2 ex_textureCoord;
in vec3 ex_normal;

out vec4 out_diffuse;
out vec4 out_normal;
out vec4 out_generic1;
out vec4 out_generic2;

void main()
{	
	// store materialtype in diffuse-component alpha-channel
	out_diffuse.a = materialConfig.x;

	// store base-color of material
	out_diffuse.rgb = materialColor.rgb;

	// multitexturing sums up each texture fetch
	// the number of textures for this material is stored in
	// materialConfig.y - component
	for ( int i = 0; i < materialConfig.y; i++ )
	{
		out_diffuse.rgb += texture( diffuseTextures[i], ex_textureCoord ).rgb;
	}

	// normal-mapping enabled – fetch from texture
	if ( 1.0 == materialConfig.z )
	{
		out_normal.xyz = texture( normalMap, ex_textureCoord ).xyz;
	}
	else
	{
		out_normal.xyz = ex_normal.xyz;
	}

	// set alpha component of normal o 0
	out_normal.a = 0.0;

	// specular-mapping enabled – fetch from texture and store
	// in normal alpha-component
	// specular-map should be a 1channel 8bit luminance map with 	// values between 0 – 255
	if ( 1.0 == materialConfig.w )
	{
		out_normal.a = texture( ex_textureCoord ).r;
	}

	out_generic1 = genericMaterialAttrib1;
	out_generic2 = genericMaterialAttrib2;
}
```

It is obvious that in this shader no distinction between the material type is done – no dynamic branching like if ( 0 == materialConfig.x ). This is because the configurations are set in a way that this is not necessary in the geometry-stage.
Transparency is completely ignored in the geometry-buffer because deferred rendering is inherent unable to render transparency directly. For this purpose an additional non-deferred rendering stage needs to be applied after the lighting-stage. See lighting&shadowing for discussion of this stage.

**Lighting & Shadowing**
The renderer should support a theoretically unlimited number of shadow-casting lights of the following category.
  * Spotlight - perspective planar Shadow-Map
  * Directional-Light – orthographic planar Shadow-Map
  * Pointlight – perspective cube Shadow-Map

Each light can be configured individually.
  * Shadow-Caster YES/NO
  * Falloff – Spotlight & Pointlight only
  * FieldOfView – Spotlight only (Pointlight is 90° in all 6 directions both shadow & light)
  * Color

The FBO of the geometry-stage is bound during lighting-stage too to have depth-values at hand which become useful for later usage when working with light-boundaries.
The lighting-stage renders into an additional Color-Attachment4 because we then need to apply transparency and though need the lighting-stage composition in a texture available.

Fragment-Shader of Light-Stage
```
uniform sampler diffuseMap;
uniform sampler normalMap;
uniform sampler depthMap;
uniform sampler genericMap1;
uniform sampler genericMap2;

uniform shadowSampler shadowMap;
uniform shadowSamplerCube shadowCubeMap;

out vec4 final_color;

uniform block light_data
{
	vec4 lightConfig; // x: type, y: falloff, z: shadowCaster 0/1
	vec4 lightPosition;		// spot & point only
	vec4 lightDirection; 	// spot & directional only
	vec4 color;

	mat4 lightSpace;
	mat4 lightSpaceUniform;
}

void main()
{
	// fetch the coordinate of this fragment in normalized
	// screen-space ( 0 – 1 ) 
	vec2 screenCoord = ...;

	vec4 diffuse = texture( diffuseMap, screenCoord );
	vec4 normal = texture( normalMap, screenCoord );
	// stored as luminance floating point 32bit
	vec4 depth = texture( depthMap, screenCoord ).x;
	vec4 generic1 = texture( genericMap1, screenCoord );
	vec4 generic2 = texture( genericMap2, screenCoord );
	
	float shadow = 0.0f;

	// light is not a point light
	if ( 2 != lightConfig.x )
	{
		// get the world-coordinate of this fragment
		vec4 worldCoord = ...; // apply inverse projection-matrix and undo projection
		// transform the worldCoord into model-coordinates
		vec4 modelCoord = ...; // apply inverse view-matrix
		// get the fragment in the light-space
		vec4 lightCoord = ...; // apply the lightSpace-Matrix to the modelCoord
		// transform lightcoord to fit from NDC (lightSpace matrix includes viewing-projection) into the unit-cube 0-1 to be able to access the shadow-map
		vec4 shadowCoord = ...; // apply the unit-cube matrix

		// spot-light is projective – shadowlookup must be projective
		if ( 0 == lightConfig.x )
		{
			shadow = textureProj( shadowMap, shadowcoord );
		}
		// directional-light is orthographic – no projective shadowlookup
		else
		{
			shadow = texture( shadowMap, shadowcoord );
		}
	}
	// point-light shadow is handled with cube map
	else
	{
		// cube-map access is done through the world-space normal
		vec3 worldSpaceNormal = ...;
		shadow = texture( shadowCubeMap, worldSpaceNormal );
	}

	// this fragment is not in shadow, only then apply lighting
	if ( shadow == 0.0 )
	{
		if ( 0 == diffuse.a )
			renderDiffuseBRDF();
		else if ( 1 == diffuse.a )
			 renderLambertianBRDF();
		else if ( 2 == diffuse.a )
			 renderPhongBRDF();
		else if ( 3 == diffuse.a )
			 renderOrenNayarBRDF();
		else if ( 4 == diffuse.a )
			 renderSSSBRDF();
		else if ( 5 == diffuse.a )
			 renderWardsBRDF();
		else if ( 6 == diffuse.a )
			 renderMicrofacetBRDF();
		end if
	}
	else
	{
		// TODO: soft-shadows

		// when in shadow, only the diffuse color is used for
		// the final color output but darkened by its half
		final_color.rgb = diffuse.rgb * 0.5;
		final_color.a = 1.0;
	}
}

void renderDiffuseBRDF()
{
}

void renderLambertianBRDF()
{
}

void renderPhongBRDF()
{
}

void renderOrenNayarBRDF()
{
}

void renderSSSBRDF()
{
}

void renderWardsBRDF()
{
}

void renderMicrofacetBRDF()
{
}
```

**Transparency-rendering**
I've been working for about 1.5 months on a deferred renderer and now i'm making up my mind about transparency. As we all know it is not possible to directly render transparency with deferred shading and so it must be applied in a post-deferred-shading stage.

What i want to achieve is:
  * layerd transparency (correct transparency behind other transparency)
  * correct depth-visibility
  * normal-maps for transparent objects

The depth-buffer is needed for correct visibility but Z-Buffer doesn't work with transparency because it doesn't care about the alpha value but with deferred rendering we have a big advantage despise the bad news mentioned bevore.
At this point of the rendering-pipeline we have the background already at hand and can access it like a texture, so we can use the depth-buffer very conveniently for correct depth-visibility and then just need to access the background-texture for transparency effects.

The basic approach is to use the output of the lighting-stage which has been stored in a render-target as background-texture. The fragment-shader for transparent rendering does a texture-lookup into this background-texture and maybe perturb the texture-coordinates by using the normals or a normalmap. For correct depth-visibility the depth-buffer target of the geometry-stage can be used.

Things get tricky when one wants to achieve correct layered transparency ( transparency behind transparency ).

My solution is to accumulate the transparencies in two separate render-targets. We need two render-targets because we need to read from one and write the new transparency-accumulation into the other one.
These render-targets don't have to be created additionally but can be recycled e.g. the diffuse-color and normal-map render-targets of the geometry-stage.

The steps are then
  1. ensure that the FBO of the geometry-stage is binded with its color-attachments and depth-buffer.
  1. clear colorattachment0 and colorattachment1
  1. bind color-attachment4 ( output of the lighting-stage ) to texture unit 0
  1. bind color-attachment0 (diffuse-color of geometry-stage) to texture unit 1
  1. set the color-attachment1 (normals of the geometry-stage) of the FBO as the target to which we want to render
  1. render the first transparent object with the program described below
  1. now only the transparent object is written to color-attachment1.
  1. now bind color-attachment1 to texture unit 1 and set color-attachment0 as the target to which we want to render - the targets have been flipped: color0 was previously read from and is now being written to, color1 was previously written to and is now read from.
  1. render the next transparent object
  1. flip the color-attachments again
  1. render the next transparent object
  1. and so on...

Fragment-shader for rendering transparency
```
uniform sampler lStageBg;
uniform sampler transpAcc;

uniform sampler normalMap;
uniform sampler diffuseColor;

in vec2 ex_screenSpaceTexCoord;
out vec4 out_color;

void main()
{
    // fetch normal from normalmap
    vec3 normal = texture( normalMap, ex_screenSpaceTexCoord );

    // perturb texture coordinate
    vec2 texCoord = ex_screenSpaceTexCoord.xy + normal.xy;
    // access the lighting-stage result as background
    vec3 lStageBgColor = texture( lStageBg, texCoord );
    // access transparency accumulation
    vec3 transpAccColor = texture( transpAcc, texCoord );

    // a is 1 when another transparency is already accumulated at this fragment, 
    // in this case lStageBgColor will not be added because of multiplication by 0 but the transparency accumulated will be used
    // a is 0 when no transparency has been accumulated at this fragment, 
    // in this case lStageBgColor will be used because the content of transpAccColor is all 0
    vec3 bgColor = lStageBgColor * ( 1.0 - transpAccColor.a ) + transpAccColor;

    // apply transparency-blending
    vec3 diffuseColor = texture( diffuseColor, texCoord );

    // when using the diffuse-color texture as alpha source:
    out_color.rgb = bgColor * ( 1.0 - diffuseColor.a ) + diffuseColor.rgb * diffuseColor.a;

    // when using a constant for the whole object as alpha-source:
    // 10% transparent means alpha = 0.9 =>
    // bgColor contributes 10% to the color
    // diffuseColor contributes 90% of the color
    out_color.rgb = bgColor * ( 1.0 - alpha ) + diffuseColor.rgb * alpha;

    out_color.a = 1.0;
}
```

Then we have the transparencies accumulated in either color-attachment 0 (odd) or 1 (even) and must fill the empty gaps with the lighting-stage output texture.

  1. color-attachment4 ( lighting-stage output ) still bound to texture unit 0
  1. color-attachment0 or 1 bind to texture unit 1
  1. direct output of fragment-shader to color-attachment 0 or 1 ( the one not used in the previous step )

```
uniform sampler lStageBg;
uniform sampler transpAcc;

in vec2 ex_screenSpaceTexCoord;

out vec4 out_color;

void main()
{
    // access the lighting-stage result as background
    vec3 lStageBgColor = texture( lStageBg, ex_screenSpaceTexCoord );
    // access transparency accumulation
    vec3 transpAccColor = texture( transpAcc, ex_screenSpaceTexCoord );

    // a is 1 when another transparency is already accumulated at this fragment, 
    // in this case lStageBgColor will not be added because of multiplication by 0 but the transparency accumulated will be used
    // a is 0 when no transparency has been accumulated at this fragment, 
    // in this case lStageBgColor will be used because the content of transpAccColor is all 0
    out_color = lStageBgColor.rgb * ( 1.0 - transpAccColor.a ) + transpAccColor.rgb;
    out_color.a = 1.0;
}
```

To finally bring the result on screen, simply render a screen-sized quad and use the last written color-attachment ( 0 or 1 ) as texture.

Can we apply Shadows & lighting to the transparent objects?

**Optimizations**
  * use bounding geometry for lights: sphere for pointlight, cone for spotlight, full-screen quad for directional light.
  * do conditional rendering based upon occlusion query of the bounding geometry of the light
  * do stenciling for light-pixels
  * split lighting-stage fragment-shader into 3 distinct programs, one for each light-type to eliminate the dynamic-branchings (if they should become a bottleneck=
  * faster screen-space to world-space transformation

## Modeling ##
Supported model-formats
  * 3DS – 3D Studio
  * MS3D – Milkshape 3D
  * PLY -  Ply-Format (http://en.wikipedia.org/wiki/PLY_%28file_format%29)
  * TODO: OBJ – Obj-Format (http://en.wikipedia.org/wiki/Wavefront_.obj_file)

## Occlusion Culling ##
  * View-Frustum culling – NDC culling of BoundingBoxes
  * Dynamic occlusion culling with occlusion queries – CHC++. The depth buffer of the G-Stage can be utilized for the occlusion queries.

## Nice to have ##
  * SSAO
  * Planar Reflections
  * Dynamic Environment Reflections using cube-maps
  * Parallax Mapping

## Animation ##
It is not planned to implement any kind of animations in this engine.