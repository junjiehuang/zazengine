#include "TextureFactory.h"

#include "../ZazenGraphics.h"
#include "../util/GLUtils.h"

#include <iostream>

#define MAX_MIPMAP_LEVEL 10
#define ANISOTROPY_LEVEL 4.0f

using namespace std;
using namespace boost;

map<string, Texture*> TextureFactory::allTextures;
boost::filesystem::path TextureFactory::textureDataPath;

int perlinNoisePerm[ 256 ]= {151,160,137,91,90,15,
  131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,8,99,37,240,21,10,23,
  190, 6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,57,177,33,
  88,237,149,56,87,174,20,125,136,171,168, 68,175,74,165,71,134,139,48,27,166,
  77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,55,46,245,40,244,
  102,143,54, 65,25,63,161, 1,216,80,73,209,76,132,187,208, 89,18,169,200,196,
  135,130,116,188,159,86,164,100,109,198,173,186, 3,64,52,217,226,250,124,123,
  5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,189,28,42,
  223,183,170,213,119,248,152, 2,44,154,163, 70,221,153,101,155,167, 43,172,9,
  129,22,39,253, 19,98,108,110,79,113,224,232,178,185, 112,104,218,246,97,228,
  251,34,242,193,238,210,144,12,191,179,162,241, 81,51,145,235,249,14,239,107,
  49,192,214, 31,181,199,106,157,184, 84,204,176,115,121,50,45,127, 4,150,254,
  138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180};

/* These are Ken Perlin's proposed gradients for 3D noise. I kept them for
   better consistency with the reference implementation, but there is really
   no need to pad this to 16 gradients for this particular implementation.
   If only the "proper" first 12 gradients are used, they can be extracted
   from the grad4[][] array: grad3[i][j] == grad4[i*2][j], 0<=i<=11, j=0,1,2
*/
int perlinNoiseGrad3[16][3] = {{0,1,1},{0,1,-1},{0,-1,1},{0,-1,-1},
                   {1,0,1},{1,0,-1},{-1,0,1},{-1,0,-1},
                   {1,1,0},{1,-1,0},{-1,1,0},{-1,-1,0}, // 12 cube edges
                   {1,0,-1},{-1,0,-1},{0,-1,1},{0,1,1}}; // 4 more to make 16

#define PERLIN_NOISE_2D_TEX_NAME	"PERLINNOISETEXTURE2D"

void
TextureFactory::init( const boost::filesystem::path& textureDataPath )
{
	ilInit();
	iluInit();

	TextureFactory::textureDataPath = textureDataPath;
}

Texture*
TextureFactory::get( const std::string& file )
{
	map<string, Texture*>::iterator findIter = TextureFactory::allTextures.find( file );
	if ( findIter != TextureFactory::allTextures.end() )
	{
		return findIter->second;
	}

	filesystem::path fullFileName( TextureFactory::textureDataPath.generic_string() + file );

	if ( false == filesystem::exists( fullFileName ) )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "Texture::get: file for texture " << fullFileName << " does not exist";
		return NULL;
	}

	if ( filesystem::is_directory( fullFileName ) )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "Texture::get: file for texture " << fullFileName << " is a directory";
		return NULL;
	}

	GLuint textureID = TextureFactory::createTexture( fullFileName.generic_string() );
	if ( 0 == textureID )
	{
		return NULL;
	}

	ZazenGraphics::getInstance().getLogger().logInfo() << "successfully loaded texture from " << fullFileName;

	Texture* newTexture = new Texture( textureID, Texture::TEXTURE_2D );
	
	TextureFactory::allTextures[ file ] = newTexture;
	
	return newTexture;
}

Texture*
TextureFactory::getNoiseTexture()
{
	map<string, Texture*>::iterator findIter = TextureFactory::allTextures.find( PERLIN_NOISE_2D_TEX_NAME );
	if ( findIter != TextureFactory::allTextures.end() )
	{
		return findIter->second;
	}

	GLuint textureID = TextureFactory::createPerlinNoise2dTexture();
	if ( 0 == textureID )
	{
		return NULL;
	}

	ZazenGraphics::getInstance().getLogger().logInfo() << "successfully created Perlin-Noise 2D texture";

	Texture* perlinNoise2DTex = new Texture( textureID, Texture::TextureType::TEXTURE_2D );
	
	TextureFactory::allTextures[ PERLIN_NOISE_2D_TEX_NAME ] = perlinNoise2DTex;
	
	return perlinNoise2DTex;
}

Texture*
TextureFactory::getCube( const boost::filesystem::path& cubeMapPath, const std::string& fileType )
{
	map<string, Texture*>::iterator findIter = TextureFactory::allTextures.find( cubeMapPath.generic_string() );
	if ( findIter != TextureFactory::allTextures.end() )
	{
		return findIter->second;
	}

	filesystem::path fullFileName( TextureFactory::textureDataPath.generic_string() + cubeMapPath.generic_string() );

	if ( false == filesystem::exists( fullFileName ) )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "Texture::getCube: file for texture " << fullFileName << " does not exist";
		return NULL;
	}

	if ( false == filesystem::is_directory( fullFileName ) )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "Texture::getCube: file for texture " << fullFileName << " is NOT a directory";
		return NULL;
	}

	vector<string> fileNames;
	fileNames.push_back( fullFileName.generic_string() + "/xpos" + "." + fileType );
	fileNames.push_back( fullFileName.generic_string() + "/xneg" + "." + fileType );
	fileNames.push_back( fullFileName.generic_string() + "/ypos" + "." + fileType );
	fileNames.push_back( fullFileName.generic_string() + "/yneg" + "." + fileType );
	fileNames.push_back( fullFileName.generic_string() + "/zpos" + "." + fileType );
	fileNames.push_back( fullFileName.generic_string() + "/zneg" + "." + fileType );

	GLuint textureID = TextureFactory::createCubeTexture( fileNames );
	if ( 0 == textureID )
	{
		return NULL;
	}

	ZazenGraphics::getInstance().getLogger().logInfo() << "successfully loaded cube-texture from " << fullFileName;

	Texture* newTexture = new Texture( textureID, Texture::TEXTURE_CUBE );
	
	TextureFactory::allTextures[ cubeMapPath.generic_string() ] = newTexture;
	
	return newTexture;
}

bool
TextureFactory::captureScreen( const std::string& fileName )
{
	ILuint imageID = ilGenImage();
    ilBindImage( imageID );
    ilutGLScreen();
    ilEnable( IL_FILE_OVERWRITE );
	ilSaveImage( fileName.c_str() );
    ilDeleteImage( imageID );

	return true;
}

void
TextureFactory::freeAll()
{
	map<string, Texture*>::iterator iter = TextureFactory::allTextures.begin();
	while(iter != TextureFactory::allTextures.end())
	{
		delete iter->second;
		
		iter++;
	}
	
	TextureFactory::allTextures.clear();
}

/** USE FRAGMENT-SHADER FUNCTION TO GET PERLIN NOISE
	//  2D classic noise uses only permTexture.
	uniform sampler2D permTexture;

	//To create offsets of one texel and one half texel in the
	//texture lookup, we need to know the texture image size.
	#define ONE 0.00390625
	#define ONEHALF 0.001953125

	// The interpolation function. This could be a 1D texture lookup
	// to get some more speed, but it's not the main part of the algorithm.
	float fade(float t) {
	  // return t*t*(3.0-2.0*t); // Old fade, yields discontinuous second derivative
	  return t*t*t*(t*(t*6.0-15.0)+10.0); // Improved fade, yields C2-continuous noise
	}

	// 2D classic Perlin noise. Fast, but less useful than 3D noise.
	float noise(vec2 P)
	{
	  vec2 Pi = ONE*floor(P)+ONEHALF; // Integer part, scaled and offset for texture lookup
	  vec2 Pf = fract(P);             // Fractional part for interpolation

	  // Noise contribution from lower left corner
	  vec2 grad00 = texture2D(permTexture, Pi).rg * 4.0 - 1.0;
	  float n00 = dot(grad00, Pf);

	  // Noise contribution from lower right corner
	  vec2 grad10 = texture2D(permTexture, Pi + vec2(ONE, 0.0)).rg * 4.0 - 1.0;
	  float n10 = dot(grad10, Pf - vec2(1.0, 0.0));

	  // Noise contribution from upper left corner
	  vec2 grad01 = texture2D(permTexture, Pi + vec2(0.0, ONE)).rg * 4.0 - 1.0;
	  float n01 = dot(grad01, Pf - vec2(0.0, 1.0));

	  // Noise contribution from upper right corner
	  vec2 grad11 = texture2D(permTexture, Pi + vec2(ONE, ONE)).rg * 4.0 - 1.0;
	  float n11 = dot(grad11, Pf - vec2(1.0, 1.0));

	  // Blend contributions along x
	  vec2 n_x = mix(vec2(n00, n01), vec2(n10, n11), fade(Pf.x));

	  // Blend contributions along y
	  float n_xy = mix(n_x.x, n_x.y, fade(Pf.y));

	  // We're done, return the final noise value.
	  return n_xy;
	}
**/

GLuint
TextureFactory::createPerlinNoise2dTexture()
{
	GLuint textureId = 0;
	bool error = false;

	char pixels[ 256 * 256  *4 ];
  
	glGenTextures( 1, &textureId );
	if ( GL_PEEK_ERRORS )
	{
		error = true;
		goto cleanupExit;
	}

	glBindTexture( GL_TEXTURE_2D, textureId );
	if ( GL_PEEK_ERRORS )
	{
		error = true;
		goto cleanupExit;
	}

	for( unsigned int i = 0; i < 256; ++i ) {
		for( unsigned j = 0; j<256; ++j ) {
			int offset = ( i * 256 + j ) * 4;
			char value = perlinNoisePerm[ ( j + perlinNoisePerm[ i ]) & 0xFF ];
			pixels[offset] = perlinNoiseGrad3[ value & 0x0F ][ 0 ] * 64 + 64;		// Gradient x
			pixels[offset+1] = perlinNoiseGrad3[ value & 0x0F ][ 1 ] * 64 + 64;	// Gradient y
			pixels[offset+2] = perlinNoiseGrad3[ value & 0x0F ][ 2 ] * 64 + 64;	// Gradient z
			pixels[offset+3] = value;									// Permuted index
		}
	}
  
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
	GL_PEEK_ERRORS_AT
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
	GL_PEEK_ERRORS_AT

	glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, 256, 256, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels );
	if ( GL_PEEK_ERRORS )
	{
		error = true;
		goto cleanupExit;
	}

cleanupExit:
	if ( error )
	{
		if ( textureId )
		{
			glDeleteTextures( 1, &textureId );
		}
	}

	// unbind currently bound texture
	glBindTexture( GL_TEXTURE_2D, 0 );

	return textureId;
}

GLuint
TextureFactory::createTexture( const std::string& fullFileName )
{
	ILuint imageId = 0;
	GLuint textureId = 0;
	bool error = false;

	if ( false == TextureFactory::createImage( fullFileName, &imageId ) )
	{
		return 0;
	}

	ilBindImage( imageId );
	
	glGenTextures( 1, &textureId );
	if ( GL_PEEK_ERRORS )
	{
		error = true;
		goto cleanupExit;
	}

	glBindTexture( GL_TEXTURE_2D, textureId );
	if ( GL_PEEK_ERRORS )
	{
		error = true;
		goto cleanupExit;
	}

	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP );
	GL_PEEK_ERRORS_AT
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP );
	GL_PEEK_ERRORS_AT
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR );
	GL_PEEK_ERRORS_AT
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
	GL_PEEK_ERRORS_AT
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0 ); 
	GL_PEEK_ERRORS_AT
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, MAX_MIPMAP_LEVEL ); 
	GL_PEEK_ERRORS_AT
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, ANISOTROPY_LEVEL ); 
	GL_PEEK_ERRORS_AT

	glTexImage2D( GL_TEXTURE_2D,
					0,
					GL_RGBA,
					ilGetInteger( IL_IMAGE_WIDTH ),
					ilGetInteger( IL_IMAGE_HEIGHT ),
					0,
					GL_RGBA,
					GL_UNSIGNED_BYTE,
					ilGetData() );
	if ( GL_PEEK_ERRORS )
	{
		error = true;
		goto cleanupExit;
	}

	glGenerateMipmap( GL_TEXTURE_2D );
	if ( GL_PEEK_ERRORS )
	{
		error = true;
		goto cleanupExit;
	}

cleanupExit:
	if ( error )
	{
		if ( textureId )
		{
			glDeleteTextures( 1, &textureId );
		}
	}

	// Because we have already copied image data into texture data we can release memory used by image.
 	ilDeleteImages( 1, &imageId ); 

	// unbind currently bound texture
	glBindTexture( GL_TEXTURE_2D, 0 );

	return textureId;
}

GLuint
TextureFactory::createCubeTexture( const std::vector<std::string>& fileNames )
{
	GLuint textureId = 0;
	ILuint imageIds[ 6 ];
	bool error = false;

	memset( imageIds, 0, sizeof( imageIds ) );

	if ( false == TextureFactory::createImages( fileNames, imageIds ) )
	{
		return 0;
	}

	glGenTextures( 1, &textureId );
	if ( GL_PEEK_ERRORS )
	{
		error = true;
		goto cleanupExit;
	}

	glBindTexture( GL_TEXTURE_CUBE_MAP, textureId );
	if ( GL_PEEK_ERRORS )
	{
		error = true;
		goto cleanupExit;
	}

	glTexParameteri( GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR );
	GL_PEEK_ERRORS_AT
	glTexParameteri( GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
	GL_PEEK_ERRORS_AT
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	GL_PEEK_ERRORS_AT
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	GL_PEEK_ERRORS_AT
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
	GL_PEEK_ERRORS_AT
	glTexParameteri( GL_TEXTURE_CUBE_MAP, GL_TEXTURE_BASE_LEVEL, 0 ); 
	GL_PEEK_ERRORS_AT
	glTexParameteri( GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAX_LEVEL, MAX_MIPMAP_LEVEL ); 
	GL_PEEK_ERRORS_AT
	glTexParameterf( GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAX_ANISOTROPY_EXT, ANISOTROPY_LEVEL ); 
	GL_PEEK_ERRORS_AT

	for ( unsigned int i = 0; i < 6; i++ )
	{
		ilBindImage( imageIds[ i ] );
		if ( IL_NO_ERROR != ilGetError() )
		{
			error = true;
			goto cleanupExit;
		}
		
		ILint internalFormat = ilGetInteger( IL_IMAGE_FORMAT );
		glTexImage2D( GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,	// face
						0,									// level
						internalFormat,	// internal format
						ilGetInteger( IL_IMAGE_WIDTH ),		// width of face
						ilGetInteger( IL_IMAGE_HEIGHT ),	// height of face
						0,									// border
						GL_RGBA,							// format
						GL_UNSIGNED_BYTE,					// data-type
						ilGetData() );						// data

		if ( GL_PEEK_ERRORS )
		{
			error = true;
			goto cleanupExit;
		}
	}

	glGenerateMipmap( GL_TEXTURE_CUBE_MAP );
	if ( GL_PEEK_ERRORS )
	{
		error = true;
		goto cleanupExit;
	}

cleanupExit:
	if ( error )
	{
		if ( textureId )
		{
			glDeleteTextures( 1, &textureId );
			GL_PEEK_ERRORS_AT
		}
	}

	// Because we have already copied image data into texture data we can release memory used by image.
	ilDeleteImages( 6, imageIds ); 

	// unbind currently bound texture
	glBindTexture( GL_TEXTURE_CUBE_MAP, 0 );
	GL_PEEK_ERRORS_AT

	return textureId;
}


bool
TextureFactory::createImage( const std::string& fileName, ILuint* imageId )
{
	ilGenImages( 1, imageId );
	if ( IL_NO_ERROR != ilGetError() )
	{
		return false;
	}

	if ( false == TextureFactory::loadImage( fileName, *imageId ) )
	{
		ilDeleteImages( 1, imageId );
		*imageId = 0;
		return false;
	}

	return true;
}

bool
TextureFactory::createImages( const std::vector<std::string>& fileNames, ILuint* imageIds )
{
	bool errorFlag = false;

	ilGenImages( fileNames.size(), imageIds );
	if ( IL_NO_ERROR != ilGetError() )
	{
		return false;
	}

	for ( unsigned int i = 0; i < fileNames.size(); i++ )
	{
		if ( false == TextureFactory::loadImage( fileNames[ i ], imageIds[ i ] ) )
		{
			errorFlag = true;
			imageIds[ i ] = 0;
			break;
		}
	}

	if ( errorFlag )
	{
		ilDeleteImages( fileNames.size(), imageIds );
		return false;
	}

	return true;
}

bool
TextureFactory::loadImage( const std::string& fileName, ILuint imageId )
{
	ilBindImage( imageId );
	if ( IL_NO_ERROR != ilGetError() )
	{
		return false;
	}

	if ( IL_TRUE == ilLoadImage( fileName.c_str() ) )
	{
		if ( IL_TRUE != ilConvertImage( IL_RGBA, IL_UNSIGNED_BYTE ) )
		{
			ZazenGraphics::getInstance().getLogger().logError() << "Texture::loadImage: Image load failed - IL reports error: " << ilGetError();
			return false;
		}
 	}
  	else
  	{
		ZazenGraphics::getInstance().getLogger().logError() << "Texture::loadImage: Image load failed - IL reports error: " << ilGetError();
		return false;
  	}

	return true;
}
