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
		
		glTexImage2D( GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,	// face
						0,									// level
						ilGetInteger( IL_IMAGE_FORMAT ),	// internal format
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
