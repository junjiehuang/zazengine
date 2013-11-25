#ifndef _GLUTILS_H
#define _GLUTILS_H

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define AT __FILE__ ":" TOSTRING(__LINE__)

#define GL_PEEK_ERRORS_AT GLUtils::peekErrors( AT );
#define GL_PEEK_ERRORS GLUtils::peekErrors( AT )
#define GL_PEEK_ERRORS_SILENT GLUtils::peekErrorsSilent();

#ifdef _DEBUG
	#define GL_PEEK_ERRORS_AT_DEBUG GLUtils::peekErrors( AT );
#else
	#define GL_PEEK_ERRORS_AT_DEBUG
#endif

#include <string>

class GLUtils
{
	public:
		static bool peekErrors( const std::string& );
		static bool peekErrorsSilent();
		static void enableDebugOutput();
};

#endif /* _GLUTILS_H */
