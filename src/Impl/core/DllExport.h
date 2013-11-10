#ifndef _DLL_EXPORT_H_
#define _DLL_EXPORT_H_

#ifdef DLL_EXPORTS
#define DLL_API __declspec( dllexport )
#else
#define DLL_API __declspec( dllimport )
#endif

#endif /* _DLL_EXPORT_H_ */ 