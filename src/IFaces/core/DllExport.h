#ifndef DLL_EXPORT_H_
#define DLL_EXPORT_H_

#ifdef DLL_EXPORTS
#define DLL_API __declspec( dllexport )
#else
#define DLL_API __declspec( dllimport )
#endif

#endif /* DLL_EXPORT_H_ */ 