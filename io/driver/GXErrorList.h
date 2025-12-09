/**
@File      GXErrorList.h
@Brief     the errorn list for the GxIAPI dll module. 
@Author    Software Department
@Date      2023-06-20
@Version   1.0.2306.9201
*/

#ifndef GX_ERROR_LIST_H
#define GX_ERROR_LIST_H


//////////////////////////////////////////////////////////////////////////
//Chinese：	类型定义，以下类型都在标准C库头文件stdint.h中有定义，但是在微软的编译平台
//			VS2010之前的版本中都不包含此文件,所以在此需要重定义

//English:	The following types are defined in the standard C library header file stdint.h, but are available on Microsoft's compilation platform
//			This file was not included in previous versions of VS2010, so it needs to be redefined here
//////////////////////////////////////////////////////////////////////////

#if defined(_WIN32)
	#ifndef _STDINT_H 
		#ifdef _MSC_VER // Microsoft compiler
			#if _MSC_VER < 1600
				typedef __int8            int8_t;
				typedef __int16           int16_t;
				typedef __int32           int32_t;
				typedef __int64           int64_t;
				typedef unsigned __int8   uint8_t;
				typedef unsigned __int16  uint16_t;
				typedef unsigned __int32  uint32_t;
				typedef unsigned __int64  uint64_t;
			#else
				// In Visual Studio 2010 is stdint.h already included
				#include <stdint.h>
			#endif
		#else
			// Not a Microsoft compiler
			#include <stdint.h>
		#endif
	#endif 
#else
	// Linux
	#include <stdint.h>
#endif

//------------------------------------------------------------------------------
//Chinese： 错误码定义
//English:	Error code definition
//------------------------------------------------------------------------------
typedef enum GX_STATUS_LIST
{
	GX_STATUS_SUCCESS                =  0,           ///< \Chinese	成功															\English	Success
	GX_STATUS_ERROR					 = -1,           ///< \Chinese	不期望发生的未明确指明的内部错误								\English	There is an unspecified internal error that is not expected to occur
	GX_STATUS_NOT_FOUND_TL           = -2,           ///< \Chinese	找不到TL库													\English	The TL library cannot be found
	GX_STATUS_NOT_FOUND_DEVICE       = -3,           ///< \Chinese	找不到设备													\English	The device is not found
	GX_STATUS_OFFLINE                = -4,           ///< \Chinese	当前设备为掉线状态											\English	The current device is in an offline status
	GX_STATUS_INVALID_PARAMETER		 = -5,           ///< \Chinese	无效参数,一般是指针为NULL或输入的IP等参数格式无效				\English	Invalid parameter. Generally, the pointer is NULL or the input IP and other parameter formats are invalid
	GX_STATUS_INVALID_HANDLE         = -6,           ///< \Chinese	无效句柄														\English	Invalid handle
	GX_STATUS_INVALID_CALL			 = -7,           ///< \Chinese	无效的接口调用,专指软件接口逻辑错误								\English	The interface is invalid, which refers to software interface logic error
	GX_STATUS_INVALID_ACCESS		 = -8,           ///< \Chinese	功能当前不可访问或设备访问模式错误								\English	The function is currently inaccessible or the device access mode is incorrect
	GX_STATUS_NEED_MORE_BUFFER		 = -9,           ///< \Chinese	用户申请的buffer不足:读操作时用户输入buffersize小于实际需要		\English	The user request buffer is insufficient: the user input buffersize during the read operation is less than the actual need
	GX_STATUS_ERROR_TYPE			 = -10,          ///< \Chinese	用户使用的FeatureID类型错误，比如整型接口使用了浮点型的功能码		\English	The type of FeatureID used by the user is incorrect, such as an integer interface using a floating - point function code
	GX_STATUS_OUT_OF_RANGE           = -11,          ///< \Chinese	用户写入的值越界												\English	The value written by the user is crossed
	GX_STATUS_NOT_IMPLEMENTED        = -12,          ///< \Chinese	当前不支持的功能												\English	This function is not currently supported
	GX_STATUS_NOT_INIT_API           = -13,          ///< \Chinese	没有调用初始化接口											\English	There is no call to initialize the interface
	GX_STATUS_TIMEOUT                = -14,          ///< \Chinese	超时错误														\English	Timeout error
}GX_STATUS_LIST;
typedef int32_t GX_STATUS;


#endif //GX_ERROR_LIST_H