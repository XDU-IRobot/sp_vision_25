/**
@File      GxIAPI.h
@Brief     the interface for the GxIAPI dll module.
@Author    Software Department
@Date      2024-12-19
@Version   2.0.2412.9191
*/

#ifndef GX_GALAXY_H
#define GX_GALAXY_H

#include "GXDef.h"
#include "GXErrorList.h"
#include "GxIAPILegacy.h"
#include "GxPixelFormat.h"

//------------------------------------------------------------------------------
//Chinese	标准C API功能函数定义
//English	Standard C API function definition
//------------------------------------------------------------------------------

#ifndef _WIN32
//----------------------------------------------------------------------------------
/**
\Chinese:
\brief		获取动态库版本号.
\return 	动态库版本号.

\English
\brief      Gets the library version number.
\return     const char*   Library version number of string type.
*/
//----------------------------------------------------------------------------------
GX_EXTC GX_DLLEXPORT const char *GX_STDC GXGetLibVersion();
#endif

//------------------------------------------------------------------------
/**
\Chinese：
\brief		初始化设备库。
\attention	调用其他接口（除了GXGetLastError和GXCloseLib）之前必须先调用此接口，当用户不再使用库的时候调用GXCloseLib释放库资源。
			如果之前用户已经调用过GXInitLib，没有调用GXCloseLib，而再次调用GXInitLib接口，接口返回成功。
\return		GX_STATUS_SUCCESS				操作成功，没有发生错误
			GX_STATUS_NOT_FOUND_TL			找不到TL库
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief		Initialize the device library.
\attention	This interface must be called before calling any other interface (except GXGetLastError and GXCloseLib),
			and GXCloseLib is called to release library resources when the user is no longer using the library.
			If the user has called GXInitLib before, does not call GXCloseLib, and calls the GXInitLib interface again, the interface returns success.
\return		GX_STATUS_SUCCESS				The operation is successful, no error occurs.
			GX_STATUS_NOT_FOUND_TL			Can not found the library.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//------------------------------------------------------------------------
GX_API GXInitLib();

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief		关闭设备库，释放资源
\attention	释放库资源，当用户不再使用库的时候调用此接口。
			如果用户之前没有调用GXInitLib，直接调用GXCloseLib，接口返回成功。
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief		Close the device library to release resources.
\attention	Frees library resources and invokes this interface when the user is no longer using the library.
			If the user did not call GXInitLib before, call GXCloseLib directly and the interface returns success.
\return		GX_STATUS_SUCCESS				The operation is successful, no error occurs.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXCloseLib();

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      设置指定类型的日志可发送
\attention  此接口的作用是设置指定类型日志可发送，调用之前需要调用GXInitLib接口
\param      [in]ui32LogType                 日志类型： GX_LOG_TYPE_FATAL|GX_LOG_TYPE_ERROR|GX_LOG_TYPE_INFO
                                                      GX_LOG_TYPE_OFF： 全部不可发送
\return     GX_STATUS_SUCCESS               操作成功，没有发生错误
            GX_STATUS_NOT_INIT_API          没有调用GXInitLib初始化库
            GX_STATUS_INVALID_ACCESS        配置文件路径错误
            其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Set whether logs of the specified type can be sent
\attention  The function of this interface is to set whether the specified type of logs can be sent.
            The GXInitLib interface needs to be called before the invocation
\param      [in]ui32LogType                 log type:   GX_LOG_TYPE_FATAL|GX_LOG_TYPE_ERROR|GX_LOG_TYPE_INFO
                                                        GX_LOG_TYPE_OFF: all cannot be sent
\return		GX_STATUS_SUCCESS				The operation is successful and no error occurs.
            GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
            GX_STATUS_INVALID_ACCESS		The config file path is not available.
            The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXSetLogType(const uint32_t ui32LogType);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      获取指定类型的日志是否可发送
\attention  此接口的作用是获取指定类型日志是否可发送，调用之前需要调用GXInitLib接口
\param      [out]pui32Value                  日志类型是否可发送：FATAL|ERROR|WARN|INFO|DEBUG|TRACE
\return     GX_STATUS_SUCCESS               操作成功，没有发生错误
            GX_STATUS_NOT_INIT_API          没有调用GXInitLib初始化库
            GX_STATUS_INVALID_PARAMETER     用户输入的指针为NULL
            GX_STATUS_INVALID_ACCESS        配置文件路径错误
            其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Gets whether logs of a specified type can be sent
\attention  The function of this interface is to get whether the specified type of logs can be sent.
            The GXInitLib interface needs to be called before the invocation
\param      [out]pui32Value                 log type value: FATAL|ERROR|WARN|INFO|DEBUG|TRACE
\return		GX_STATUS_SUCCESS				The operation is successful and no error occurs.
            GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
            GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
            GX_STATUS_INVALID_ACCESS		The config file path is not available.
            The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXGetLogType(uint32_t* pui32Value);

//------------------------------------------------------------------------
/**
\Chinese：
\brief      获取程序最后的错误描述信息
\attention  当用户调用其它接口失败的时候，可以调用此接口获取关于失败信息的详细描述
\param		[out] pErrorCode		返回最后的错误码，如果用户不想获取此值，那么此参数可以传NULL
\param		[out] pszErrText		返回错误信息缓冲区地址
\param		[in,out] pSize			错误信息缓冲区地址大小，单位字节
									如果pszErrText为NULL：
									[out]pnSize返回实际需要的buffer大小
									如果pszErrText非NULL：
									[in]pnSize为实际分配的buffer大小
									[out]pnSize返回实际填充buffer大小
\return		GX_STATUS_SUCCESS                操作成功，没有发生错误
			GX_STATUS_INVALID_PARAMETER      用户输入的指针为NULL
			GX_STATUS_NEED_MORE_BUFFER       用户分配的buffer过小
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      To get the latest error descriptions information of the program.
\attention  When a user fails to invoke another interface, the user can invoke this interface to obtain a detailed description of the failure information.
\param		[out] pErrorCode    Return the last error code. You could set the parameter to NULL if you don't need this value.
\param		[out] pszErrText    Return the address of the buffer allocated for error information.
\param		[in,out] pSize      The address size of the buffer allocated for error information. Unit: byte.
								If pszErrText is NULL:
								[out]pnSize Return the actual required buffer size.
								If pszErrText is not NULL:
								[in]pnSize It is the actual allocated buffer size.
								[out]pnSize Return the actual allocated buffer size.
\return		GX_STATUS_SUCCESS					The operation is successful and no error occurs.
			GX_STATUS_INVALID_PARAMETER			The pointer that the user input is NULL.
			GX_STATUS_NEED_MORE_BUFFER			The buffer that the user filled is too small.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//------------------------------------------------------------------------
GX_API GXGetLastError(GX_STATUS *pErrorCode, char *pszErrText, size_t *pSize);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief		枚举特定类型设备并且获取设备个数
\attention	此接口的作用是更新库内部设备列表，此接口会改变库内部设备列表，
			所以调用GXGetInterfaceInfo、GXGetInterfaceNum、GXGetInterfaceHandle、GXGetDeviceInfo和GXOpenDevice之前需要调用此接口。
			如果在用户指定超时时间内成功枚举到设备，则立即返回；如果在用户指定超时时间内没有枚举到设备，则一直等待，直到达到用户指定的超时时间返回。
\param		[in]nTLType				枚举特定类型的设备，参考GX_TL_TYPE_LIST
			[out]punNumDevices		返回设备个数
\param		[in]unTimeOut			枚举的超时时间(单位ms)。
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief		Enumerate a specific type of device and get the number of devices
\attention  The function of this interface is to update the list of internal devices in the library. This interface changes the list of internal devices in the library.
			Therefore, this interface must be invoked before calling GXGetInterfaceInfo, GXGetInterfaceNum, GXGetInterfaceHandle, GXGetDeviceInfo, and GXOpenDevice.
			If the device is successfully enumerated within the specified timeout time, the value returns immediately.
			If the device is not enumerated within the specified timeout time, then it waits until the specified timeout time is over and then it returns.
\param		[in]nTLType				Enumerate specific types of devices,reference to GX_TL_TYPE_LIST
			[out]punNumDevices		Return number of devices
\param		[in]unTimeOut			The timeout time of enumeration (unit: ms).
\return		GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXUpdateAllDeviceListEx( uint64_t nTLType, uint32_t* punNumDevices, uint32_t nTimeOut);

//----------------------------------------------------------------------------------
/**
\ Chinese：
\brief		枚举所有设备并且获取设备个数,对于千兆网设备此接口能够枚举所有子网内的设备
\attention	此接口的作用是更新库内部设备列表，此接口会改变库内部设备列表，
			所以调用GXGetInterfaceInfo、GXGetInterfaceNum、GXGetInterfaceHandle、GXGetDeviceInfo和GXOpenDevice之前需要调用此接口。
			如果在用户指定超时时间内成功枚举到设备，则立即返回；如果在用户指定超时时间内没有枚举到设备，则一直等待，直到达到用户指定的超时时间返回
\param		[out]punNumDevices			 返回设备个数
\param		[in]unTimeOut				 枚举的超时时间(单位ms)。
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief		Enumerate all devices and obtain the number of devices,
			For gige devices, this interface can enumerate devices in all subnets.
\attention	The function of this interface is to update the list of internal devices in the library. This interface changes the list of internal devices in the library.
			Therefore, this interface must be invoked before calling GXGetInterfaceInfo, GXGetInterfaceNum, GXGetInterfaceHandle, GXGetDeviceInfo, and GXOpenDevice.
			If the device is successfully enumerated within the specified timeout time, the value returns immediately.
			If the device is not enumerated within the specified timeout time, then it waits until the specified timeout time is over and then it returns.
\param		[out]punNumDevices			 Return number of devices
\param		[in]unTimeOut				 The timeout time of enumeration (unit: ms).
\return		GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXUpdateAllDeviceList      (uint32_t* punNumDevices, uint32_t nTimeOut);

//----------------------------------------------------------------------------------
/**
\ Chinese：
\brief		通过发送ActionCommond命令使网络上的相机同时执行action动作
\attention	请使用IPV4地址，groupMask必须不为0，如果timeoutMs不为0则pNumResults、*pNumResults、pResults必须不为0
\param      [in]deviceKey			         协议规定的设备秘钥
\param      [in]groupKey			         协议规定的组秘钥
\param      [in]groupMask			         协议规定的组掩码
\param      [in]pBroadcastAddress			 发送cmd的目的ip,可为广播ip(255.255.255.255)、子网广播(192.168.42.255)、单播(192.168.42.42)
\param      [in]pSpecialAddress			     可选：发送cmd的源ip，用于明确的表面从哪个网口上发送命令，可解决多网卡情况下重复发送cmd问题
\                                            如果不指定当广播跟子网广播情况下当前主机的每一个符合的网卡都会发action
\param      [in]timeoutMs			         可选：响应ack的超时时间(ms)。如果已收到 pNumResults，则停止等待确认。
\                                            如果不需要检查操作命令结果，则此参数可以为 0。
\param      [in][out]pNumResults			 可选：结果数组中的结果数。
\                                            传递的值应等于确认命令的预期设备数量。
\                                            如果 timeoutMs 为 0，则忽略此参数。因此，如果 timeoutMs 为 0，则此参数可以为 NULL。
\param      [in][out]pResults			     可选：一个包含 *pNumResults 元素的数组，用于保存操作命令结果状态。缓冲区从头开始填充。
\                                            如果收到的结果少于可用的结果项，则不会更改剩余结果。
\                                            如果 timeoutMs 为 0，则忽略此参数。因此，如果 timeoutMs 为 0，则此参数可以为 NULL。
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief		Sending the ActionCommon command allows cameras on the network to execute actions simultaneously.
\attention	 IP4 addresses must be used. groupMask must not be 0. pNumResult, *pNumResult and pResults must not be 0 if a timeoutMs value other than 0 is passed.
\param      [in]deviceKey			         Device key specified by the protocol
\param      [in]groupKey			         The group key specified by the protocol
\param      [in]groupMask			         Protocol-defined group mask
\param      [in]pBroadcastAddress			 The destination IP to send cmd to can be broadcast IP (255.255.255.255), subnet broadcast (192.168.42.255), unicast (192.168.42.42)
\param      [in]pSpecialAddress			     The source IP of the cmd is used to clearly identify the network port from which the command is sent. 
\                                            This can solve the problem of repeated cmd sending in the case of multiple network cards.
\param      [in]timeoutMs			         Optional: Time in milliseconds the call is waiting for acknowledges of the addressed devices. 
\                                            Waiting for acknowledges is stopped if pNumResults have been received. 
\                                            This parameter can be 0 if a check of action command results is not required.
\param      [in][out]pNumResults			 Optional: The number of results in the results array. 
\                                            The value passed should be equal to the expected number of devices that acknowledge the command. 
\                                            Returns the number of received results. This parameter is ignored if timeoutMs is 0. 
\                                            Thus, this parameter can be NULL if timeoutMs is 0.
\param      [in][out]pResults			     Optional: An array with *pNumResults elements to hold the action command result status. 
\                                            The buffer is filled beginning from the start. 
\                                            Remaining results are not changed if less results are received than result items available. 
\                                            This parameter is ignored if timeoutMs is 0. Thus, this parameter can be NULL if timeoutMs is 0.
\return		GX_STATUS_SUCCESS                The operation was successful and no errors occurred
            GX_STATUS_NOT_INIT_API            GXInitLib is not called to initialize the library
            For other error conditions, see GX_STATUS_LIST
*/
//----------------------------------------------------------------------------------
GX_API GXGigEIssueActionCommand( uint32_t nDeviceKey, uint32_t nGroupKey, 
                                uint32_t nGroupMask, const char* strBroadcastAddress, 
                                const char* strSpecialAddress, uint32_t nTimeoutMs, 
                                uint32_t* pnNumResults, GX_GIGE_ACTION_COMMAND_RESULT* pstResults );

//----------------------------------------------------------------------------------
/**
\ Chinese：
\brief		通过发送ActionCommond命令使网络上的相机同时执行action动作
\attention	请使用IPV4地址，groupMask必须不为0，如果timeoutMs不为0则pNumResults、*pNumResults、pResults必须不为0
\param      [in]deviceKey			         协议规定的设备秘钥
\param      [in]groupKey			         协议规定的组秘钥
\param      [in]groupMask			         协议规定的组掩码
\param      [in]pBroadcastAddress			 发送cmd的目的ip,可为广播ip(255.255.255.255)、子网广播(192.168.42.255)、单播(192.168.42.42)
\param      [in]actiontimeNs			     执行操作的时间（以纳秒为单位）。实际值取决于所使用的主时钟。
\                                            例如，可以通过在从一组相机设备中锁存时间戳值GXSetCommandValue(hDevice, "TimestampLatch")
\                                            后读取时间戳值GXGetIntValue(hDevice, "TimestampLatchValue", &pstIntValue) 来获取一组同步相机设备的主时钟值。
\param      [in]pSpecialAddress			     可选：发送cmd的源ip，用于明确的表面从哪个网口上发送命令，可解决多网卡情况下重复发送cmd问题,
\                                            如果不指定当广播跟子网广播情况下当前主机的每一个符合的网卡都会发action
\param      [in]timeoutMs			         可选：响应ack的超时时间(ms)。如果已收到 pNumResults，则停止等待确认。
\                                            如果不需要检查操作命令结果，则此参数可以为 0。
\param      [in][out]pNumResults			 可选：结果数组中的结果数。
\                                            传递的值应等于确认命令的预期设备数量。
\                                            如果 timeoutMs 为 0，则忽略此参数。因此，如果 timeoutMs 为 0，则此参数可以为 NULL。
\param      [in][out]pResults			     可选：一个包含 *pNumResults 元素的数组，用于保存操作命令结果状态。缓冲区从头开始填充。
\                                            如果收到的结果少于可用的结果项，则不会更改剩余结果。
\                                            如果 timeoutMs 为 0，则忽略此参数。因此，如果 timeoutMs 为 0，则此参数可以为 NULL。
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief		Sending the ActionCommon command allows cameras on the network to execute actions simultaneously.
\attention	IP4 addresses must be used. groupMask must not be 0. pNumResult, *pNumResult and pResults must not be 0 if a timeoutMs value other than 0 is passed.
\param      [in]deviceKey			         Device key specified by the protocol
\param      [in]groupKey			         The group key specified by the protocol
\param      [in]groupMask			         Protocol-defined group mask
\param      [in]actiontimeNs                 The time in nanoseconds to perform the operation.The actual value depends on the master clock used.
\                                            For example, the master clock value of a set of synchronized camera devices can be obtained by reading the 
\                                            timestamp value GXGetIntValue(hDevice, "TimestampLatch") after latching it from the set of camera devices
\param      [in]pBroadcastAddress			 The destination IP to send cmd to can be broadcast IP (255.255.255.255), subnet broadcast (192.168.42.255), unicast (192.168.42.42)
\param      [in]pSpecialAddress			     Optional: The source IP of the cmd is used to clearly identify the network port from which the command is sent. 
\                                            This can solve the problem of repeated cmd sending in the case of multiple network cards.
\param      [in]timeoutMs			         Optional: Time in milliseconds the call is waiting for acknowledges of the addressed devices. 
\                                            Waiting for acknowledges is stopped if pNumResults have been received. 
\                                            This parameter can be 0 if a check of action command results is not required.
\param      [in][out]pNumResults			 Optional: The number of results in the results array. 
\                                            The value passed should be equal to the expected number of devices that acknowledge the command. 
\                                            Returns the number of received results. This parameter is ignored if timeoutMs is 0. 
                                             Thus, this parameter can be NULL if timeoutMs is 0.
\param      [in][out]pResults			     Optional: An array with *pNumResults elements to hold the action command result status. 
\                                            The buffer is filled beginning from the start. 
\                                            Remaining results are not changed if less results are received than result items available. 
\                                            This parameter is ignored if timeoutMs is 0. Thus, this parameter can be NULL if timeoutMs is 0.
\return		GX_STATUS_SUCCESS                The operation was successful and no errors occurred
            GX_STATUS_NOT_INIT_API            GXInitLib is not called to initialize the library
            For other error conditions, see GX_STATUS_LIST
*/
//----------------------------------------------------------------------------------
GX_API GXGigEIssueScheduledActionCommand( uint32_t nDeviceKey, uint32_t nGroupKey, 
                                         uint32_t nGroupMask, uint64_t nActiontimeNs, 
                                         const char* strBroadcastAddress, const char* strSpecialAddress, 
                                         uint32_t nTimeoutMs, uint32_t* pnNumResults, 
                                         GX_GIGE_ACTION_COMMAND_RESULT* pstResults );

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      获取枚举到的Interface列表数量
\attention  此接口的作用是获取枚举到的Interface数量，
			调用之前需要调用GXUpdateAllDeviceList、GXUpdateAllDeviceListEx接口。
\param      [in,out]punNumInterfaces      返回Interface个数
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Obtain the number of interface lists enumerated.
\attention  The purpose of this Interface is to get the number of interfaces enumerated,
			please call GXUpdateAllDeviceList, GXUpdateAllDeviceListEx interfaces before calling.
\param      [in,out]punNumInterfaces		Return the number of interfaces
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXGetInterfaceNum(uint32_t* punNumInterfaces);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      获取次序为Index的Interface信息
\attention  此接口的作用是获取枚举到Interface接口，
            调用之前需要调用GXUpdateAllDeviceList、GXUpdateAllDeviceListEx接口
\param      [in]nIndex					Interface 次序，从开始 1
            [in,out]pstInterfaceInfo	返回Interface信息
\return     GX_STATUS_SUCCESS				操作成功，没有发生错误
            GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
            GX_STATUS_INVALID_PARAMETER		用户输入的指针为NULL
            其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Obtain basic interface information nIndex device.
\attention  The purpose of this interface is to obtain the enumerated Interface Info interface,
			please call GXUpdateAllDeviceList, GXUpdateAllDeviceListEx interface before calling.
\param      [in]nIndex					Interface order, starting from 1
			[out]pstInterfaceInfo       Return interface information
\return		GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXGetInterfaceInfo(uint32_t nIndex, GX_INTERFACE_INFO* pstInterfaceInfo);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      获取次序为Index的Interface Handle
\attention  此接口的作用是获取枚举到Interface Handle的接口，
            调用之前需要调用GXUpdateAllDeviceList、GXUpdateAllDeviceListEx接口
\param      [in]nIndex              Interface次序，从1开始
            [in,out]phIF			返回Interface句柄
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
            GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
            GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
            其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Get Interface Handles of Index.
\attention  The purpose of this interface is to obtain the enumerated Handle interface,
			please call GXUpdateAllDeviceList, GXUpdateAllDeviceListEx interface before calling.
\param      [in]nIndex					Interface order, starting from 1
			[in,out]phIF				Return Interface handle
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXGetInterfaceHandle(uint32_t nIndex, GX_IF_HANDLE* phIF);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      获取次序为Index的设备信息
\attention  此接口的作用是获取枚举到设备信息接口，
            调用之前需要调用GXUpdateAllDeviceList、GXUpdateAllDeviceListEx接口
\param      [in]nIndex                    设备次序，从1开始
            [in,out]pstDeviceInfo         返回设备信息
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
            GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
            GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
            其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Obtain basic information of nIndex devices
\attention  The function of this interface is to obtain enumeration to the device information interface,
			please call GXUpdateAllDeviceList, GXUpdateAllDeviceListEx interface before calling.
\param      [in]nIndex                    Device order,starting from 1
			[in,out]pstDeviceInfo         Return Device information
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXGetDeviceInfo(uint32_t nIndex, GX_DEVICE_INFO* pstDeviceInfo);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief		通过指定唯一标示打开设备，例如指定SN、IP、MAC等
\attention	此接口调用之前需要调用GXUpdateAllDeviceList、GXUpdateAllDeviceListEx接口，更新库内部设备列表
\param		[in]pOpenParam		用户配置的打开设备参数,参见GX_OPEN_PARAM结构体定义
\param		[out]phDevice		返回设备句柄
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
			GX_STATUS_NOT_FOUND_DEVICE    没有找到与指定信息匹配的设备
			GX_STATUS_INVALID_ACCESS      设备的访问方式不对
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief		Open the device by a specific unique identification, such as: SN, IP, MAC, Index etc.
\attention	Before invoking this interface, you need to call the GXUpdateAllDeviceList or GXUpdateAllDeviceListEx interfaces to update the device list in the library
\param		[in]pOpenParam		The open device parameter which is configurated by user,Ref:GX_OPEN_PARAM.
\param		[out]phDevice		Return device handle
\return		GX_STATUS_SUCCESS             The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API        The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER   The pointer that the user input is NULL.
			GX_STATUS_NOT_FOUND_DEVICE    Not found the device that matches the specific information.
			GX_STATUS_INVALID_ACCESS      The device can not be opened under the current access mode.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXOpenDevice               (GX_OPEN_PARAM* pOpenParam, GX_DEV_HANDLE* phDevice);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief		指定设备句柄关闭设备
\attention	不能重复关闭同一个设备
\param		[in]hDevice			即将要关闭的设备句柄
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄，或者关闭已经被关闭的设备
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief		Specify the device handle to close the device.
\attention	The same device cannot be shut down repeatedly.
\param		[in]hDevice			The device handle that the user specified to close.
\return		GX_STATUS_SUCCESS             The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API        The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE      The illegal handle that the user introduces, or reclose thedevice.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXCloseDevice              (GX_DEV_HANDLE hDevice);


// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      获取设备所属的Interface句柄
\param		[in]hDevice			    设备handle
\param		[in,out]phIF            Interface Handle
\return     GX_STATUS_SUCCESS				操作成功，没有发生错误
            GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
            GX_STATUS_INVALID_PARAMETER		用户输入的指针为NULL
			GX_STATUS_INVALID_HANDLE		用户传入非法的句柄，或者关闭已经被关闭的设备
            其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Obtain the device's interface handle.
\param		[in]hDevice			    Device handle
\param		[in,out]phIF            Interface Handle
\return		GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose thedevice.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXGetParentInterfaceFromDev(GX_DEV_HANDLE hDevice, GX_IF_HANDLE* phIF);


// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      获取本地设备handle
\param		[in]hDevice			    设备handle
\param		[in,out]phLocalDev      本地设备Handle
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
            GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
            GX_STATUS_INVALID_PARAMETER		用户输入的指针为NULL
			GX_STATUS_INVALID_HANDLE		用户传入非法的句柄，或者关闭已经被关闭的设备
            其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Get local device handle.
\param		[in]hDevice			    Device handle
\param		[in,out]phLocalDev      Local device handle
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXGetLocalDeviceHandleFromDev(GX_DEV_HANDLE hDevice, GX_LOCAL_DEV_HANDLE* phLocalDev);

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      获取设备流通道数量
\param		[in]hDevice			    设备handle
\param		[in|out]pnDSNum         设备流通道数量
\return     GX_STATUS_SUCCESS				操作成功，没有发生错误
            GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
            GX_STATUS_INVALID_PARAMETER		用户输入的指针为NULL
			GX_STATUS_INVALID_HANDLE		用户传入非法的句柄，或者关闭已经被关闭的设备
            其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Obtain the number of data stream.
\param		[in]hDevice			    Device handle
\param		[in|out]pnDSNum         Data stream number
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXGetDataStreamNumFromDev(GX_DEV_HANDLE hDevice, uint32_t* pnDSNum);

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      获取设备流通道句柄
\param		[in]hDevice			    设备handle
\param		[in]nDSIndex			设备流序号，序号从1 开始
\param		[in|out]phDS            设备流handle
\return     GX_STATUS_SUCCESS				操作成功，没有发生错误
            GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
            GX_STATUS_INVALID_PARAMETER		用户输入的指针为NULL
			GX_STATUS_INVALID_HANDLE		用户传入非法的句柄，或者关闭已经被关闭的设备
            其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Get data stream handle.
\param		[in]hDevice			    Device handle
\param		[in]nDSIndex			Data stream order, starting from 1
\param		[in|out]phDS            Data stream handle
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXGetDataStreamHandleFromDev(GX_DEV_HANDLE hDevice, uint32_t nDSIndex, GX_DS_HANDLE* phDS);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief     获取设备的永久IP信息
\attention 该接口只适用于网络设备
\param     [in]       hDevice                  设备句柄
\param     [in]       pszIP                    设备永久IP字符串地址
\param     [in, out]  pnIPLength               设备永久IP地址字符串长度,单位字节。
\param     [in]       pnIPLength:              用户buffer大小
\param     [out]      pnIPLength:              实际填充大小
\param     [in]       pszSubNetMask            设备永久子网掩码字符串地址
\param     [in, out]  pnSubNetMaskLength       设备永久子网掩码字符串长度
\param     [in]       pnSubNetMaskLength:      用户buffer大小
\param     [out]      pnSubNetMaskLength:      实际填充大小
\param     [in]       pszDefaultGateWay        设备永久网关字符串地址
\param     [in, out]  pnDefaultGateWayLength   设备永久网关字符串长度
\param     [in]       pnDefaultGateWayLength:  用户buffer大小
\param     [out]      pnDefaultGateWayLength:  实际填充大小
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
		   GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
		   上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST

\English:
\brief     Obtain the permanent IP information of the device.
\attention This interface applies only to network devices
\param     [in]       hDevice                  The handle of the device.
\param     [in]       pszIP                    Device permanent IP string address
\param     [in, out]  pnIPLength               The length of the device's permanent IP address string (in bytes).
\param     [in]       pnIPLength:              User buffer size
\param     [out]      pnIPLength:              Actual fill size
\param     [in]       pszSubNetMask            Device persistent subnet mask string address
\param     [in, out]  pnSubNetMaskLength       Device persistent subnet mask string length
\param     [in]       pnSubNetMaskLength:      User buffer size
\param     [out]      pnSubNetMaskLength:      Actual fill size
\param     [in]       pszDefaultGateWay        Device permanent gateway string address
\param     [in, out]  pnDefaultGateWayLength   Device permanent gateway string length
\param     [in]       pnDefaultGateWayLength:  User buffer size
\param     [out]      pnDefaultGateWayLength:  Actual fill size
\return    GX_STATUS_SUCCESS             The operation is successful and no error occurs.
		   GX_STATUS_NOT_INIT_API        The GXInitLib initialization library is not called.
		   GX_STATUS_INVALID_PARAMETER   The input parameter that the user introduces is invalid.
		   The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXGetDevicePersistentIpAddress (GX_DEV_HANDLE hDevice,
                                       char* pszIP,
                                       size_t *pnIPLength,
                                       char* pszSubNetMask,
                                       size_t *pnSubNetMaskLength,
                                       char* pszDefaultGateWay,
                                       size_t *pnDefaultGateWayLength);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief     设置设备的永久IP信息
\attention 该接口只适用于网络设备
\param     [in]     hDevice              设备句柄
\param     [in]     pszIP                设备永久IP字符串，末尾’\0’
\param     [in]     pszSubNetMask        设备永久子网掩码字符串，末尾’\0’
\param     [in]     pszDefaultGateWay    设备永久网关字符串，末尾’\0’
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄，或者关闭已经被关闭的设备
		   其它错误情况请参见GX_STATUS_LIST

\English:
\brief     Set the permanent IP information of the device.
\attention This interface applies only to network devices
\param     [in]     hDevice              The handle of the device.
\param     [in]     pszIP                Device permanent IP string address, end with ’\0’.
\param     [in]     pszSubNetMask        Device persistent subnet mask string address, end with ’\0’.
\param     [in]     pszDefaultGateWay    Device permanent gateway string address, end with ’\0’.
\return    GX_STATUS_SUCCESS             The operation is successful and no error occurs.
		   GX_STATUS_NOT_INIT_API        The GXInitLib initialization library is not called.
		   GX_STATUS_INVALID_HANDLE      The illegal handle that the user introduces, or reclose the device.
		   The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXSetDevicePersistentIpAddress (GX_DEV_HANDLE  hDevice,
									   const char* pszIP,
									   const char* pszSubNetMask,
									   const char* pszDefaultGateWay);


// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      设置远端设备静态（永久）IP地址
\param		[in]   pszDevcieMacAddress  设备MAC地址
\param		[in]   ui32IpConfigFlag     ip配置方式（静态IP、DHCP、LLA、默认方式）
\param		[in]   pszIPAddress         设备IP地址
\param		[in]   pszSubnetMask        子网掩码
\param		[in]   pszDefaultGateway    网关
\param		[in]   pszUserID            用户自定义名称
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER   无效参数
			GX_STATUS_NOT_FOUND_DEVICE    没有找到设备
			GX_STATUS_ERROR               操作失败
			GX_STATUS_INVALID_ACCESS      拒绝访问
			GX_STATUS_TIMEOUT             操作超时
			GC_ERR_IO                     IO通讯错误
			GC_ERR_INVALID_ID             ID无法与资源建立连接

\English:
\brief      Configure the static IP address of the camera.
\param      [in]pszDeviceMacAddress The MAC address of the device.
\param      [in]ui32IpConfigFlag    IP Configuration.
\param      [in]pszIPAddress        The IP address to be set.
\param      [in]pszSubnetMask       The subnet mask to be set.
\param      [in]pszDefaultGateway   The default gateway to be set.
\param      [in]pszUserID           The user-defined name to be set.
\retrun     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER     The parameter is invalid.
			GX_STATUS_NOT_FOUND_DEVICE      Can not found the device.
			GX_STATUS_ERROR                 The operation is failed.
			GX_STATUS_INVALID_ACCESS        Access denied.
			GX_STATUS_TIMEOUT               The operation is timed out.
			GC_ERR_IO                       IO error.
			GC_ERR_INVALID_ID               Invalid ID.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXGigEIpConfiguration(const char* pszDeviceMacAddress,
							 GX_IP_CONFIGURE_MODE emIpConfigMode,
							 const char* pszIpAddress,
							 const char* pszSubnetMask,
							 const char* pszDefaultGateway,
							 const char* pszUserID);

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      ForceIP
\param		[in]   pszDevcieMacAddress  设备MAC地址
\param		[in]   pszIPAddress         设备IP地址
\param		[in]   pszSubnetMask        子网掩码
\param		[in]   pszDefaultGateway    网关
\return		GX_STATUS_SUCCESS				操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER		无效参数
			GX_STATUS_NOT_FOUND_DEVICE		没有找到设备
			GX_STATUS_ERROR					操作失败
			GX_STATUS_INVALID_ACCESS		拒绝访问
			GX_STATUS_TIMEOUT				操作超时
			GC_ERR_IO						IO通讯错误
			GC_ERR_INVALID_ID				ID无法与资源建立连接

\English:
\brief      Execute the Force IP.
\param      [in]pszDeviceMacAddress The MAC address of the device.
\param      [in]pszIPAddress        The IP address to be set.
\param      [in]pszSubnetMask       The subnet mask to be set.
\param      [in]pszDefaultGateway   The default gateway to be set.
\retrun     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER     The parameter is invalid.
			GX_STATUS_NOT_FOUND_DEVICE      Can not found the device.
			GX_STATUS_ERROR                 The operation is failed.
			GX_STATUS_INVALID_ACCESS        Access denied.
			GX_STATUS_TIMEOUT               The operation is timed out.
			GC_ERR_IO                       IO error.
			GC_ERR_INVALID_ID               Invalid ID.
*/
// ---------------------------------------------------------------------------
GX_API GXGigEForceIp(const char* pszDeviceMacAddress,
							 const char* pszIpAddress,
							 const char* pszSubnetMask,
							 const char* pszDefaultGateway);

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      重连/复位
\param		[in]   pszDevcieMacAddress  设备MAC地址
\param		[in]   ui32FeatureInfo      重置设备模式
\return     GX_STATUS_SUCCESS				操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER		无效参数
			GX_STATUS_NOT_FOUND_DEVICE		没有找到设备
			GX_STATUS_ERROR					操作失败
			GX_STATUS_INVALID_ACCESS		拒绝访问
			GX_STATUS_TIMEOUT				操作超时
			GC_ERR_IO						IO通讯错误
			GC_ERR_INVALID_ID				ID无法与资源建立连接

\English:
\brief      Reconnection/Reset
\param      [in]pszDeviceMacAddress The MAC address of the device.
\param      [in]ui32FeatureInfo     Reconnection mode.
\retrun     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER     The parameter is invalid.
			GX_STATUS_NOT_FOUND_DEVICE      Can not found the device.
			GX_STATUS_ERROR                 The operation is failed.
			GX_STATUS_INVALID_ACCESS        Access denied.
			GX_STATUS_TIMEOUT               The operation is timed out.
			GC_ERR_IO                       IO error.
			GC_ERR_INVALID_ID               Invalid ID.
*/
// ---------------------------------------------------------------------------
GX_API GXGigEResetDevice(const char* pszDeviceMacAddress, GX_RESET_DEVICE_MODE ui32FeatureInfo);

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      获取设备流PayloadSize
\param		[in]hDStream			   设备流handle
\param		[in|out]punPacketSize      PayLoadSize指针
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
            GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
            GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
            其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Get device stream PayloadSize.
\param		[in]hDStream			   Device stream handle.
\param		[in|out]punPacketSize      PayLoadSize pointer
\return     GX_STATUS_SUCCESS             The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API        The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER   The pointer that the user input is NULL.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXGetPayLoadSize(GX_DS_HANDLE hDStream, uint32_t* punPacketSize);

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      保存用户参数组
\param		[in]hPort               Handle句柄
\param		[in]strFileName         保存用户参数组的文件路径
\return     GX_STATUS_SUCCESS				操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER		用户输入的指针为NULL
			GX_STATUS_INVALID_HANDLE		用户传入非法的句柄，或者关闭已经被关闭的设备
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Save the user parameter group
\param		[in]hPort               Handle
\param		[in]strFileName         File path to save user parameter groups
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXFeatureSave(GX_PORT_HANDLE hPort, const char* strFileName);

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      加载用户参数组
\param		[in]hPort               Handle句柄
\param		[in]strFileName         加载用户参数组的文件路径
\param		[in]bVerify             如果此值为true，所有导入进去的值将会被读出进行校验是否一致
\return     GX_STATUS_SUCCESS				操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER		用户输入的指针为NULL
			GX_STATUS_INVALID_HANDLE		用户传入非法的句柄，或者关闭已经被关闭的设备
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Load the user parameter group
\param		[in]hPort               Handle
\param		[in]strFileName         File path for loading user parameter groups
\param		[in]bVerify             If the value is true, All imported values will be read out for consistency verification
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXFeatureLoad(GX_PORT_HANDLE hPort, const char* strFileName, bool bVerify);

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      保存用户参数组
\param		[in]hPort               Handle句柄
\param		[in]strFileName         保存用户参数组的文件路径（宽字节）
\return     GX_STATUS_SUCCESS				操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER		用户输入的指针为NULL
			GX_STATUS_INVALID_HANDLE		用户传入非法的句柄，或者关闭已经被关闭的设备
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Save the user parameter group
\param		[in]hPort               Handle
\param		[in]strFileName         File path to save user parameter groups(Wide byte)
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXFeatureSaveW(GX_PORT_HANDLE hPort, const wchar_t* strFileName);

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      加载用户参数组
\param		[in]hPort               Handle句柄
\param		[in]strFileName         加载用户参数组的文件路径（宽字节）
\param		[in]bVerify             如果此值为true，所有导入进去的值将会被读出进行校验是否一致
\return     GX_STATUS_SUCCESS				操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER		用户输入的指针为NULL
			GX_STATUS_INVALID_HANDLE		用户传入非法的句柄，或者关闭已经被关闭的设备
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Load the user parameter group
\param		[in]hPort               Handle
\param		[in]strFileName         File path for loading user parameter groups(Wide byte)
\param		[in]bVerify             If the value is true, All imported values will be read out for consistency verification
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXFeatureLoadW(GX_PORT_HANDLE hPort, const wchar_t* strFileName, bool bVerify);

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      获取节点的读写属性
\param		[in]hPort					Handle句柄
\param		[in]strName					节点名称
\param		[in|out]pemAccessMode      节点读写属性
\return     GX_STATUS_SUCCESS				操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER		用户输入的指针为NULL
			GX_STATUS_INVALID_HANDLE		用户传入非法的句柄，或者关闭已经被关闭的设备
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Get the read and write attributes of the node.
\param		[in]hPort					The handle.
\param		[in]strName					Node name.
\param		[in|out]pemAccessMode		Read and write attributes of the node.
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXGetNodeAccessMode(GX_PORT_HANDLE hPort, const char* strName, GX_NODE_ACCESS_MODE* pemAccessMode);

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      获取整型节点信息
\param		[in]       hPort              Handle句柄
\param		[in]       strName            节点名称
\param		[in|out]   pstIntValue        整型节点信息
\return     GX_STATUS_SUCCESS				操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER		用户输入的指针为NULL
			GX_STATUS_INVALID_HANDLE		用户传入非法的句柄，或者关闭已经被关闭的设备
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Get int value information.
\param		[in]hPort				The handle.
\param		[in]strName				Node name.
\param		[in|out]pstIntValue     Point to the pointer of the current returned value.
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXGetIntValue(GX_PORT_HANDLE hPort, const char* strName, GX_INT_VALUE* pstIntValue);

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      设置整型节点值
\param		[in]hPort              Handle句柄
\param		[in]strName            节点名称
\param		[in|out]i64Value       整型节点值
\return     GX_STATUS_SUCCESS				操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER		用户输入的指针为NULL
			GX_STATUS_INVALID_HANDLE		用户传入非法的句柄，或者关闭已经被关闭的设备
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Set int value information.
\param		[in]hPort				The handle.
\param		[in]strName				Node name.
\param		[in|out]i64Value        Integer node value
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXSetIntValue(GX_PORT_HANDLE hPort, const char* strName, int64_t i64Value);

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      获取枚举型节点信息
\param		[in]hPort				Handle句柄
\param		[in]strName				节点名称
\param		[in|out]pstEnumValue    枚举类型节点信息（数值型值）
\return     GX_STATUS_SUCCESS				操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER		用户输入的指针为NULL
			GX_STATUS_INVALID_HANDLE		用户传入非法的句柄，或者关闭已经被关闭的设备
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      To get the current enumeration value
\param		[in]hPort				The handle.
\param		[in]strName				Node name.
\param		[in|out]pstEnumValue    The value of the enumeration type.(numeric value)
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXGetEnumValue(GX_PORT_HANDLE hPort, const char* strName, GX_ENUM_VALUE* pstEnumValue);

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      设置枚举型节点值
\param		[in]hPort           Handle句柄
\param		[in]strName         节点名称
\param		[in]i64Value        枚举类型节点值
\return     GX_STATUS_SUCCESS				操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER		用户输入的指针为NULL
			GX_STATUS_INVALID_HANDLE		用户传入非法的句柄，或者关闭已经被关闭的设备
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Sets the value of an enumerated node
\param		[in]hPort           The handle.
\param		[in]strName         Node name.
\param		[in]i64Value        Enumerate type node values
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXSetEnumValue(GX_PORT_HANDLE hPort, const char* strName, int64_t i64Value);

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      设置枚举型节点值
\param		[in]hPort           Handle句柄
\param		[in]strName         节点名称
\param		[in]strValue        枚举类型节点值（字符串型值）
\return     GX_STATUS_SUCCESS				操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER		用户输入的指针为NULL
			GX_STATUS_INVALID_HANDLE		用户传入非法的句柄，或者关闭已经被关闭的设备
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Sets the value of an enumerated node
\param		[in]hPort           The handle.
\param		[in]strName         Node name.
\param		[in]strValue        Enumerate type node values.(String value)
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXSetEnumValueByString(GX_PORT_HANDLE hPort, const char* strName, const char* strValue);

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      获取浮点型节点信息
\param		[in]hPort				Handle句柄
\param		[in]strName				节点名称
\param		[in|out]pstFloatValue   浮点类型节点信息
\return     GX_STATUS_SUCCESS				操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER		用户输入的指针为NULL
			GX_STATUS_INVALID_HANDLE		用户传入非法的句柄，或者关闭已经被关闭的设备
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Get the value of float type.
\param		[in]hPort				The handle.
\param		[in]strName				Node name.
\param		[in|out]pstFloatValue   Float node information.
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXGetFloatValue(GX_PORT_HANDLE hPort, const char* strName, GX_FLOAT_VALUE* pstFloatValue);

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      设置浮点型节点值
\param		[in]hPort           Handle句柄
\param		[in]strName         节点名称
\param		[in]dValue          浮点类型节点值
\return     GX_STATUS_SUCCESS				操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER		用户输入的指针为NULL
			GX_STATUS_INVALID_HANDLE		用户传入非法的句柄，或者关闭已经被关闭的设备
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Set the value of float type.
\param		[in]hPort           The handle.
\param		[in]strName         Node name.
\param		[in]dValue          Floating point node value
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXSetFloatValue(GX_PORT_HANDLE hPort, const char* strName, double dValue);

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      获取布尔型节点信息
\param		[in]hPort              Handle句柄
\param		[in]strName            节点名称
\param		[in|out]pbValue        布尔类型节点信息
\return     GX_STATUS_SUCCESS				操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER		用户输入的指针为NULL
			GX_STATUS_INVALID_HANDLE		用户传入非法的句柄，或者关闭已经被关闭的设备
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Get the value of bool type.
\param		[in]hPort              The handle.
\param		[in]strName            Node name.
\param		[in|out]pbValue        Boolean node information
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXGetBoolValue(GX_PORT_HANDLE hPort, const char* strName, bool* pbValue);

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      设置布尔型节点值
\param		[in]hPort           Handle句柄
\param		[in]strName         节点名称
\param		[in]bValue          布尔类型节点值
\return     GX_STATUS_SUCCESS				操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER		用户输入的指针为NULL
			GX_STATUS_INVALID_HANDLE		用户传入非法的句柄，或者关闭已经被关闭的设备
			其它错误情况请参见GX_STATUS_LIST


\English:
\brief      Set the value of bool type.
\param		[in]hPort           The handle.
\param		[in]strName         Node name.
\param		[in]bValue          Boolean node value
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXSetBoolValue(GX_PORT_HANDLE hPort, const char* strName, bool bValue);

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      获取字符串型节点信息
\param		[in]hPort					Handle句柄
\param		[in]strName					节点名称
\param		[in|out]pstStringValue		字符串类型节点数据
\return     GX_STATUS_SUCCESS				操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER		用户输入的指针为NULL
			GX_STATUS_INVALID_HANDLE		用户传入非法的句柄，或者关闭已经被关闭的设备
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Get the content of the string type.
\param		[in]hPort					The handle.
\param		[in]strName					Node name.
\param		[in|out]pstStringValue		String type node data.
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXGetStringValue(GX_PORT_HANDLE hPort, const char* strName, GX_STRING_VALUE* pstStringValue);

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      设置字符串类型节点值
\param		[in]hPort				Handle句柄
\param		[in]strName				节点名称
\param		[in]strValue			字符串类型节点值
\return     GX_STATUS_SUCCESS				操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER		用户输入的指针为NULL
			GX_STATUS_INVALID_HANDLE		用户传入非法的句柄，或者关闭已经被关闭的设备
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Set a string type node value
\param		[in]hPort				The handle.
\param		[in]strName				Node name.
\param		[in]strValue			String type Node value
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXSetStringValue(GX_PORT_HANDLE hPort, const char* strName, const char* strValue);

//string 长度

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      发送命令
\param		[in]hPort				Handle句柄
\param		[in]strName				节点名称
\return     GX_STATUS_SUCCESS				操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER		用户输入的指针为NULL
			GX_STATUS_INVALID_HANDLE		用户传入非法的句柄，或者关闭已经被关闭的设备
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Send the command.
\param		[in]hPort				The handle.
\param		[in]strName				Node name.
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXSetCommandValue(GX_PORT_HANDLE hPort, const char* strName);

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      获取寄存器型节点数据长度
\param		[in]hPort              Handle句柄
\param		[in]strName            节点名称
\param		[in|out]pnSize         保存数据长度
\return     GX_STATUS_SUCCESS				操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER		用户输入的指针为NULL
			GX_STATUS_INVALID_HANDLE		用户传入非法的句柄，或者关闭已经被关闭的设备
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Gets the data length of a register node
\param		[in]hPort              The handle.
\param		[in]strName            Node name.
\param		[in|out]pnSize         Save the data length.
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXGetRegisterLength(GX_PORT_HANDLE hPort, const char* strName, size_t* pnSize);

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      获取寄存器型节点值
\param		[in]hPort              Handle句柄
\param		[in]strName            节点名称
\param		[in|out]pBuffer        寄存器值数据
\param		[in|out]pnSize         数据长度
\return     GX_STATUS_SUCCESS				操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER		用户输入的指针为NULL
			GX_STATUS_INVALID_HANDLE		用户传入非法的句柄，或者关闭已经被关闭的设备
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Get register type node values.
\param		[in]hPort              The handle.
\param		[in]strName            Node name.
\param		[in|out]pBuffer        Register value data
\param		[in|out]pnSize         Data length
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXGetRegisterValue(GX_PORT_HANDLE hPort, const char* strName, uint8_t* pBuffer, size_t* pnSize);

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      设置寄存器值
\param		[in]hPort           Handle句柄
\param		[in]strName         节点名称
\param		[in]pBuffer         寄存器数据
\param		[in]nSize           寄存器数据长度
\return     GX_STATUS_SUCCESS				操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER		用户输入的指针为NULL
			GX_STATUS_INVALID_HANDLE		用户传入非法的句柄，或者关闭已经被关闭的设备
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Set register value
\param		[in]hPort           The handle.
\param		[in]strName         Node name.
\param		[in]pBuffer         Register data
\param		[in]nSize           Register data length
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXSetRegisterValue(GX_PORT_HANDLE hPort, const char* strName, uint8_t* pBuffer, size_t nSize);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      读用户指定寄存器的值
\param		[in]hPort           句柄
\param		[in]ui64Address     寄存器地址
\param		[out]pBuffer        返回寄存器的值，不能为NULL
\param		[in, out]piSize     [in]用户申请的Buffer大小
								[out]成功读取寄存器的值后，返回实际大小
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST

\English:
\brief      Reads the value of a user-specified register
\param		[in]hPort           The handle
\param		[in]ui64Address     Register address
\param		[out]pBuffer        Returns the value of the register, which cannot be NULL
\param		[in, out]piSize     [in]Buffer size applied by the user
								[out]After successfully reading the value of the register, the actual size is returned
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXReadPort(GX_PORT_HANDLE hPort, uint64_t ui64Address, void* pBuffer, size_t* piSize);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      向用户指定的寄存器中写入用户给定的数据
\param		[in]hPort           句柄
\param		[in]ui64Address     寄存器地址
\param		[in]pBuffer         寄存器的值，不能为NULL
\param		[in, out]piSize     [in]用户要写入的Buffer长度
								[out]返回实际写入寄存器的长度
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST

\English:
\brief      Writes the data given by the user to a register specified by the user
\param		[in]hPort           The handle
\param		[in]ui64Address     Register address
\param		[in]pBuffer         Register value, which cannot be NULL
\param		[in, out]piSize     [in]The Buffer length to be written by the user
								[out]Returns the length of the actual register written
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXWritePort(GX_PORT_HANDLE hPort, uint64_t ui64Address, const void* pBuffer, size_t* piSize);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      批量读用户指定寄存器的值（仅限命令值为4字节长度的寄存器,其余类型不保证数据有效性）
\param		[in]hPort           句柄
\param		[in|out]pstEntries  [in]批量读取寄存器的地址及值
								[out]读取到对应寄存器的数据
\param		[in, out]piSize     [in]读取设备寄存器的个数
								[out]成功读取寄存器的个数
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST

\English:
\brief      Batch read the value of the user specified register (only the command value is 4 bytes long register, other types do not guarantee data validity)
\param		[in]hPort           The handle
\param		[in|out]pstEntries  [in]Read register addresses and values in batches
								[out]The data is read from the corresponding register
\param		[in, out]piSize     [in]Read the number of device registers
								[out]The number of registers read successfully
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXReadPortStacked(GX_PORT_HANDLE hPort, GX_REGISTER_STACK_ENTRY* pstEntries, size_t *piSize);


//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      批量向用户指定的寄存器中写入用户给定的数据（仅限命令值为4字节长度的寄存器）
            调用当前接口后，使用GXGetEnum、GXGetInt、GXGetBool等接口获取到的节点值为修改前值
            可使用GXReadPort接口获取最新的寄存器值
\param		[in]hPort           句柄
\param		[in]pstEntries      [in]批量写寄存器的地址及值
\param		[in|out]piSize      [in]设置设备寄存器的个数
								[out]成功写寄存器的个数
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST

\English:
\brief      Write user-specified data in batches to user-specified registers (only registers with 4-byte command values)
			After the current interface is invoked, the node value obtained by using interfaces such as GXGetEnum, GXGetInt, and GXGetBool is the previous value
			The latest register values can be obtained using the GXReadPort interface
\param		[in]hPort           The handle
\param		[in]pstEntries      [in]Batch write register addresses and values
\param		[in|out]piSize      [in]Sets the number of device registers
								[out]The number of successful register writes
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXWritePortStacked(GX_PORT_HANDLE hPort, const GX_REGISTER_STACK_ENTRY* pstEntries, size_t *piSize);

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief       获取最优包长值
\param		[in]hDevice				设备句柄
\param		[out]punPacketSize		最优包长值
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_TIMEOUT             操作超时
			GC_ERR_IO                     IO通讯错误
			GX_STATUS_INVALID_PARAMETER   无效参数
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST

\English:
\brief      Gets the optimal packet length value
\param		[in]hDevice				The handle of the device.
\param		[out]punPacketSize		Optimal packet length
\return     GX_STATUS_SUCCESS             The operation is successful and no error occurs.
			GX_STATUS_TIMEOUT             Operation timeout
			GC_ERR_IO                     IO communication error
			GX_STATUS_INVALID_PARAMETER   The input parameter that the user introduces is invalid.
			GX_STATUS_INVALID_HANDLE      The illegal handle that the user introduces, or reclose the device.
			GX_STATUS_NOT_IMPLEMENTED     Currently unsupported features
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API  GXGetOptimalPacketSize (GX_DEV_HANDLE hDevice, uint32_t* punPacketSize);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      用户设置采集buffer个数
\param		[in]hDevice         设备句柄
\param		[in]nBufferNum      用户设置的buffer个数
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_INVALID_PARAMETER    输入参数无效
			上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST

\English:
\brief      Set the number of the acquisition buffers.
\param		[in]hDevice         The handle of the device.
\param		[in]nBufferNum      The number of the acquisition buffers that the user sets.
\return     GX_STATUS_SUCCESS             The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API        The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE      The handle that the user introduces is illegal.
			GX_STATUS_INVALID_PARAMETER    The input parameter that the user introduces is invalid.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXSetAcqusitionBufferNumber(GX_DEV_HANDLE hDevice, uint64_t nBufferNum);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      注册采集回调函数
\attention  必须在发送开采命令之前注册采集回调函数
\param		[in]hDevice			设备句柄
\param		[in]pUserParam		用户私有数据
\param		[in]callBackFun		用户注册的回调函数
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_INVALID_PARAMETER   用户传入指针为NULL
			GX_STATUS_INVALID_CALL        发送开采命令后，不能注册采集回调函数
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Register the capture callback function
\attention  The collection callback function must be registered before the mining command is sent
\param		[in]hDevice			The handle of the device.
\param		[in]pUserParam		User private data
\param		[in]callBackFun		User registration callback function
\return		GX_STATUS_SUCCESS             The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API        The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE      The handle that the user introduces is illegal.
			GX_STATUS_INVALID_PARAMETER   The input parameter that the user introduces is invalid.
			GX_STATUS_INVALID_CALL        After sending the start acquisition command, the user can not register the capture callback function.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXRegisterCaptureCallback  (GX_DEV_HANDLE hDevice, void *pUserParam, GXCaptureCallBack callBackFun);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      注销采集回调函数
\attention  必须在发送停采命令之后注销采集回调函数
\param		[in]hDevice				设备句柄
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_INVALID_CALL        发送停采命令之前，不能注销采集回调函数
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Unregister the capture callback function
\attention  The collection callback function must be deregistered after the stop mining command is sent
\param		[in]hDevice				The handle of the device.
\return		GX_STATUS_SUCCESS             The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API        The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE      The handle that the user introduces is illegal.
			GX_STATUS_INVALID_CALL        The collection callback function cannot be deregistered before sending the stop mining command
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXUnregisterCaptureCallback(GX_DEV_HANDLE hDevice);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      直接获取一帧图像
\attention  如果用户已注册采集回调函数，调用此接口会报错GX_STATUS_INVALID_CALL
\param		[in]hDevice        设备句柄
\param		[in|out]pFrameData 图像信息结构体指针
\param		[in]nTimeout       超时时间
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_INVALID_CALL        发送停采命令之前，不能注销采集回调函数
			GX_STATUS_INVALID_PARAMETER   用户传入图像地址指针为NULL
			GX_STATUS_NEED_MORE_BUFFER    用户分配的图像buffer小于实际需要的大小
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Get a frame of image directly
\attention  If the user has registered the collection callback function, calling this interface results in an error GX_STATUS_INVALID_CALL
\param		[in]hDevice			The handle of the device.
\param		[in|out]pFrameData	Pointer to image information structure
\param		[in]nTimeout		Timeout period
\return		GX_STATUS_SUCCESS             The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API        The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE      The handle that the user introduces is illegal.
			GX_STATUS_INVALID_CALL        After registering the capture callback function, the user calls the GXGetImage to get image.
			GX_STATUS_INVALID_PARAMETER   User incoming image address pointer is NULL.
			GX_STATUS_NEED_MORE_BUFFER    The image buffer assigned by the user is smaller than it needs to be
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXGetImage(GX_DEV_HANDLE hDevice, GX_FRAME_DATA *pFrameData, uint32_t nTimeout);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      清空采集输出队列
\attention  如果用户处理图像的速度较慢，库内会残留上次采集过程的缓存图像，特别在触发模式下，
            用户发送完触发之后，获取到的是旧图，如果用户想获取到当前触发对应的图像，需要在
		    发送触发之前调用GXFlushQueue接口，先清空图像输出队列。
\param		[in]hDevice        设备句柄
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Clear the collection output queue
\attention  If the user processes the images too slow, the image of last acquisition may be remained in the queue.
			Especially in the trigger mode, after the user send the trigger signal, and get the old image (last image).
			If you want to get the current image that corresponding to trigger signal, you should call the
			GXFlushQueue interface before sending the trigger signal to empty the image output queue.
\param		[in]hDevice				The handle of the device.
\return		GX_STATUS_SUCCESS             The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API        The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE      The handle that the user introduces is illegal.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXFlushQueue(GX_DEV_HANDLE hDevice);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      注册事件回调函数
\attention  设备事件比如，掉线事件、曝光结束等，这些事件都可以通过这个接口的回调方式传出，
            用户不需要获取事件的时候调用GXUnregisterEventCallback接口注销回调函数
\param		[in]hDevice			设备句柄
\param		[in]pUserParam		用户私有数据
\param		[in]callBackFun		用户注册的回调函数
\param		[out]pHCallBack		掉线回调函数句柄，此句柄用来注销回调函数使用
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_INVALID_PARAMETER   用户传入回调函数非法或者传入事件类型非法
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Register the event callback function
\attention  Exposure device events, for example, offline events, such as end, these events can be covered by the interface mode of the callback,
			users do not need to access the event called when GXUnregisterEventCallback interface cancelled the callback function
\param		[in]hDevice			The handle of the device.
\param		[in]pUserParam		User private parameter.
\param		[in]callBackFun		User registration callback function
\param		[out]pHCallBack			The handle of offline callback function, the handle is used for unregistering the callback function.
\return		GX_STATUS_SUCCESS             The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API        The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE      The handle that the user introduces is illegal.
			GX_STATUS_INVALID_PARAMETER   The callback function or event type passed by the user is invalid
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXRegisterDeviceOfflineCallback    (GX_DEV_HANDLE hDevice,
										   void* pUserParam,
										   GXDeviceOfflineCallBack callBackFun,
										   GX_EVENT_CALLBACK_HANDLE *pHCallBack);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      注销事件回调函数
\attention  设备事件比如，掉线事件、曝光结束等，这些事件都可以通过这个接口的回调方式传出，
            用户不需要获取事件的时候调用GXUnregisterEventCallback接口注销回调函数
\param		[in]hDevice			设备句柄
\param		[in]pHCallBack		掉线回调函数句柄
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_INVALID_CALL        发送停采命令之前，不能注销采集回调函数
			GX_STATUS_INVALID_PARAMETER   用户传入事件类型非法
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Unregister event handle callback function
\attention  Device events such as drop events, end of exposure, etc., these events can be transmitted through the callback mode of this interface.
			Called when users do not need to obtain events GXUnregisterEventCallback interface cancelled the callback function
\param		[in]hDevice				The handle of the device.
\param		[in]pHCallBack			The handle of offline callback function
\return		GX_STATUS_SUCCESS             The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API        The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE      The handle that the user introduces is illegal.
			GX_STATUS_INVALID_CALL        The collection callback function cannot be deregistered before sending the stop mining command
			GX_STATUS_INVALID_PARAMETER   The callback function or event type passed by the user is invalid
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXUnregisterDeviceOfflineCallback  (GX_DEV_HANDLE hDevice, GX_EVENT_CALLBACK_HANDLE  hCallBack);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      清空事件输出队列
\attention  库内部事件数据的接收和处理采用缓存机制，如果用户接收、处理事件的速度慢于事件产生的速度，
            事件数据就会在库内积累，会影响用户获取实时事件数据。如果用户想获取实时事件数据，需要先
			调用GXFlushEvent接口清空事件缓存数据。此接口一次性清空所有事件数据。
\param		[in]hDevice        设备句柄
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Empty the event output queue
\attention  The library internal event data receiving and processing using caching mechanism, if the user
			receiving, processing event speed is slower than the event generates, then the event data will be
			accumulated in the library, it will affect the the user to get real-time event data. If you want to get
			the real-time event data, you need to call the GXFlushEvent interface to clear the event cache data.
			This interface empties all the event data at once.
\param		[in]hDevice			The handle of the device.
\return		GX_STATUS_SUCCESS             The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API        The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE      The handle that the user introduces is illegal.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXFlushEvent               (GX_DEV_HANDLE hDevice);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief     获取当前事件队列里面的事件个数
\param     [in]hDevice        设备句柄
\param     [in]pnEventNum     事件个数指针
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
		   GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
           GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
           GX_STATUS_INVALID_PARAMETER   用户传入pnEventNum为NULL指针
           其它错误情况请参见GX_STATUS_LIST

\English:
\brief     Get the number of the events in the current remote device event queue cache.
\param     [in]hDevice        The handle of the device.
\param     [in]pnEventNum     The pointer of event number.
\return    GX_STATUS_SUCCESS             The operation is successful and no error occurs.
		   GX_STATUS_NOT_INIT_API        The GXInitLib initialization library is not called.
		   GX_STATUS_INVALID_HANDLE      The handle that the user introduces is illegal.
		   GX_STATUS_INVALID_PARAMETER   The pointer that the user input is NULL.
		   The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXGetEventNumInQueue       (GX_DEV_HANDLE hDevice, uint32_t *pnEventNum);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      注册属性更新回调函数
\attention  用户可通过此接口获取事件数据，详见示例程序
\param		[in]hDevice          设备句柄
\param		[in]pUserParam       用户私有数据
\param		[in]callBackFun      用户注册的回调函数
\param		[in]strfeatureName   节点名称
\param		[out]pHCallBack      回调函数句柄
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_INVALID_PARAMETER   用户传入回调函数非法
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Register property update callback function.
\attention  Users can obtain event data through this interface, as described in the example program
\param		[in]hDevice          The handle of the device
\param		[in]pUserParam       User Private Parameters
\param		[in]callBackFun      User registration callback function
\param		[in]strfeatureName   Node name
\param		[out]pHCallBack      Callback function handle
\return		GX_STATUS_SUCCESS             The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API        The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE      The handle that the user introduces is illegal.
			GX_STATUS_INVALID_PARAMETER   The pointer that the user input is NULL.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXRegisterFeatureCallbackByString  (GX_PORT_HANDLE hPort,
								   void* pUserParam,
								   GXFeatureCallBackByString  callBackFun,
								   const char*  strfeatureName,
								   GX_FEATURE_CALLBACK_BY_STRING_HANDLE *pHCallBack);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      注销属性更新回调函数
\attention  与GXRegisterFeatureCallback配套使用，每次注册都必须有相应的注销与之对应
\param		[in]hDevice				设备句柄
\param		[in]strfeatureName		节点名称
\param		[out]pHCallBack			回调函数句柄
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Unregister device attribute update callback function.
\attention  Used in conjunction with GXRegisterFeatureCallback, each registration must have a corresponding cancellation corresponding to it.
\param		[in]hDevice				The handle of the device
\param		[in]strfeatureName		Node name
\param		[out]pHCallBack			Callback function handle
\return		GX_STATUS_SUCCESS             The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API        The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE      The handle that the user introduces is illegal.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXUnregisterFeatureCallbackByString(GX_PORT_HANDLE  hPort,
                                    const char* strfeatureName,
                                    GX_FEATURE_CALLBACK_BY_STRING_HANDLE  hCallBack);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      导出相机当前参数到配置文件（ANSI接口）
\param		[in]hDevice         设备句柄
\param		[in]pszFilePath     配置文件输出路径
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST

\English:
\brief      Export the current parameter of the camera to the configuration file.(ANSI)
\param      [in]hDevice             The handle of the device.
\param      [in]pszFilePath         The path of the configuration file that to be generated.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXExportConfigFile (GX_DEV_HANDLE hDevice, const char * pszFilePath);


//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      导出相机当前参数到配置文件（UNICODE接口）
\param		[in]hDevice         设备句柄
\param		[in]pszFilePath     配置文件输出路径（wchar_t类型）
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST

\English:
\brief		(unicode interface)
\param		[in]hDevice
\param		[in]pszFilePath
\return     GX_STATUS_SUCCESS
			GX_STATUS_NOT_INIT_API
			GX_STATUS_INVALID_HANDLE
			Other status refer to GX_STATUS_LIST
*/
//----------------------------------------------------------------------------------
GX_API GXExportConfigFileW(GX_DEV_HANDLE hDevice, const wchar_t * pszFilePath);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      将配置文件中参数导入到相机（ANSI接口）
\param		[in]hDevice         设备句柄
\param		[in]pszFilePath     配置文件路径
\param		[in]bVerify         如果此值为true，所有导入进去的值将会被读出进行校验是否一致
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST

\English:
\brief      Import the configuration file for the camera.(ANSI)
\param      [in]hDevice             The handle of the device.
\param      [in]pszFilePath         The path of the configuration file.
\param      [in]bVerify             If bVerify is true, all imported values will be read out to verify consistency.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
#ifndef __cplusplus
GX_API GXImportConfigFile(GX_DEV_HANDLE hDevice, const char * pszFilePath, bool bVerify);
#else
GX_API GXImportConfigFile(GX_DEV_HANDLE hDevice, const char * pszFilePath, bool bVerify = false);
#endif

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      将配置文件中参数导入到相机（UNICODE接口）
\param		[in]hDevice         设备句柄
\param		[in]pszFilePath     配置文件路径
\param		[in]bVerify         如果此值为true，所有导入进去的值将会被读出进行校验是否一致
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST

\English:
\brief		(unicode interface)
\param		[in]hDevice
\param		[in]pszFilePath
\param		[in]bVerify
\return     GX_STATUS_SUCCESS
			GX_STATUS_NOT_INIT_API
			GX_STATUS_INVALID_HANDLE
			Other status refer to GX_STATUS_LIST
*/
//----------------------------------------------------------------------------------
#ifndef __cplusplus
GX_API GXImportConfigFileW(GX_DEV_HANDLE hDevice, const wchar_t * pszFilePath, bool bVerify);
#else
GX_API GXImportConfigFileW(GX_DEV_HANDLE hDevice, const wchar_t * pszFilePath, bool bVerify = false);
#endif

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      采集之后可通过此接口获取一张图片
\attention  无
\param		[in]hDevice    设备句柄
\param      [out]ppFrameBuffer      待放回 GxIAPI 库的图像数据 Buf 指针
\param      [in]nTimeOut            超时时间单元毫秒
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
			GX_STATUS_INVALID_ACCESS      当前不可访问
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      After starting the acquisition, an image (zero copy) can be acquired through this interface.
\param      [in]hDevice             The handle of the device.
\param      [out]ppFrameBuffer      Address pointer of image data output by the interface.
\param      [in]nTimeOut            Take timeout time (unit: ms).
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
            GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
            GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
            GX_STATUS_INVALID_PARAMETER     The pointer that the user introduces is NULL.
            GX_STATUS_INVALID_CALL          Acquisition is not started or the callback is registered. It
                                            is not allowed to call the interface.
            GX_STATUS_TIMEOUT               Acquire image timeout error.
            GX_STATUS_ERROR                 Unspecified internal errors that are not expected to occur.
            The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXDQBuf (GX_DEV_HANDLE hDevice,
                PGX_FRAME_BUFFER *ppFrameBuffer,
                uint32_t nTimeOut);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      在开始采集之后，通过此接口可以将图像数据 Buf 放回 GxIAPI 库，继续用于采集。
\attention  无
\param		[in]hDevice    设备句柄
\param      [out]ppFrameBuffer      待放回 GxIAPI 库的图像数据 Buf 指针
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
			GX_STATUS_INVALID_ACCESS      当前不可访问
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      After the acquisition is started, the image data buffer can be placed back into the GxIAPI
            library through this interface and continue to be used for acquisition.
\param      [in]hDevice             The handle of the device.
\param      [in]pFrameBuffer        Image data buffer pointer to be placed back into the GxIAPI library.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
            GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
            GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
            GX_STATUS_INVALID_PARAMETER     The pointer that the user introduces is NULL.
            GX_STATUS_INVALID_CALL          Acquisition is not started or the callback is registered. It
                                           is not allowed to call the interface.
            The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXQBuf (GX_DEV_HANDLE hDevice, PGX_FRAME_BUFFER pFrameBuffer);

//----------------------------------------------------------------------------------
/**
\brief    Register the extern buffer for grab.
\return   void
*/
//----------------------------------------------------------------------------------

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      注册缓冲区
\param		[in]hDevice			   		  设备handle
\param		[in]pBuffer		         	  注册buffer中的pBuffer
\param		[in]nSize      	    	  	  注册buffer的大小
\param		[in]pUserParam      	      用户参数
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
            GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
            GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
            其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Register the extern buffer.
\param		[in]hDevice			   		  The handle of the device
\param		[in]pBuffer      	    	  The Register Buffer
\param		[in]nSize      	    	  	  The Register Buffer size
\param		[in]pUserParam      	      The User Parameter
\return     GX_STATUS_SUCCESS             The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API        The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER   The pointer that the user input is NULL.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXRegisterBuffer(GX_DEV_HANDLE hDevice, void* pBuffer, size_t nSize, void* pUserParam);

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      注销缓冲区
\param		[in]hDevice			   		设备handle
\param		[in]pBuffer      	     注册buffer中的pBuffer
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
            GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
            GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
            其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Unregister the extern buffer.
\param		[in]hDevice			   		The handle of the device
\param		[in]pBuffer 	     	    The RegisterBuffer pBuffer
\return     GX_STATUS_SUCCESS             The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API        The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER   The pointer that the user input is NULL.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXUnRegisterBuffer(GX_DEV_HANDLE hDevice, void* pBuffer);

#ifndef _WIN32
//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      开始采集包括流采集和设备采集
\attention  无
\param		[in]hDevice    设备句柄
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
			GX_STATUS_INVALID_ACCESS      当前不可访问
			其它错误情况请参见GX_STATUS_LIST 

\English:
\brief      Start acquisition, including stream acquisition and device acquisition.
\param      [in]hDevice             The handle of the device.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
            GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
            GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
            GX_STATUS_INVALID_ACCESS        Device access mode error.
            GX_STATUS_ERROR                 Unspecified internal errors that are not expected to occur.
            The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXStreamOn (GX_DEV_HANDLE hDevice);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      停止采集包括流采集和设备采集
\attention  无
\param		[in]hDevice    设备句柄
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
			GX_STATUS_INVALID_ACCESS      当前不可访问
			其它错误情况请参见GX_STATUS_LIST 

\English:
\brief      Stop acquisition, including stop stream acquisition and stop device acquisition.
\param      [in]hDevice             The handle of the device.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
            GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
            GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
            GX_STATUS_INVALID_ACCESS        Device access mode error.
            GX_STATUS_INVALID_CALL          Acquisition is not started or the callback is registered. It
                                            is not allowed to call the interface.
            GX_STATUS_ERROR                 Unspecified internal errors that are not expected to occur.
            The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXStreamOff (GX_DEV_HANDLE hDevice);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      在开始采集之后，通过此接口可以获取所有采集到图像的 buf（零拷贝）。获取到的图像数据数组中的
存图顺序是从旧到新，即 ppFrameBufferArray[0]存储的是最旧的图，ppFrameBufferArray[nFrameCount -
1]存储的是最新的图。
\attention  无
\param		[in]hDevice    设备句柄
\param      [out]ppFrameBufferArray         图像数据指针的数组
\param      [in]nFrameBufferArraySize       图像数组申请个数
\param      [out]pnFrameCount               返回实际填充图像个数
\param      [in]nTimeOut                    取图的超时时间（单位 ms）
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
			GX_STATUS_INVALID_ACCESS      当前不可访问
			其它错误情况请参见GX_STATUS_LIST 

\English:
\brief      After starting the acquisition, all the buffers (zero copies) of the acquired images can be
            obtained through this interface. The order of the stored images in the image data array is
            from old to new, that is, ppFrameBufferArray[0] stores the oldest image, and
            ppFrameBufferArray[nFrameCount - 1] stores the latest image.
\param      [in]hDevice                     The handle of the device.
\param      [out]ppFrameBufferArray         Array of image data pointers.
\param      [in]nFrameBufferArraySize       The number of applications for image arrays.
\param      [out]pnFrameCount               Returns the number of actual filled images.
\param      [in]nTimeOut                    Take timeout time (unit: ms).
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
            GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
            GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
            GX_STATUS_INVALID_PARAMETER     The pointer that the user introduces is NULL.
            GX_STATUS_INVALID_CALL          Acquisition is not started or the callback is registered. It
                                            is not allowed to call the interface.
            GX_STATUS_NEED_MORE_BUFFER      Insufficient buffer requested by the user: When reading,the user
                                            input buffer size is smaller than the actual need.
            GX_STATUS_TIMEOUT               Acquire image timeout error.
            GX_STATUS_ERROR                 Unspecified internal errors that are not expected to occur.
            The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXDQAllBufs (GX_DEV_HANDLE hDevice, 
                    PGX_FRAME_BUFFER *ppFrameBufferArray, 
                    uint32_t nFrameBufferArraySize, 
                    uint32_t *pnFrameCount,
                    uint32_t nTimeOut);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      在开始采集之后，通过此接口可以将图像数据 Buf 放回 GxIAPI 库，继续用于采集。
\attention  无
\param		[in]hDevice    设备句柄
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
			GX_STATUS_INVALID_ACCESS      当前不可访问
			其它错误情况请参见GX_STATUS_LIST 

\English:
\brief      After the acquisition is started, all the acquired image data buffers can be put back into
            the GxIAPI library through this interface, and continue to be used for acquisition.
\param      [in]hDevice             The handle of the device.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
            GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
            GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
            GX_STATUS_INVALID_CALL          Invalid interface call.
            The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXQAllBufs (GX_DEV_HANDLE hDevice);
#endif

#endif  //GX_GALAXY_H
