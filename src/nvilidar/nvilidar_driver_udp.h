#pragma once

#include "nvilidar_def.h"
#include "nvilidar_protocol.h"
#include "socket/nvilidar_socket.h"
#include "nvilidar_filter.h"
#include <string>
#include <vector>
#include <stdint.h>
#include <math.h>

#if defined(_WIN32)
#include <conio.h>
#include <process.h>
#include <tlhelp32.h>
#include <sys/utime.h>
#include <io.h>
#include <direct.h>
#else
#include <assert.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#endif


//---定义库信息 VS系列的生成库文件  
#ifdef WIN32
	#define NVILIDAR_DRIVER_UDP_API __declspec(dllexport)
#else
	#define NVILIDAR_DRIVER_UDP_API
#endif // ifdef WIN32

namespace nvilidar
{
    //lidar driver 
	class  NVILIDAR_DRIVER_UDP_API LidarDriverUDP
    {
		public:
			LidarDriverUDP();                //构造函数
			~LidarDriverUDP();           //析构函数

			void LidarLoadConfig(Nvilidar_UserConfigTypeDef cfg);
			bool LidarIsConnected();			//is lidar connected 
			bool LidarGetScanState();			//is lidar transing data 
			bool LidarInitialialize();			//lidar init 
			bool LidarCloseHandle();			//lidar quit and disconnect 
			bool LidarTurnOn();					//start scan    
			bool LidarTurnOff();				//stop scan 


			std::string getSDKVersion();										//get current sdk version 
			bool StartScan(void);                           //start scan 
			bool StopScan(void);                            //stop scan 
			bool Reset(void);								//lidar reset 

			bool SetIntensities(const uint8_t has_intensity, uint32_t timeout = NVILIDAR_DEFAULT_TIMEOUT);     //set lidar has/has not sensitive 
			bool GetDeviceInfo(Nvilidar_DeviceInfo & info, uint32_t timeout = NVILIDAR_DEFAULT_TIMEOUT);
			bool SetScanMotorSpeed(uint16_t frequency_add, uint16_t &ret_frequency,    						//set lidar aim speed 
													uint32_t timeout = NVILIDAR_DEFAULT_TIMEOUT);
			bool SetSamplingRate(uint32_t rate_add, uint32_t &rate,
											uint32_t timeout = NVILIDAR_DEFAULT_TIMEOUT);   	//set lidar sampling 
			bool SetTrailingLevel(uint8_t tral_set, uint8_t &tral,
												uint32_t  timeout = NVILIDAR_DEFAULT_TIMEOUT);
			bool SetApdValue(uint16_t apd_set, uint16_t &apd,
											uint32_t  timeout = NVILIDAR_DEFAULT_TIMEOUT);		//set lidar apd value 

			bool GetLidarCfg(Nvilidar_StoreConfigTypeDef &info,									//get config para 
												uint32_t  timeout = NVILIDAR_DEFAULT_TIMEOUT);

			bool GetZeroOffsetAngle(int16_t &angle,												//lidar lidar 0 offset 
												uint32_t timeout = NVILIDAR_DEFAULT_TIMEOUT);
			bool SetZeroOffsetAngle(int16_t angle_set, int16_t &angle,
												uint32_t timeout = NVILIDAR_DEFAULT_TIMEOUT); 	//get lidar 0 offset 

			bool GetFilterQualityThreshold(uint16_t &ret_filter_quality,uint32_t timeout = NVILIDAR_DEFAULT_TIMEOUT);	 //get lidar filter quality 

			bool SetFilterQualityThreshold(uint16_t filter_quality_set, uint16_t &ret_filter_quality,
												uint32_t timeout = NVILIDAR_DEFAULT_TIMEOUT);	//set lidar filter quality value 

			bool SaveCfg(bool &flag,uint32_t timeout = NVILIDAR_DEFAULT_TIMEOUT);				//save para

			bool LidarSamplingProcess(LidarScan &scan, uint32_t timeout = NVILIDAR_DEFAULT_TIMEOUT);  //lidar data output 

			Nvilidar_PackageStateTypeDef   lidar_state;											//lidar state 

		private:
			bool LidarConnect(std::string ip_addr, uint16_t port);  //serialport init 
			void LidarDisconnect();      //close udp  
			bool SendUDP(const uint8_t *data, size_t size);      //send data to udp  
			bool SendCommand(uint8_t cmd, uint8_t *payload = NULL,uint16_t payloadsize = 0);
			void NormalDataUnpack(uint8_t *buf, uint16_t len);		//unpack（normal data）
			void NormalDataAnalysis(Nvilidar_Protocol_NormalResponseData data);	
			bool PointDataUnpack(uint8_t *byte, uint16_t len);		//unpack（point cloud）
			void PointDataAnalysis(Nvilidar_PointViewerPackageInfoTypeDef data);	
			LidarModelListEnumTypeDef GetLidarModelName(Nvilidar_DeviceInfo info);			//get lidar model name  
			
			//thread  
			bool createThread();		//create thread 
			void closeThread();			//close thread 
			bool waitNormalResponse(uint32_t timeout = NVILIDAR_POINT_TIMEOUT);	//wait for lidar response nomal data 
			void setNormalResponseUnlock();	//unlock nomal data  
			void setCircleResponseUnlock();	//unlock point data 
			void LidarSamplingData(CircleDataInfoTypeDef info, LidarScan &outscan);		//interface for lidar point data 

			//----------------------network---------------------------

			nvilidar_socket::Nvilidar_Socket_UDP socket_udp;

			//----------------------value ----------------------------
			Nvilidar_UserConfigTypeDef     lidar_cfg;				//lidar config data 
			CircleDataInfoTypeDef		   circleDataInfo;			//lida circle data  
			NvilidarRecvInfoTypeDef		   recv_info;				//lidar receive data 

			uint32_t    m_0cIndex = 0;                  //0 index
			int32_t     m_last0cIndex = 0;              //0 index
			uint32_t    m_differ0cIndex = 0;            //0 index
			bool        m_first_circle_finish = false;  //first circle finish,case calc fault
			uint64_t	m_run_circles = 0;				//has send data   

			//---------------------thread---------------------------
			#if defined(_WIN32)
				HANDLE  _thread = NULL;
				HANDLE  _event_analysis;		
				HANDLE  _event_circle;			

				DWORD static WINAPI periodThread(LPVOID lpParameter);		//thread  
			#else 
				pthread_t _thread = -1;
				pthread_cond_t _cond_analysis;
				pthread_mutex_t _mutex_analysis;
				pthread_cond_t _cond_point;
				pthread_mutex_t _mutex_point;
				static void *periodThread(void *lpParameter) ;
			#endif
    };
}
