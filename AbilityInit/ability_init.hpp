#ifndef _ABILITY_INIT_H_
#define _ABILITY_INIT_H_

#include <map>
extern "C" {
    #include <ab_host_api.h>
    #include <ab_cv_api.h>
    #include <ab_host_types.h>
}

Camera_Host_Result_Type CameraHost_StreamIn_Init(void);
Camera_Host_Result_Type CameraHost_StreamIn_Deinit(void);
Camera_Host_Result_Type CameraHost_StreamOut_Start(void);
Camera_Host_Result_Type CameraHost_StreamOut_Stop(void);
Camera_Host_Result_Type CameraHost_StreamOut_FrameUpdate(void* ReqPara);
Camera_Host_Result_Type CameraHost_RTP_MetaDataUpdate(void* ReqParaPtr);
Camera_Host_Result_Type CameraHost_StreamIn_FramePop(void* ResPara);
std::map<std::string, std::string> configure(const std::string& confFileName);
std::string fileNameNoExt(const std::string &filepath);
void CameraHostAPI_EventCallback(Host_Remote_ReqMsg_Struct* ReqMessagePtr, Host_Remote_ResMsg_Struct* ResMessagePtr);
void json_export(pCameraHostAPI_StreamOut_FrameUpdate_Req pFDFRResult, std::string filePath);

#endif  /* _ABILITY_INIT_H_ */