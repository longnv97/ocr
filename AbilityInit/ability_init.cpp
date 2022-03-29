#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h> 
#include <time.h>
#include <fstream>
#include <sys/types.h>
#include <signal.h>
#include <iomanip>
#include <vpu_tools_common.hpp>
#include <vpu/vpu_plugin_config.hpp>
#include <inference_engine.hpp>
#include "ab_camera_cv_sample.h"
#include "json.hpp"

#include "ability_init.hpp"
#include "../config.hpp"

extern "C" {
    #include <ab_host_api.h>
    #include <ab_cv_api.h>
}

using namespace InferenceEngine;
using json = nlohmann::json;

Camera_Host_Result_Type CameraHost_StreamIn_Init(void)
{
    printf("========= CameraHost_StreamIn_Init =========\n");
    Camera_Host_Result_Type result = CAMERA_HOST_OK;
    CameraHostAPI_General_Res   Response;
    Host_Request_Message        RequestMessage;

    CameraHostAPI_StreamIn_Init_Req InitReq     = {0};
    InitReq.StreamNum = 1;
	InitReq.MainStreamConfig.FPS = 15;
	InitReq.MainStreamConfig.FrameType = Camera_Host_Stream_Frame_Format_BGR888Planar; //Camera_Host_Stream_Frame_Format_RGB888Planar
	InitReq.MainStreamConfig.ResolutionWidth = MAIN_STREAM_FRAME_WIDTH;
	InitReq.MainStreamConfig.ResolutionHeight = MAIN_STREAM_FRAME_HEIGHT;
	InitReq.SubStreamConfig.FrameType = Camera_Host_Stream_Frame_Format_RGB888Packed;
	InitReq.SubStreamConfig.ResolutionWidth = SUB_STREAM_FRAME_WIDTH;
	InitReq.SubStreamConfig.ResolutionHeight = SUB_STREAM_FRAME_HEIGHT;
    RequestMessage.MessageID = CAMERA_HOST_API_STREAM_IN_INIT;
    RequestMessage.DataLength= sizeof(CameraHostAPI_StreamIn_Init_Req);
    RequestMessage.DataPtr   = (void*)&InitReq;

    result = ab_host_msg_send(&RequestMessage,(void*)&Response);
    return result;
}

Camera_Host_Result_Type CameraHost_StreamIn_Deinit(void)
{
    printf("========= CameraHost_StreamIn_Deinit =========\n");
    Camera_Host_Result_Type result = CAMERA_HOST_OK;
    CameraHostAPI_General_Res   Response;
    Host_Request_Message        RequestMessage;

    RequestMessage.MessageID = CAMERA_HOST_API_STREAM_IN_DEINIT;
    RequestMessage.DataLength= 0;
    RequestMessage.DataPtr   = NULL;

    result = ab_host_msg_send(&RequestMessage,(void*)&Response);
    return result;
}

Camera_Host_Result_Type CameraHost_StreamOut_Start(void)
{
    printf("========= CameraHost_StreamOut_Init =========\n");
    Camera_Host_Result_Type result = CAMERA_HOST_OK;
    CameraHostAPI_General_Res   Response;
    Host_Request_Message        RequestMessage;

    CameraHostInterface_StreamOut_Start_Req InitReq;
    InitReq.SyncMode = Camera_Host_Stream_Out_SyncMode_Sync;
    RequestMessage.MessageID = CAMERA_HOST_API_STREAM_OUT_START;
    RequestMessage.DataLength= sizeof(CameraHostInterface_StreamOut_Start_Req);
    RequestMessage.DataPtr   = (void*)&InitReq;
    
    result = ab_host_msg_send(&RequestMessage,(void*)&Response);
    return result;
}

Camera_Host_Result_Type CameraHost_StreamOut_Stop(void)
{
    printf("========= CameraHost_StreamOut_Stop =========\n");
    Camera_Host_Result_Type result = CAMERA_HOST_OK;
    CameraHostAPI_General_Res   Response;
    Host_Request_Message        RequestMessage;

    RequestMessage.MessageID = CAMERA_HOST_API_STREAM_OUT_STOP;
    RequestMessage.DataLength= 0;
    RequestMessage.DataPtr   = NULL;
    
    result = ab_host_msg_send(&RequestMessage,(void*)&Response);
    return result;
}

Camera_Host_Result_Type CameraHost_StreamOut_FrameUpdate(void* ReqPara)
{
    //printf("========= CameraHost_StreamOut_FrameUpdate =========\n");
    Camera_Host_Result_Type result = CAMERA_HOST_OK;
    CameraHostAPI_General_Res   Response;
    Host_Request_Message        RequestMessage;

    RequestMessage.MessageID = CAMERA_HOST_API_STREAM_OUT_FRAME_UPDATE;
    RequestMessage.DataLength= sizeof(CameraHostAPI_StreamOut_FrameUpdate_Req);
    RequestMessage.DataPtr   = ReqPara;
    
    result = ab_host_msg_send(&RequestMessage,(void*)&Response);
    return result;
}

Camera_Host_Result_Type CameraHost_RTP_MetaDataUpdate(void* ReqParaPtr)
{
    //printf("========= CameraHost_RTP_MetaDataUpdate =========\n");
    Camera_Host_Result_Type result = CAMERA_HOST_OK;
    CameraHostAPI_General_Res   Response;
    Host_Request_Message        RequestMessage;

    RequestMessage.MessageID = CAMERA_HOST_API_STREAM_OUT_RTP_DATA_UPDATE;
    RequestMessage.DataLength= sizeof(CameraHostAPI_StreamOut_Data_Req);
    RequestMessage.DataPtr   = ReqParaPtr;
    
    result = ab_host_msg_send(&RequestMessage,(void*)&Response);
    return result;
}

Camera_Host_Result_Type CameraHost_StreamIn_FramePop(void* ResPara)
{
    //printf("========= CameraHost_StreamIn_FramePop =========\n");
    Camera_Host_Result_Type result = CAMERA_HOST_OK;
    Host_Request_Message    RequestMessage;
    CameraHostAPI_StreamIn_GetVideoFrame_Req ReqGetFrame = {0};

    RequestMessage.MessageID = CAMERA_HOST_API_STREAM_IN_GET_VIDEO_FRAME;
    RequestMessage.DataLength= sizeof(CameraHostAPI_StreamIn_GetVideoFrame_Req);
    RequestMessage.DataPtr   = (void*)&ReqGetFrame;
    
    result = ab_host_msg_send(&RequestMessage,ResPara);
    return result;
}

void CameraHostAPI_EventCallback(Host_Remote_ReqMsg_Struct* ReqMessagePtr, Host_Remote_ResMsg_Struct* ResMessagePtr)
{
  ResMessagePtr->ResResult  = CAMERA_HOST_OK;
  ResMessagePtr->ResParaLen = 0;
  if(ReqMessagePtr != NULL) {
    switch (ReqMessagePtr->MessageID)
    {
      default:
        printf("The message is not handled\n");
        break;
    }
  }
  return;
}

std::string fileNameNoExt(const std::string &filepath) {
    auto pos = filepath.rfind('.');
    if (pos == std::string::npos) return filepath;
    return filepath.substr(0, pos);
}

std::map<std::string, std::string> configure(const std::string& confFileName) {
    auto config = parseConfig(confFileName);

    return config;
}

void json_export(pCameraHostAPI_StreamOut_FrameUpdate_Req pFDFRResult, std::string filePath)
{
    json j;
    CameraHostAPI_Display_SimpleFDFR_Struc s_fdfr = {0};
    for (size_t index = 0; index < pFDFRResult->ItemNumber; index++)
    {
        json j_merge; 
        json j_item;
        int shift = index * (sizeof(Camera_Host_Stream_Out_Display_Type) + sizeof(CameraHostAPI_Display_SimpleFDFR_Struc)) + sizeof(Camera_Host_Stream_Out_Display_Type);
        memcpy(&s_fdfr,(void*)&pFDFRResult->Items[shift],sizeof(CameraHostAPI_Display_SimpleFDFR_Struc));
        //printf("x1=%d y1=%d x2=%d y2=%d\n",s_fdfr.Left,s_fdfr.Top,s_fdfr.Right,s_fdfr.Bottom);
        j_item["label"]       = "Unknown";
        j_item["x1"]          = s_fdfr.Left;
        j_item["y1"]          = s_fdfr.Top;
        j_item["x2"]          = s_fdfr.Right;
        j_item["y2"]          = s_fdfr.Bottom;
        j_merge["item"].push_back(j_item);
        j_item.clear();
        j.push_back(j_merge);
        j_merge.clear();
    }
    json j_item;
    j_item["AI app"] = "default FD";
    j_item["resolution"] = "672x384";
    j_item["pts"] = 0;
    if (j.size() > 0)
        j.insert(j.begin(), j_item);
    else
        j.push_back(j_item);
    std::ofstream o(filePath);
    o << std::setw(4) << j << std::endl;
    o.close();
    return;
}