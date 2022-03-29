#include <stdio.h>
#include <stdlib.h>

extern "C" {
    #include <ab_host_api.h>


static Camera_Host_Result_Type CameraHost_RTP_MetaDataReset(void)
{
    printf("========= CameraHost_RTP_MetaDataReset =========\n");
    Camera_Host_Result_Type result = CAMERA_HOST_OK;
    CameraHostAPI_General_Res   Response;
    Host_Request_Message        RequestMessage;

    RequestMessage.MessageID = CAMERA_HOST_API_STREAM_OUT_RTP_FIFO_RESET;
    RequestMessage.DataLength= 0;
    RequestMessage.DataPtr   = NULL;
    
    result = ab_host_msg_send(&RequestMessage,(void*)&Response);
    return result;
}

static Camera_Host_Result_Type CameraHost_RTP_MetaDataUpdate(void* ReqParaPtr)
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

}

static Camera_Host_Result_Type CameraHost_Config_Get()
{
    Camera_Host_Result_Type result = CAMERA_HOST_OK;

    CameraHostAPI_ImageParameter_Para CameraSensorPara;
    CameraHostAPI_Stream_Param CameraStreamPara;
    CameraHostAPI_OverlayText_Param CameraOverlayTextPara;
    CameraHostAPI_OverlayTime_Param CameraOverlayTimePara;
    CameraHostAPI_OverlayImage_Param CameraOverlayImagePara;
    CameraHostAPI_Audio_Param CameraAudioPara;
    CameraHostAPI_ImageConfig_Para CameraAppearancePara;
    CameraHostAPI_GeneralInfo_Para DeviceInfoPara;
    CameraHostAPI_DateTimeConfig_Para DateTimeConfigPara;
    CameraHostAPI_Network_Discovery_Para DiscoveryPara;
    CameraHostAPI_SSHService_Para SSHPara;
    CameraHostAPI_Web_Account_Para WebAccountPara;
    CameraHostAPI_Image_Advanced_Para ImageAdvancedPara;

    Host_Request_Message    RequestMessage;

    printf("CAMERA_HOST_API_GET_IMAGE_PARAMETER_ALL\n");
    RequestMessage.MessageID = CAMERA_HOST_API_GET_IMAGE_PARAMETER_ALL;
    result = ab_host_msg_send(&RequestMessage,(void*)&CameraSensorPara);

    printf("CAMERA_HOST_API_GET_SETTING_MAIN_STREAM_ALL\n");
    RequestMessage.MessageID = CAMERA_HOST_API_GET_SETTING_MAIN_STREAM_ALL;
    result = ab_host_msg_send(&RequestMessage,(void*)&CameraStreamPara);

    printf("CAMERA_HOST_API_GET_SETTING_SECOND_STREAM_ALL\n");
    RequestMessage.MessageID = CAMERA_HOST_API_GET_SETTING_SECOND_STREAM_ALL;
    result = ab_host_msg_send(&RequestMessage,(void*)&CameraStreamPara);

    printf("CAMERA_HOST_API_GET_SETTING_OVERLAY_TEXT_ALL\n");
    RequestMessage.MessageID = CAMERA_HOST_API_GET_SETTING_OVERLAY_TEXT_ALL;
    result = ab_host_msg_send(&RequestMessage,(void*)&CameraOverlayTextPara);

    printf("CAMERA_HOST_API_GET_SETTING_OVERLAY_TIME_ALL\n");
    RequestMessage.MessageID = CAMERA_HOST_API_GET_SETTING_OVERLAY_TIME_ALL;
    result = ab_host_msg_send(&RequestMessage,(void*)&CameraOverlayTimePara);

    printf("CAMERA_HOST_API_GET_SETTING_OVERLAY_IMAGE_ALL\n");
    RequestMessage.MessageID = CAMERA_HOST_API_GET_SETTING_OVERLAY_IMAGE_ALL;
    result = ab_host_msg_send(&RequestMessage,(void*)&CameraOverlayImagePara);

    printf("CAMERA_HOST_API_GET_SETTING_AUDIO_ALL\n");
    RequestMessage.MessageID = CAMERA_HOST_API_GET_SETTING_AUDIO_ALL;
    result = ab_host_msg_send(&RequestMessage,(void*)&CameraAudioPara);

    printf("CAMERA_HOST_API_GET_IMAGE_CONFIG_ALL\n");
    RequestMessage.MessageID = CAMERA_HOST_API_GET_IMAGE_CONFIG_ALL;
    result = ab_host_msg_send(&RequestMessage,(void*)&CameraAppearancePara);

    printf("CAMERA_HOST_API_GET_CAMERA_GENERAL_INFO\n");
    RequestMessage.MessageID = CAMERA_HOST_API_GET_CAMERA_GENERAL_INFO;
    result = ab_host_msg_send(&RequestMessage,(void*)&DeviceInfoPara);

    printf("CAMERA_HOST_API_GET_DATETIME_CONFIG_ALL\n");
    RequestMessage.MessageID = CAMERA_HOST_API_GET_DATETIME_CONFIG_ALL;
    result = ab_host_msg_send(&RequestMessage,(void*)&DateTimeConfigPara);

    printf("CAMERA_HOST_API_GET_NETWORK_DISCOVERY_CONFIG_ALL\n");
    RequestMessage.MessageID = CAMERA_HOST_API_GET_NETWORK_DISCOVERY_CONFIG_ALL;
    result = ab_host_msg_send(&RequestMessage,(void*)&DiscoveryPara);

    printf("CAMERA_HOST_API_GET_NETWORK_SSH_CONFIG_ALL\n");
    RequestMessage.MessageID = CAMERA_HOST_API_GET_NETWORK_SSH_CONFIG_ALL;
    result = ab_host_msg_send(&RequestMessage,(void*)&SSHPara);

    printf("CAMERA_HOST_API_GET_WEB_ACCOUNT_ALL\n");
    RequestMessage.MessageID = CAMERA_HOST_API_GET_WEB_ACCOUNT;
    result = ab_host_msg_send(&RequestMessage,(void*)&WebAccountPara);

    printf("UserName:%s\n",WebAccountPara.User.UserName);

    printf("CAMERA_HOST_API_GET_IMAGE_ADVANCED_ALL\n");
    RequestMessage.MessageID = CAMERA_HOST_API_GET_IMAGE_ADVANCED_ALL;
    result = ab_host_msg_send(&RequestMessage,(void*)&ImageAdvancedPara);

    return result;
}

static Camera_Host_Result_Type CameraHost_Config_Sensor_Set()
{
    Camera_Host_Result_Type result = CAMERA_HOST_OK;
    Host_Request_Message    RequestMessage;
    CameraHostAPI_General_Res   Response;
    CameraHostAPI_ImageParameter_Para CameraPara;
    CameraPara.Brightness     = 60;
    CameraPara.Contrast       = 60;
    CameraPara.ColorDesaturation = 60;
    CameraPara.Sharpness = 60;
    CameraPara.WBType         = Camera_Host_WB_Manual;
    CameraPara.WBColorLevel   = 4000;
    RequestMessage.MessageID = CAMERA_HOST_API_SET_IMAGE_PARAMETER_ALL;
    RequestMessage.DataLength= sizeof(CameraHostAPI_ImageParameter_Para);
    RequestMessage.DataPtr   = (void*)&CameraPara;
    result = ab_host_msg_send(&RequestMessage,(void*)&Response);

    return result;
}

static Camera_Host_Result_Type CameraHost_Config_MainStream_Set()
{
    Camera_Host_Result_Type result = CAMERA_HOST_OK;
    Host_Request_Message    RequestMessage;
    CameraHostAPI_General_Res   Response;
    CameraHostAPI_Stream_Param CameraStreamPara;
    CameraStreamPara.Format     = Camera_Host_StreamFormat_H264;
    CameraStreamPara.Resolution = Camera_Host_StreamResolution_1920x1080;
    CameraStreamPara.Fps        = 15;
    CameraStreamPara.GOP        = 30;
    CameraStreamPara.TargetBitrate = 364;
    CameraStreamPara.Mode       = Camera_Host_StreamMode_VBR;
    CameraStreamPara.Quality    = Camera_Host_StreamQuality_High;
    RequestMessage.MessageID    = CAMERA_HOST_API_SET_SETTING_MAIN_STREAM_ALL;
    RequestMessage.DataLength   = sizeof(CameraHostAPI_Stream_Param);
    RequestMessage.DataPtr      = (void*)&CameraStreamPara;
    result = ab_host_msg_send(&RequestMessage,(void*)&Response);

    return result;
}

static Camera_Host_Result_Type CameraHost_Config_SecondStream_Set()
{
    Camera_Host_Result_Type result = CAMERA_HOST_OK;
    Host_Request_Message    RequestMessage;
    CameraHostAPI_General_Res   Response;
    CameraHostAPI_Stream_Param CameraStreamPara;
    CameraStreamPara.Enable     = Camera_Host_Switch_On;
    CameraStreamPara.Format     = Camera_Host_StreamFormat_H264;
    CameraStreamPara.Resolution = Camera_Host_StreamResolution_1920x1080;
    CameraStreamPara.Fps        = 15;
    CameraStreamPara.GOP        = 30;
    CameraStreamPara.TargetBitrate = 364;
    CameraStreamPara.Mode       = Camera_Host_StreamMode_VBR;
    CameraStreamPara.Quality    = Camera_Host_StreamQuality_High;
    RequestMessage.MessageID    = CAMERA_HOST_API_SET_SETTING_SECOND_STREAM_ALL;
    RequestMessage.DataLength   = sizeof(CameraHostAPI_OverlayText_Param);
    RequestMessage.DataPtr      = (void*)&CameraStreamPara;
    result = ab_host_msg_send(&RequestMessage,(void*)&Response);

    return result;
}

static Camera_Host_Result_Type CameraHost_Config_OverlayText_Set()
{
    Camera_Host_Result_Type result = CAMERA_HOST_OK;
    Host_Request_Message    RequestMessage;
    CameraHostAPI_General_Res       Response;
    CameraHostAPI_OverlayText_Param CameraOverlayTextPara;
    CameraOverlayTextPara.Enable         = Camera_Host_Switch_On;
    CameraOverlayTextPara.FontSize       = Camera_Host_Overlay_FontSize_Medium;
    CameraOverlayTextPara.Color.Color_R  = 0;
    CameraOverlayTextPara.Color.Color_G  = 0;
    CameraOverlayTextPara.Color.Color_B  = 255;
    CameraOverlayTextPara.Position       = Camera_Host_Overlay_Position_BottomLeft;
    snprintf(CameraOverlayTextPara.Text,CAMERA_NAME_LENGTH,"AI Cam");
    RequestMessage.MessageID        = CAMERA_HOST_API_SET_SETTING_OVERLAY_TEXT_ALL;
    RequestMessage.DataLength       = sizeof(CameraHostAPI_OverlayText_Param);
    RequestMessage.DataPtr          = (void*)&CameraOverlayTextPara;
    result = ab_host_msg_send(&RequestMessage,(void*)&Response);

    return result;
}

static Camera_Host_Result_Type CameraHost_Config_OverlayTime_Set()
{
    Camera_Host_Result_Type result = CAMERA_HOST_OK;
    Host_Request_Message    RequestMessage;
    CameraHostAPI_General_Res       Response;
    CameraHostAPI_OverlayTime_Param CameraOverlayTimePara;
    CameraOverlayTimePara.Enable         = Camera_Host_Switch_On;
    CameraOverlayTimePara.FontSize       = Camera_Host_Overlay_FontSize_Medium;
    CameraOverlayTimePara.Color.Color_R  = 0;
    CameraOverlayTimePara.Color.Color_G  = 255;
    CameraOverlayTimePara.Color.Color_B  = 255;
    CameraOverlayTimePara.Position       = Camera_Host_Overlay_Position_TopRight;
    RequestMessage.MessageID        = CAMERA_HOST_API_SET_SETTING_OVERLAY_TIME_ALL;
    RequestMessage.DataLength       = sizeof(CameraHostAPI_OverlayTime_Param);
    RequestMessage.DataPtr          = (void*)&CameraOverlayTimePara;
    result = ab_host_msg_send(&RequestMessage,(void*)&Response);

    return result;
}

static Camera_Host_Result_Type CameraHost_Config_OverlayImage_Set()
{
    Camera_Host_Result_Type result = CAMERA_HOST_OK;
    Host_Request_Message    RequestMessage;
    CameraHostAPI_General_Res       Response;
    CameraHostAPI_OverlayImage_Param CameraOverlayImagePara;
    CameraOverlayImagePara.Enable         = Camera_Host_Switch_On;
    CameraOverlayImagePara.Position       = Camera_Host_Overlay_Position_BottomRight;
    RequestMessage.MessageID        = CAMERA_HOST_API_SET_SETTING_OVERLAY_IMAGE_ALL;
    RequestMessage.DataLength       = sizeof(CameraHostAPI_OverlayImage_Param);
    RequestMessage.DataPtr          = (void*)&CameraOverlayImagePara;
    result = ab_host_msg_send(&RequestMessage,(void*)&Response);

    return result;
}

static Camera_Host_Result_Type CameraHost_Config_Audio_Set()
{
    Camera_Host_Result_Type result = CAMERA_HOST_OK;
    Host_Request_Message    RequestMessage;
    CameraHostAPI_General_Res       Response;
    CameraHostAPI_Audio_Param CameraAudioPara;
    CameraAudioPara.SpeakerEnable   = Camera_Host_Switch_Off;
    CameraAudioPara.SpeakerGain     = 100;
    CameraAudioPara.MicEnable       = Camera_Host_Switch_Off;
    CameraAudioPara.MicGain         = 100;
    RequestMessage.MessageID        = CAMERA_HOST_API_SET_SETTING_AUDIO_ALL;
    RequestMessage.DataLength       = sizeof(CameraHostAPI_Audio_Param);
    RequestMessage.DataPtr          = (void*)&CameraAudioPara;
    result = ab_host_msg_send(&RequestMessage,(void*)&Response);

    return result;
}

static Camera_Host_Result_Type CameraHost_Config_WhiteBalance_Set()
{
    Camera_Host_Result_Type result = CAMERA_HOST_OK;
    Host_Request_Message    RequestMessage;
    CameraHostAPI_General_Res   Response;
    CameraHostAPI_ImageParameter_Para CameraSensorPara;
    CameraSensorPara.WBType         = Camera_Host_WB_Manual;
    CameraSensorPara.WBColorLevel   = 7000;
    RequestMessage.MessageID = CAMERA_HOST_API_SET_IMAGE_PARAMETER_WHITEBALANCE_TYPE;
    RequestMessage.DataLength= sizeof(CameraHostAPI_ImageParameter_Para);
    RequestMessage.DataPtr   = (void*)&CameraSensorPara;
    result = ab_host_msg_send(&RequestMessage,(void*)&Response);

    RequestMessage.MessageID = CAMERA_HOST_API_SET_IMAGE_PARAMETER_WHITEBALANCE_COLOR_LEVEL;
    RequestMessage.DataPtr   = (void*)&CameraSensorPara;
    result = ab_host_msg_send(&RequestMessage,(void*)&Response);
    return result;
}

static Camera_Host_Result_Type CameraHost_Config_Recording_Set()
{
  Camera_Host_Result_Type result = CAMERA_HOST_OK;
  Host_Request_Message    RequestMessage;
  CameraHostAPI_General_Res   Response;
  CameraHostAPI_Recording_Param CameraPara;
  memset(&CameraPara, 0, sizeof(CameraPara));
  CameraPara.Enable         = Camera_Host_Switch_Off;
  CameraPara.RecordingType  = Camera_Host_Recording_OverWrite;
  CameraPara.RecordingFileSizeType  = Camera_Host_Recording_FileSize_Type_MB;
  CameraPara.FilesizeValue  = 20;
  CameraPara.RecordingLimitMB = 6144;
  RequestMessage.MessageID = CAMERA_HOST_API_SET_RECORDING_CONFIG;
  RequestMessage.DataLength= sizeof(CameraHostAPI_Recording_Param);
  RequestMessage.DataPtr   = (void*)&CameraPara;
  result = ab_host_msg_send(&RequestMessage,(void*)&Response);
  return result;
}

static Camera_Host_Result_Type CameraHost_Command_OverlaySetting_Trigger()
{
    Camera_Host_Result_Type result = CAMERA_HOST_OK;
    Host_Request_Message    RequestMessage;
    CameraHostAPI_General_Res       Response;
    RequestMessage.MessageID        = CAMERA_HOST_API_COMMAND_OVERLAY_SETTING_TRIGGER;
    result = ab_host_msg_send(&RequestMessage,(void*)&Response);

    return result;
}

static Camera_Host_Result_Type CameraHost_Command_PTZ_Zoom()
{
    Camera_Host_Result_Type result = CAMERA_HOST_OK;
    Host_Request_Message    RequestMessage;
    CameraHostAPI_General_Res       Response;
    CameraHostAPI_Zoom_Param CameraZoomPara;
    CameraZoomPara.ZoomLevel        = Camera_Host_Zoom_Level_4;
    RequestMessage.MessageID        = CAMERA_HOST_API_SET_ZOOM_LEVEL;
    RequestMessage.DataLength       = sizeof(CameraHostAPI_Zoom_Param);
    RequestMessage.DataPtr          = (void*)&CameraZoomPara;
    result = ab_host_msg_send(&RequestMessage,(void*)&Response);

    return result;
}

static Camera_Host_Result_Type CameraHost_Command_PTZ_AutoFocus()
{
    Camera_Host_Result_Type result = CAMERA_HOST_OK;
    Host_Request_Message    RequestMessage;
    CameraHostAPI_General_Res       Response;
    RequestMessage.MessageID        = CAMERA_HOST_API_SET_AUTO_FOCUS;
    result = ab_host_msg_send(&RequestMessage,(void*)&Response);

    return result;
}

static Camera_Host_Result_Type CameraHost_Command_PTZ_ManualFocus()
{
  Camera_Host_Result_Type result = CAMERA_HOST_OK;
  Host_Request_Message    RequestMessage;
  CameraHostAPI_ManualFocus_Param CameraFocusPara;
  CameraFocusPara.FocusLevel      = 5;
  CameraHostAPI_General_Res       Response;
  RequestMessage.MessageID        = CAMERA_HOST_API_SET_MANUAL_FOUCS;
  RequestMessage.DataLength       = sizeof(CameraHostAPI_ManualFocus_Param);
  RequestMessage.DataPtr          = (void*)&CameraFocusPara;
  result = ab_host_msg_send(&RequestMessage,(void*)&Response);

  return result;
}

static Camera_Host_Result_Type CameraHost_Config_AdvancedICR_Set()
{
  Camera_Host_Result_Type result = CAMERA_HOST_OK;
  Host_Request_Message    RequestMessage;
  CameraHostAPI_General_Res       Response;
  CameraHostAPI_Image_Advanced_Para  CameraPara;
  memset(&CameraPara, 0, sizeof(CameraPara));
  CameraPara.ICRType                    = Camera_Host_ICR_Type_General;
  CameraPara.ICRPara.Mode               = Camera_Host_ICR_Filter_Mode_Auto;
  CameraPara.ICRPara.BaseOnType         = Camera_Host_ICR_BaseOn_Type_LightSensor;
  CameraPara.ICRPara.ICRSync            = Camera_Host_Switch_Off;
  CameraPara.ICRPara.LightLevel         = Camera_Host_Advanced_ICR_LightLevel3;
  CameraPara.ICRPara.ICRIntensityEnable = Camera_Host_Switch_On;
  CameraPara.ICRPara.ICRIntensityValue  = 50;
  CameraPara.ICRPara.DetectionInterval  = 20;
  RequestMessage.MessageID        = CAMERA_HOST_API_SET_IMAGE_ADVANCED_IRCUT_FILTER;
  RequestMessage.DataLength       = sizeof(CameraPara);
  RequestMessage.DataPtr          = (void*)&CameraPara;
  result = ab_host_msg_send(&RequestMessage,(void*)&Response);

  return result;
}

static Camera_Host_Result_Type CameraHost_Config_AdvancedExp_Set()
{
  Camera_Host_Result_Type result = CAMERA_HOST_OK;
  Host_Request_Message    RequestMessage;
  CameraHostAPI_General_Res       Response;
  CameraHostAPI_Image_Advanced_Para  CameraPara;
  memset(&CameraPara, 0, sizeof(CameraPara));
  CameraPara.ICRType                = Camera_Host_ICR_Type_General;
  CameraPara.ExposurePara.Mode      = Camera_Host_Advanced_Exposure_Mode_Priority;
  CameraPara.ExposurePara.GainAuto  = Camera_Host_Switch_On;
  CameraPara.ExposurePara.MaxGain   = 80;
  CameraPara.ExposurePara.MinGain   = 10;
  CameraPara.ExposurePara.EVValue   = 1;
  CameraPara.ExposurePara.ExpAuto   = Camera_Host_Switch_On;
  CameraPara.ExposurePara.MaxExpTime= Camera_Host_Advanced_Exposure_Time_1_3200;
  CameraPara.ExposurePara.MinExpTime= Camera_Host_Advanced_Exposure_Time_1_30;
  CameraPara.ExposurePara.IrisAuto  = Camera_Host_Switch_On;
  CameraPara.ExposurePara.MaxIrisLevel= Camera_Host_Iris_Level_0;
  CameraPara.ExposurePara.MinIrisLevel = Camera_Host_Iris_Level_7;
  RequestMessage.MessageID        = CAMERA_HOST_API_SET_IMAGE_ADVANCED_EXPOSURE_MODE;
  RequestMessage.DataLength       = sizeof(CameraPara);
  RequestMessage.DataPtr          = (void*)&CameraPara;
  result = ab_host_msg_send(&RequestMessage,(void*)&Response);

  return result;
}
