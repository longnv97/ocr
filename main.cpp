#include "config.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h> 
#include <time.h>
#include <fstream>
#include <sys/types.h>
#include <signal.h>
#include <iomanip>
#include <iostream>

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include <vpu_tools_common.hpp>
#include <vpu/vpu_plugin_config.hpp>
#include <inference_engine.hpp>
#include "ab_camera_cv_sample.h"
#include "json.hpp"

#include "ability_init.hpp"
#include "LicesenPlateRecognize/lpr_process.hpp"
#include "event_handle.hpp"
#include "update_liveview.hpp"


extern "C" {
    #include <ab_host_api.h>
    #include <ab_cv_api.h>
}

int main()
{
    int retryCount = 0;

    if(ab_host_init() != CAMERA_HOST_OK){
        printf("ab_host_init fail\n");
    }

    if(CameraHost_StreamIn_Init() != CAMERA_HOST_OK){
        printf("CameraHost_StreamIn_Init fail\n");
    }
    if(CameraHost_StreamOut_Start() != CAMERA_HOST_OK){
        printf("CameraHost_StreamOut_Init fail\n");
    }
    
    printf("Init CameraCV\n");
    camera_cv_api_init();
    imgHandleInit();
    updateLiveviewInit();
    while(true)
    {
        if (InferenceStart() == 0) {
            break;
        }
        retryCount++;
        printf("Xlink error and retryCount = %d\n",retryCount);
    }

    CameraHost_StreamIn_Deinit();
    CameraHost_StreamOut_Stop();
    camera_cv_api_deinit();
    ab_host_deinit();
    return 1;
}

