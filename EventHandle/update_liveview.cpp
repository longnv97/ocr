#include <chrono>
#include <math.h>
#include <pthread.h>
#include <mutex>
#include <vector>
#include <iostream>


#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include "event_handle.hpp"
#include "../config.hpp"
#include "update_liveview.hpp"

UpdateParam_t UpdateParam;

void updateLiveviewInit()
{
    pthread_t pid;
    pthread_create(&pid, NULL, updateLiveviewThread, &UpdateParam);
    pthread_detach(pid);
    std::cout << "update liveview init DONE" << std::endl;
}


void* updateLiveviewThread(void *arg)
{
    std::string s_json_file_path = "/tmp/aiapp/airesult";

    CameraHostAPI_StreamOut_FrameUpdate_Req  frameUpdateReq = {0};
    CameraHostAPI_Display_SimpleFDFR_Struc   fdfr           = {0};
    Camera_Host_Stream_Out_Display_Type      itemType       = Camera_Host_Stream_Out_Display_SimpleFDFR;
    // ------------ Prepare input ------------
    std::vector<size_t> imageWidths, imageHeights;
    imageWidths.push_back(STREAM_FRAME_TO_AI_WIDTH);
    imageHeights.push_back(STREAM_FRAME_TO_AI_HEIGHT);
    int image_size = STREAM_FRAME_TO_AI_WIDTH * STREAM_FRAME_TO_AI_HEIGHT;
    frameUpdateReq.TargetWidth  = STREAM_FRAME_TO_AI_WIDTH;
    frameUpdateReq.TargetHeight = STREAM_FRAME_TO_AI_HEIGHT;
    memset(&fdfr, 0, sizeof(CameraHostAPI_Display_SimpleFDFR_Struc));

    uint32_t itemPointer;
    UpdateParam_t *UpdateParam = (UpdateParam_t*) arg;
    pthread_mutex_t lock;
    pthread_mutex_init(&lock, NULL);

    for (;;)
    {
        // ------------------- Update to liveview --------
        frameUpdateReq.ItemNumber = 0;
        frameUpdateReq.ItemsLength = 0;
        itemPointer = 0;
        memset(frameUpdateReq.Items, 0, STREAM_OUT_ITEMS_MAX_SIZE);
        
        pthread_mutex_lock(&lock);
        // std::cout << "UpdateParam->data.size() : " << UpdateParam->data.size() << std::endl;
        for (auto &obj : UpdateParam->data) 
        {
            cv::Rect live_rec((int) obj.rec.x, (int) obj.rec.y, (int) obj.rec.width, (int) obj.rec.height);              
            pad_rectangle_from_size(cv::Size(1920, 1080), live_rec, 10);
            if (frameUpdateReq.ItemNumber < MAX_FDFR_RESULT) {
                fdfr.Left     = (uint16_t)obj.rec.x;
                fdfr.Right    = (uint16_t)(obj.rec.x + obj.rec.width);
                fdfr.Top      = (uint16_t)obj.rec.y;
                fdfr.Bottom   = (uint16_t)(obj.rec.y + obj.rec.height);
                memset(fdfr.Label, 0, 32);
                std::copy(obj.license.begin(), obj.license.end(), fdfr.Label);
                fdfr.Color    = 0xFF009100; //ARGB8888: Green color

                memcpy(&frameUpdateReq.Items[itemPointer], &itemType, sizeof(Camera_Host_Stream_Out_Display_Type));
                itemPointer += sizeof(Camera_Host_Stream_Out_Display_Type);
                memcpy(&frameUpdateReq.Items[itemPointer], &fdfr, sizeof(CameraHostAPI_Display_SimpleFDFR_Struc));
                itemPointer += sizeof(CameraHostAPI_Display_SimpleFDFR_Struc);
                frameUpdateReq.ItemNumber++;
            }
            if (frameUpdateReq.ItemNumber >= MAX_FDFR_RESULT) {
                break;
            }
        }
        pthread_mutex_unlock(&lock);

        frameUpdateReq.ItemsLength = itemPointer;
        CameraHost_StreamOut_FrameUpdate(&frameUpdateReq);
        json_export(&frameUpdateReq,s_json_file_path);

        usleep(50000);
    }
}





