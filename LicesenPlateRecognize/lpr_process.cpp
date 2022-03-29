#include "yolov3_tiny_api.hpp"
#include <chrono>
#include <math.h>
#include <pthread.h>
#include <mutex>
#include <vector>
#include "../sort-tracker/ObjectTracker.hpp"
#include "update_liveview.hpp"
#include "lpr_process.hpp"

using namespace std::chrono;

WriteImage_t writeImg;


int InferenceStart()
{
    std::string inputModelPathLPD = "/mnt/sd/model/lpd_416.abgrp";
    std::string inputModelPathOCR = "/mnt/sd/model/yolov3-tiny-ocr.abgrp";
    std::string s_json_file_path= "/tmp/aiapp/airesult";

    char* cp_jsonData = NULL;
    uint32_t ui_jsonSize = 0 ;
    int loopi, loopch;
    uint32_t itemPointer;
    CameraHostAPI_StreamOut_FrameUpdate_Req  frameUpdateReq = {0};
    Camera_Host_Stream_Out_Display_Type      itemType       = Camera_Host_Stream_Out_Display_SimpleFDFR;
    CameraHostAPI_Display_SimpleFDFR_Struc   fdfr           = {0};
    CameraHostAPI_StreamIn_GetVideoFrame_Res frameData      = {0};
    CameraHostAPI_StreamOut_Data_Req         metaUpdateReq  = {0};

    pthread_mutex_t lock;
    pthread_mutex_init(&lock, NULL);

    try
    {
        // -------------------------------------------
        Core ie;
        Yolov3Tiny LPD;
        Yolov3Tiny OCR;
        LPD.initYolov3Tiny(ie, inputModelPathLPD);
        OCR.initYolov3Tiny(ie, inputModelPathOCR);

        // // ------------ Prepare input ------------
        // std::vector<size_t> imageWidths, imageHeights;
        // imageWidths.push_back(STREAM_FRAME_TO_AI_WIDTH);
        // imageHeights.push_back(STREAM_FRAME_TO_AI_HEIGHT);
        // int image_size = STREAM_FRAME_TO_AI_WIDTH * STREAM_FRAME_TO_AI_HEIGHT;
        // frameUpdateReq.TargetWidth  = STREAM_FRAME_TO_AI_WIDTH;
        // frameUpdateReq.TargetHeight = STREAM_FRAME_TO_AI_HEIGHT;
        // memset(&fdfr, 0, sizeof(CameraHostAPI_Display_SimpleFDFR_Struc));

        // -------------------------------------------
        std::vector<DetectionObject> lpd_objects, ocr_objects;
        std::vector<Object> lpd_to_nms_objects, ocr_to_nms_objects, new_track_objects;

        std::vector<ObjectTrace> detected;
        std::unique_ptr<ObjectTracker> m_tracker;
        m_tracker = std::make_unique<ObjectTracker>();
        std::vector<TrackingTrace> tracks;
        size_t old_id = 0, new_id = 0;

        std::vector<cv::Rect> location, live_location;
        std::vector<std::string> text_result;
        std::vector<UpdateLiveview_t> live_update;
        // writeImg.DataPtr = (uint8_t *)malloc(frameData.Frame[0].DataLen);

        pthread_mutex_t lock;
        pthread_mutex_init(&lock, NULL);

        while(true)
        {
            // ------------ Pop out frame ------------
            if(CameraHost_StreamIn_FramePop(&frameData) == CAMERA_HOST_OK) {
                // OpenCvSaveJpeg(frameData);
                // printf("Pop frame, size:%d (expected %dx%dx3)\n", frameData.Frame[0].DataLen, imageInput->getTensorDesc().getDims()[3], imageInput->getTensorDesc().getDims()[2]);
            } else {
                printf("CameraHost_StreamIn_FramePop Fail!!\n");
                usleep(500000);
                continue;
            }


            // cv::Mat img_event = CameraCvToCVMat(frameData.Frame[0].DataPtr);
            // std::string path_to_LPD_event = "/mnt/sd/img_test/";
            // SaveImgEvent(img_event, path_to_LPD_event);
           std::cout << "START>>>>>>>>>>>>>>>>>>> " << std::endl;
            // ------------- LPD process --------------
            auto start = high_resolution_clock::now();
            lpd_objects.clear();
            lpd_to_nms_objects.clear();
            lpd_objects = LPD.inferYolov3Tiny(frameData.Frame[0].DataPtr, cv::Size(1920, 1080), cv::Size(416, 416), THRESHOLD_LPD, 2);
            // std::cout << "LPD OBJECT NUMBER: " << lpd_objects.size() << std::endl;

            if (lpd_objects.size() == 0) live_update.clear();

            for(int i = 0; i < lpd_objects.size(); i++) 
            {
                float confidence = lpd_objects[i].confidence;
                auto label = lpd_objects[i].class_id;
                auto xmin = lpd_objects[i].xmin;
                auto ymin = lpd_objects[i].ymin;
                auto xmax = lpd_objects[i].xmax;
                auto ymax = lpd_objects[i].ymax;
                if (xmin < 0) { xmin = 0; }
                if (ymin < 0) { ymin = 0; }
                if (xmax >= STREAM_FRAME_TO_AI_WIDTH) { xmax = STREAM_FRAME_TO_AI_WIDTH - 1; }
                if (ymax >= STREAM_FRAME_TO_AI_HEIGHT) { ymax = STREAM_FRAME_TO_AI_HEIGHT - 1; }

                if (confidence > THRESHOLD_LPD) //threshold = 0.5
                {

                    Object object;
                    object.class_id = lpd_objects[i].class_id;
                    object.rec.x = lpd_objects[i].xmin;
                    object.rec.y = lpd_objects[i].ymin;
                    object.rec.width = lpd_objects[i].xmax - lpd_objects[i].xmin;
                    object.rec.height = lpd_objects[i].ymax - lpd_objects[i].ymin;
                    object.prob = lpd_objects[i].confidence;
                    lpd_to_nms_objects.push_back(object);
                }
            }
            mergeObject(lpd_to_nms_objects);
            std::cout << "LPD OBJECT NUMBER: " << lpd_to_nms_objects.size() << std::endl;
            // std::cout << "lpd_to_nms_objects number: " << lpd_to_nms_objects.size() << std::endl;
            // ----------------------------------------------
            detected.clear();
            for(auto &object: lpd_to_nms_objects) 
            {
                ObjectTrace obj;
                // obj.label = "licesen plate";
                obj.score = object.prob;
                obj.rect.x = object.rec.x;
                obj.rect.y = object.rec.y;
                obj.rect.width = object.rec.width;
                obj.rect.height = object.rec.height;
                detected.push_back(obj);
            }
           
            if (detected.size() != 0)
            {
                m_tracker->update(detected);
                tracks.clear();
                tracks = m_tracker->getTracks();

                for (auto &track: tracks) 
                {
                    if(!track.isOutOfFrame)
                    {
                        for (auto &obj: live_update)
                        {
                            if (obj.ID == track.m_ID) 
                            {
                                obj.rec = track.m_rect;
                            }
                        }
                        new_id = track.m_ID;
                        if (new_id > old_id)
                        {
                            cv::Rect crop_rec(track.m_rect.x * ALIGN_WIDTH, track.m_rect.y * ALIGN_HEIGHT, track.m_rect.width * ALIGN_WIDTH, track.m_rect.height * ALIGN_HEIGHT);
                            pad_rectangle_from_size(cv::Size(1920, 1080), crop_rec, 10);
                            uint8_t* lpd_bff = new uint8_t[crop_rec.width * crop_rec.height * 3];
                            cropFrameData(frameData.Frame[0].DataPtr, crop_rec, lpd_bff);
                            // std::string path_to_LPD_event = "/mnt/sd/OCR_event_images/";
                            // cv::Mat lpd_img = CameraCvToCVMatCrop(lpd_bff, crop_rec);
                            // SaveImgEvent(lpd_img, path_to_LPD_event);
                            //----------------- OCR ----------------------
                            ocr_objects.clear();
                            ocr_to_nms_objects.clear();
                            ocr_objects = OCR.inferYolov3Tiny(lpd_bff, cv::Size(crop_rec.width, crop_rec.height), cv::Size(320, 320), THRESHOLD_OCR, 36);
                            // std::cout << "ocr_objects number: " << ocr_objects.size() << std::endl;

                            for (int i = 0; i < ocr_objects.size(); i++)
                            {
                                Object object;
                                object.class_id = ocr_objects[i].class_id;
                                object.rec.x = ocr_objects[i].xmin;
                                object.rec.y = ocr_objects[i].ymin;
                                object.rec.width = ocr_objects[i].xmax - ocr_objects[i].xmin;
                                object.rec.height = ocr_objects[i].ymax - ocr_objects[i].ymin;
                                object.prob = ocr_objects[i].confidence;
                                ocr_to_nms_objects.push_back(object);
                            }
                            mergeObject(ocr_to_nms_objects);
                            // std::cout << "ocr_to_nms_objects number: " << ocr_to_nms_objects.size() << std::endl;
                            sortObject(ocr_to_nms_objects);
                            std::string text = getText(ocr_to_nms_objects);
                            track.label = text;
                            std::cout << "LICENSE : " << text << std::endl;
                            // -----------------------------------------------
                            
                            location.push_back(crop_rec);
                            text_result.push_back(text);
                            live_update.push_back({track.m_ID, track.m_rect, track.label});
                            // -----------------------------------------------
                            delete(lpd_bff);
                            old_id = new_id;
                        }
                    }
                    else
                    {

                        int i = 0;
                        while (i < live_update.size())
                        {
                            if (live_update[i].ID == track.m_ID) 
                            {
                                live_update.erase(live_update.begin() + i);
                            }
                            i++;
                        }
                    }
                }
            }
            auto stop = high_resolution_clock::now();           
            auto time = duration_cast<milliseconds>(stop - start);
            std::cout << "Time taken by Licesen Plate Recognize: " << time.count() << " milliseconds" << std::endl;

            std::cout << "live_update.size() : " << live_update.size() << std::endl;

            pthread_mutex_lock(&lock);
            UpdateParam.data.clear();
            for (int i = 0; i < live_update.size(); i++)
            {
                
                UpdateParam.data.push_back(live_update[i]);
                
            }
            pthread_mutex_unlock(&lock);

            if (location.size())
            {
                writeImg.new_event = true;
                writeImg.location = location;
                writeImg.text = text_result;

                if (frameData.Frame[0].DataPtr != NULL && frameData.Frame[0].DataLen == 1920*1080*3)
                    memcpy(writeImg.data, frameData.Frame[0].DataPtr, (size_t)frameData.Frame[0].DataLen);

                updateImgToSave(writeImg);
                
                location.clear();
                text_result.clear();
            }

            usleep(POP_FRAME_INTERVAL);
        }
    }
    catch(const std::exception& error)
    {
        printf("[AB DBG] catch exception\n");
        std::cerr << error.what() << std::endl;
        return -1;
    }
    catch (...) {
        printf("[AB DBG] catch ... unexpected error\n");
        return -1;
    }
    return -1;
}
