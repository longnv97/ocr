#ifndef _EVENT_HANDLE_H_
#define _EVENT_HANDLE_H_

#include "../AbilityInit/ability_init.hpp"
#include "../config.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

extern "C" {
    #include <ab_host_api.h>
    #include <ab_cv_api.h>
}


typedef struct
{
    bool new_event;
    // uint8_t *DataPtr;
    uint8_t data[1920*1080*3];
    std::vector<cv::Rect> location;
    std::vector<std::string> text;
} WriteImage_t;

void updateImgToSave(WriteImage_t &writeImg);
void imgHandleInit();
void OpenCvSaveJpeg(CameraHostAPI_StreamIn_GetVideoFrame_Res frameData);
void CameraCvSaveJpeg(std::vector<cv::Rect> location, CameraHostAPI_StreamIn_GetVideoFrame_Res frameData);
void SaveImgEvent(cv::Mat &image, std::string path);
int CameraCvResize( CameraHostAPI_StreamIn_GetVideoFrame_Res &data,  
                    int resize_width, 
                    int resize_height);
int CameraCvResizeToAI( CameraHostAPI_StreamIn_GetVideoFrame_Res &data,
                        int size_width_in, int size_height_in,
                        int resize_width, int resize_height);
void CameraCvToCVMat(uint8_t *DataPtr, cv::Mat &cv_frame);
cv::Mat putBoxToImg(std::vector<cv::Rect> location, cv::Mat image);
void pad_rectangle(cv::Mat image, cv::Rect &rect, int paddingPercent);
void pad_rectangle_from_size(cv::Size size, cv::Rect &rect, int paddingPercent);
void setLabel(cv::Mat& im, const std::string& label, const cv::Point & p);
void cropFrameData(uint8_t *frame, cv::Rect rec, uint8_t *crop);
cv::Mat CameraCvToCVMatCrop(uint8_t* DataPtr, cv::Rect rect);

#endif  /* _EVENT_HANDLE_H_ */