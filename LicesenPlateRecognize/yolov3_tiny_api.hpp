#ifndef _YOLOV3_TINY_API_H_
#define _YOLOV3_TINY_API_H_

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

#include <inference_engine.hpp>

#include <vpu_tools_common.hpp>
#include <vpu/vpu_plugin_config.hpp>

#include "nms.hpp"
#include "../config.hpp"

extern "C" {
    #include <ab_host_api.h>
    #include <ab_cv_api.h>
}

using namespace InferenceEngine;


typedef struct DetectionObject {
    int xmin, ymin, xmax, ymax, class_id;
    float confidence;

    DetectionObject(double x, double y, double h, double w, int class_id, float confidence, float h_scale, float w_scale) :
        xmin{static_cast<int>((x - w / 2) * w_scale)},
        ymin{static_cast<int>((y - h / 2) * h_scale)},
        xmax{static_cast<int>(this->xmin + w * w_scale)},
        ymax{static_cast<int>(this->ymin + h * h_scale)},
        class_id{class_id},
        confidence{confidence} {}

    bool operator <(const DetectionObject &s2) const {
        return this->confidence < s2.confidence;
    }
    bool operator >(const DetectionObject &s2) const {
        return this->confidence > s2.confidence;
    }
} DetectionObject;

class Yolov3Tiny
{
    public:
        int initYolov3Tiny(Core &ie, std::string path_to_model);
        std::vector<DetectionObject> inferYolov3Tiny(uint8_t *dataPrt, cv::Size input_img_size, cv::Size shape_model, const double threshold, int classes_number);
    private:
        InferRequest infer_request_yolov3_tiny;
        std::string imageInputName, imInfoInputName;
        std::string outputName_1;
        CDataPtr outputInfo_1;
        std::string outputName_2;
        CDataPtr outputInfo_2;
};

void ParseYOLOV3Output(const Blob::Ptr blob,
                       const float* output_blob, 
                       std::vector<float> anchors,
                       const unsigned long resized_im_h, const unsigned long resized_im_w, 
                       const unsigned long original_im_h, const unsigned long original_im_w,
                       const double threshold, std::vector<DetectionObject> &objects,
                       int classes);

std::string getText(std::vector<Object> objects);
#endif  /* _YOLOV3_TINY_API_H_ */