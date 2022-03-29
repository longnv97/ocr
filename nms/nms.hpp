#ifndef _NMS_H_
#define _NMS_H_

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

struct Object{
    cv::Rect    rec;
    int         class_id;
    float       prob;
    std::string text;  
};

void mergeObject(std::vector<Object>& boxes);
void sortObject(std::vector<Object>& boxes);

#endif  /* _NMS_H_ */