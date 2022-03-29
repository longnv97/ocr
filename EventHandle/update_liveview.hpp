#ifndef _UPDATE_LIVEVEW_H_
#define _UPDATE_LIVEVEW_H_


extern "C" {
    #include <ab_host_api.h>
    #include <ab_cv_api.h>
}

typedef struct 
{
    size_t ID;
    cv::Rect2f rec;
    std::string license;
} UpdateLiveview_t;

typedef struct 
{
    std::vector<UpdateLiveview_t> data;
} UpdateParam_t;

extern UpdateParam_t UpdateParam;

void updateLiveviewInit();
void* updateLiveviewThread(void *arg);

#endif  /* _UPDATE_LIVEVEW_H_ */