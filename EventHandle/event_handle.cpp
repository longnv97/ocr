#include <time.h>
#include <chrono>
#include "../datetime.hpp"
#include "event_handle.hpp"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <queue>
#include <pthread.h>
#include <mutex>

using namespace std;
std::mutex g_mutex;
const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y%m%d%h%s", &tstruct);
    return buf;
}
static std::queue<WriteImage_t> WriteImageQueue;

void updateImgToSave(WriteImage_t &writeImg)
{
    std::lock_guard<std::mutex> lk(g_mutex);
    if (WriteImageQueue.size() < 3)
        WriteImageQueue.push(writeImg);
}

void* writeImgThread(void *arg)
{
    WriteImage_t writeImg;
    cv::Mat img_event;
    std::string path_to_LPD_event;
    for (;;)
    {
        std::lock_guard<std::mutex> lk(g_mutex);
        if (WriteImageQueue.size())
        {
            CameraCvToCVMat(WriteImageQueue.front().data, img_event);
            if (writeImg.location.size())
            {
                for (int i = 0; i < writeImg.location.size(); i++)
                {
                    cv::rectangle(img_event, writeImg.location[i], cv::Scalar(0, 255, 0), 2);
                    cv::putText(img_event, writeImg.text[i], cv::Point(writeImg.location[i].x, writeImg.location[i].y - 10), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 0, 0), 2);              
                }
            }
            if (writeImg.new_event)
            {
                path_to_LPD_event = "/mnt/sd/LPD_event_images/";
                SaveImgEvent(img_event, path_to_LPD_event);
            }
            WriteImageQueue.pop();
        }
        else
        {
            usleep(50000);
        }    
    }
}

void imgHandleInit()
{
    pthread_t pid;
    pthread_create(&pid, NULL, writeImgThread, NULL);
    pthread_detach(pid);
    std::cout << "image handle init done" << std::endl;
}


void pad_rectangle(cv::Mat image, cv::Rect &rect, int paddingPercent)
{
    float padding = (float)(paddingPercent) / 100;
    rect.x = rect.x - (rect.width * padding) / 2;
    rect.y = rect.y -  (rect.height * padding) / 2;
    rect.width = rect.width + (rect.width * padding);
    rect.height = rect.height + (rect.height * padding);

    if (rect.x < 0) rect.x = 0;
    if (rect.y < 0) rect.y = 0; 
    if ((rect.x + rect.width) > image.cols) rect.width = image.cols - rect.x;
    if ((rect.y + rect.height) > image.rows) rect.height = image.rows - rect.y;
}

void pad_rectangle_from_size(cv::Size size, cv::Rect &rect, int paddingPercent)
{
    float padding = (float)(paddingPercent) / 100;
    rect.x = rect.x - (rect.width * padding) / 2;
    rect.y = rect.y -  (rect.height * padding) / 2;
    rect.width = rect.width + (rect.width * padding);
    rect.height = rect.height + (rect.height * padding);

    if (rect.x < 0) rect.x = 0;
    if (rect.y < 0) rect.y = 0; 
    if ((rect.x + rect.width) > size.width) rect.width = size.width - rect.x;
    if ((rect.y + rect.height) > size.height) rect.height = size.height - rect.y;
}


void SaveImgEvent(cv::Mat &image, std::string path)
{
    static int index = 0;
    datetime_t datetime = getDatetime();
    std::string time2str = time2string(datetime.hour, datetime.minute, datetime.second);
    std::string date2str = date2string(datetime.year, datetime.month, datetime.date);

    struct stat st = {0};
    if (stat((path + date2str).c_str(), &st) == -1) 
    {
        std::string cmd_result = getCmdResult("ls " + path + " | wc -l");
        if (std::stoi(cmd_result) > 5)
        {
            std::string oldest_folder = getCmdResult("ls -c " + path + " | tail -1");
            std::string sys_cmd = "rm -rf " + path + oldest_folder;
    
            std::cout << "DELETE FOLDER : " << sys_cmd << std::endl; 
            system(sys_cmd.c_str());
        }
        std::cout << "CREATE FOLDER : " << path + date2str << std::endl;
        mkdir((path + date2str).c_str(), 0700);
    }
    std::string path_out = path + date2str + "/" + time2str + "_" + std::to_string(index++) +".jpg";
    std::cout << "path_out : " << path_out << std::endl;
    char path_image_event_mask[80];
    
    sprintf(path_image_event_mask, path_out.c_str());
    cv::imwrite(path_image_event_mask, image); 
}



void OpenCvSaveJpeg(CameraHostAPI_StreamIn_GetVideoFrame_Res frameData)
{
    CAMERA_CV_API_MAT_ST src;

    src.pv_data = frameData.Frame[0].DataPtr;
    src.i_data_type = CAMERA_CV_DATA_TYPE_8UC3;
    src.s_size.i_width = MAIN_STREAM_FRAME_WIDTH;
    src.s_size.i_height = MAIN_STREAM_FRAME_HEIGHT;
    src.i_step = CAMERA_CV_API_AUTO_STEP;

    CAMERA_CV_API_MAT_ST dst;
    int32_t result = camera_cv_api_cvt_color(&src, &dst, CAMERA_CV_CONVERT_RGB888Planar_2_RGB888Packed);

    cv::Mat cv_frame(MAIN_STREAM_FRAME_HEIGHT, MAIN_STREAM_FRAME_WIDTH, CV_8UC3, (char*)dst.pv_data);
    camera_cv_api_mem_free(&dst.pv_data);
    // cv::Mat Bands[3], merged;
    // cv::split(cv_frame, Bands);
    // std::vector<cv::Mat> channels = {Bands[2],Bands[1],Bands[0]};
    // cv::merge(channels, merged);
    // cv::rectangle(merged, point_1, point_2, cv::Scalar(255, 0, 0), 2);

    char path_image_event_mask[80];
    sprintf(path_image_event_mask, "/mnt/sd/TestData/output_images/OpenCvSaveJpeg/");
    SaveImgEvent(cv_frame, path_image_event_mask);
    // cv::imwrite(path_image_event_mask, cv_frame);
    std::cout << ">>>>>>>>>>>>>>>>>>>OpenCvSaveJpeg" << std::endl;

}


void CameraCvToCVMat(uint8_t* DataPtr, cv::Mat &cv_frame)
{
    CAMERA_CV_API_MAT_ST src;

    src.pv_data = DataPtr;
    src.i_data_type = CAMERA_CV_DATA_TYPE_8UC3;
    src.s_size.i_width = MAIN_STREAM_FRAME_WIDTH;
    src.s_size.i_height = MAIN_STREAM_FRAME_HEIGHT;
    src.i_step = CAMERA_CV_API_AUTO_STEP;

    CAMERA_CV_API_MAT_ST dst;
    int32_t result = camera_cv_api_cvt_color(&src, &dst, CAMERA_CV_CONVERT_RGB888Planar_2_BGR888Packed);
    cv_frame = cv::Mat(MAIN_STREAM_FRAME_HEIGHT, MAIN_STREAM_FRAME_WIDTH, CV_8UC3, (char*)dst.pv_data);
    cv::cvtColor(cv_frame, cv_frame, cv::COLOR_BGR2RGB);

    camera_cv_api_mem_free(&dst.pv_data);
}

cv::Mat putBoxToImg(std::vector<cv::Rect> location, cv::Mat image)
{
    if (location.size())
    {
        for (int i = 0; i < location.size(); i++)
        {
            cv::rectangle(image, location[i], cv::Scalar(0, 255, 0), 2);
        }
    }
    return image;
}



void CameraCvSaveJpeg(std::vector<cv::Rect> location, CameraHostAPI_StreamIn_GetVideoFrame_Res frameData)
{
	// CameraHostAPI_StreamIn_GetVideoFrame_Res frameData      = {0};
	// if(CameraHost_StreamIn_FramePop(&frameData) == CAMERA_HOST_OK) {
		CAMERA_CV_API_MAT_ST src;
		CAMERA_CV_API_JPEG_RESULT_ST dst;	
		CAMERA_CV_API_ROI_ST roi;
		CAMERA_CV_API_SIZE_ST dsize;
		/*ROI Setting*/
		roi.i_x = 0;
		roi.i_y = 0;
		roi.i_width = MAIN_STREAM_FRAME_WIDTH;
		roi.i_height = MAIN_STREAM_FRAME_HEIGHT;
		/*Dst Size Setting*/
		dsize.i_width = MAIN_STREAM_FRAME_WIDTH;
		dsize.i_height = MAIN_STREAM_FRAME_HEIGHT;
		/*Src Parm Setting*/
		src.pv_data = frameData.Frame[0].DataPtr;
		src.i_data_type = CAMERA_CV_DATA_TYPE_8UC3;
		src.s_size.i_width = MAIN_STREAM_FRAME_WIDTH;
		src.s_size.i_height = MAIN_STREAM_FRAME_HEIGHT;
		src.i_step = CAMERA_CV_API_AUTO_STEP;
		
		/*Convert Jpeg*/
		int32_t result = camera_cv_api_sub_img_cvt_jpeg_and_resize(&src, &dst, CAMERA_CV_CONVERT_JPEG_BGR888Planar_2_JPEG, CAMERA_CV_JPEG_QUALITY_MEDIAN, &roi, &dsize, 1);
		
		// printf("Convert Result = %d\n", result);	

		if(!result)
		{
			/*Save File*/
			FILE * pFile = fopen ( "/mnt/sd/TestData/output_images/CameraCvsave.jpg" , "wb");
			if(pFile)
			{
				fwrite(dst.pv_data, sizeof(uint8_t), dst.i_data_size, pFile);
				fclose (pFile);
			}
			/*Free output*/
			camera_cv_api_mem_free(&dst.pv_data);
		}
	// } 
    cv::Mat event_image = cv::imread("/mnt/sd/TestData/output_images/CameraCvsave.jpg");
    cv::Mat license_plate_image;
                    
    for (int i = 0; i < location.size(); i++)
    {
        // event_image.clone()(location[i]).copyTo(license_plate_image);

        cv::rectangle(event_image, location[i], cv::Scalar(0, 255, 0), 2);
        // cv::putText(event_image, text[i], cv::Point(location[i].x, location[i].y - 10), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 0, 0), 2);
    }
    std::string path_to_LPD_event = "/mnt/sd/LPD_event_images/";
    std::string path_to_LPR_event = "/mnt/sd/TestData/output_images/LPR_event_images/";
    SaveImgEvent(event_image, path_to_LPD_event);
    // OpenCvOutputHandle(license_plate_image, path_to_LPD_event);
}


int CameraCvResize( CameraHostAPI_StreamIn_GetVideoFrame_Res &data, int resize_width, int resize_height)
{
    CAMERA_CV_API_MAT_ST src;
    CAMERA_CV_API_MAT_ST dst;
    CAMERA_CV_API_SIZE_ST dsize;

    src.pv_data = data.Frame[0].DataPtr;
    src.s_size.i_width = MAIN_STREAM_FRAME_WIDTH;
    src.s_size.i_height = MAIN_STREAM_FRAME_HEIGHT;
    src.i_data_type = CAMERA_CV_DATA_TYPE_8UC3;
    src.i_step = CAMERA_CV_API_AUTO_STEP;

    dsize.i_width = resize_width;
    dsize.i_height = resize_height;
    int32_t result = camera_cv_api_resize(&src, &dst, &dsize);

    data.Frame[0].DataPtr = (uint8_t*) dst.pv_data;
    data.Frame[0].DataLen = resize_width*resize_height*3;

    camera_cv_api_mem_free(&dst.pv_data);
    return result;
}

int CameraCvResizeToAI(CameraHostAPI_StreamIn_GetVideoFrame_Res &data, int size_width_in, int size_height_in, int resize_width, int resize_height)
{
    CAMERA_CV_API_MAT_ST src;
    CAMERA_CV_API_MAT_ST dst;
    CAMERA_CV_API_SIZE_ST dsize;
    src.pv_data = data.Frame[0].DataPtr;
    src.s_size.i_width = size_width_in;
    src.s_size.i_height = size_height_in;
    src.i_data_type = CAMERA_CV_DATA_TYPE_8UC3;
    src.i_step = CAMERA_CV_API_AUTO_STEP;

    dsize.i_width = resize_width;
    dsize.i_height = resize_height;

    int32_t result = camera_cv_api_resize(&src, &dst, &dsize);

    data.Frame[0].DataPtr = (uint8_t*) dst.pv_data;
    data.Frame[0].DataLen = resize_width*resize_height*3;
    camera_cv_api_mem_free(&dst.pv_data);
    return result;
}

void setLabel(cv::Mat& im, const std::string& label, const cv::Point & p) 
{
    int fontface = cv::FONT_HERSHEY_SIMPLEX;
    double scale = 0.7;
    int thickness = 1;
    int baseline = 0;

    cv::Size text_size = cv::getTextSize(label, fontface, scale, thickness, &baseline);
    auto text_position = p;
    text_position.x = std::max(0, p.x);
    text_position.y = std::max(text_size.height, p.y);

    cv::rectangle(im, text_position + cv::Point(0, baseline), text_position + cv::Point(text_size.width, -text_size.height), CV_RGB(50, 205, 50), cv::FILLED);
    cv::putText(im, label, text_position, fontface, scale, CV_RGB(255, 255, 255), thickness, 8);
}

typedef struct data_t
{
    int i;
    int j;
    int index;
    cv::Rect rec;
    uint8_t* input;
    uint8_t* output = new uint8_t[rec.width * rec.height];
}data_t;



void* worker_1(void *arg)
{
    data_t* wk = (data_t*) arg;
    wk->output[wk->index] = wk->input[(wk->i + wk->rec.y) * 1920 + (wk->j + wk->rec.x)];
}

void* worker_2(void *arg)
{
    data_t* wk = (data_t*) arg;
    wk->output[wk->index + wk->rec.width * wk->rec.height] = wk->input[1920*1080 + (wk->i + wk->rec.y) * 1920 + (wk->j + wk->rec.x)];
}

void* worker_3(void *arg)
{
    data_t* wk = (data_t*) arg;
    wk->output[wk->index + wk->rec.width * wk->rec.height * 2] = wk->input[1920*1080*2 + (wk->i + wk->rec.y) * 1920 + (wk->j + wk->rec.x)];
}
data_t crop_data_1, crop_data_2, crop_data_3;
void cropFrameData(uint8_t *frame, cv::Rect rec, uint8_t *crop)
{
    int index = 0;
    uint8_t *buffer = new uint8_t[rec.width * rec.height * 3];

    for (int i = 0; i < rec.height; i++)
    {
        for (int j = 0; j < rec.width; j++)
        {
            buffer[index] = frame[(i + rec.y) * 1920 + (j + rec.x)];
            buffer[rec.width * rec.height + index] = frame[1920*1080 + (i + rec.y) * 1920 + (j + rec.x)];
            buffer[rec.width * rec.height * 2 + index++] = frame[1920*1080*2 + (i + rec.y) * 1920 + (j + rec.x)];

            // crop_data_1.i = i;
            // crop_data_1.j = j;
            // crop_data_1.index = index;
            // crop_data_1.input = frame;

            // crop_data_2.i = i;
            // crop_data_2.j = j;
            // crop_data_2.index = index;
            // crop_data_2.input = frame;

            // crop_data_3.i = i;
            // crop_data_3.j = j;
            // crop_data_3.index = index;
            // crop_data_3.input = frame;
          

            // pthread_t id1, id2, id3;
            // pthread_create(&id1, NULL, worker_1, (void*) &crop_data_1);
            // pthread_create(&id2, NULL, worker_2, (void*) &crop_data_2);
            // pthread_create(&id3, NULL, worker_3, (void*) &crop_data_3);
            // pthread_join(id1, NULL);
            // pthread_join(id2, NULL);
            // pthread_join(id3, NULL);



            // index++;
            // std::cout << "xong vong for: " << i << " " <<j<<" " << index << std::endl;
        }
    }
    // std::cout << "xong vong for: " << index << std::endl;
    // memcpy(crop, crop_data_1.output, rec.width * rec.height);
    // memcpy(crop + rec.width * rec.height, crop_data_2.output, rec.width * rec.height);
    // memcpy(crop + rec.width * rec.height * 2, crop_data_3.output, rec.width * rec.height);
    memcpy(crop, buffer, rec.width * rec.height * 3);
    delete(buffer);
}

cv::Mat CameraCvToCVMatCrop(uint8_t* DataPtr, cv::Rect rect)
{
    CAMERA_CV_API_MAT_ST src;

    src.pv_data = DataPtr;
    src.i_data_type = CAMERA_CV_DATA_TYPE_8UC3;
    src.s_size.i_width = rect.width;
    src.s_size.i_height = rect.height;
    src.i_step = CAMERA_CV_API_AUTO_STEP;

    CAMERA_CV_API_MAT_ST dst;
    int32_t result = camera_cv_api_cvt_color(&src, &dst, CAMERA_CV_CONVERT_RGB888Planar_2_BGR888Packed); 
    cv::Mat cv_frame(rect.height, rect.width, CV_8UC3, (char*)dst.pv_data);
    cv::cvtColor(cv_frame, cv_frame, cv::COLOR_BGR2RGB);

    camera_cv_api_mem_free(&dst.pv_data);

    return cv_frame;
    
}