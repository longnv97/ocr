/*Header Files*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h> 
#include <time.h>
#include <fstream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <time.h>
#include <sys/time.h>
#include <iostream>
#include <typeinfo>
#include <sstream>
#include <string>
#include <algorithm>
#include <vector>

extern "C" {
    #include <ab_host_api.h>
    #include <ab_cv_api.h>
}
/*Header Files*/

/*Definitions*/
#define DEFAULT_SAMPLE_IMG_PATH	"/mnt/sd/TestData/cv_test.yuv"

#define WARP_AFFINE_SAMPLE_RESULT_PATH	"/mnt/sd/TestData/output/warpaffine.yuv"
#define WARP_AFFINE_SAMPLE_IMG_SRC_WIDTH	1920.0
#define WARP_AFFINE_SAMPLE_IMG_SRC_HEIGHT	1080.0
#define WARP_AFFINE_SAMPLE_IMG_DST_WIDTH	800.0
#define WARP_AFFINE_SAMPLE_IMG_DST_HEIGHT	600.0

#define RESIZE_SAMPLE_RESULT_PATH	"/mnt/sd/TestData/output/resize.yuv"
#define RESIZE_SAMPLE_IMG_SRC_WIDTH		1920
#define RESIZE_SAMPLE_IMG_SRC_HEIGHT	1080
#define RESIZE_SAMPLE_IMG_DST_WIDTH		1280
#define RESIZE_SAMPLE_IMG_DST_HEIGHT	720

#define FEATURE_DETECT_SAMPLE_IMG_SRC_WIDTH			1920
#define FEATURE_DETECT_SAMPLE_IMG_SRC_HEIGHT		1080

#define CVTCOLOR_SAMPLE_RESULT_PATH	"/mnt/sd/TestData/output/cvtcolor.raw"
#define CVTCOLOR_SAMPLE_IMG_SRC_WIDTH		1920
#define CVTCOLOR_SAMPLE_IMG_SRC_HEIGHT		1080

#define VCONCAT_SAMPLE_RESULT_PATH	"/mnt/sd/TestData/output/v_concat.yuv"
#define VCONCAT_SAMPLE_IMG_SRC_WIDTH		1920
#define VCONCAT_SAMPLE_IMG_SRC_HEIGHT		1080

#define HCONCAT_SAMPLE_RESULT_PATH	"/mnt/sd/TestData/output/h_concat.yuv"
#define HCONCAT_SAMPLE_IMG_SRC_WIDTH		1920
#define HCONCAT_SAMPLE_IMG_SRC_HEIGHT		1080

#define CVTJPEG_SAMPLE_RESULT_PATH	"/mnt/sd/TestData/output/cvtjpeg.jpg"
#define CVTJPEG_GRAY_SAMPLE_RESULT_PATH	"/mnt/sd/TestData/output/cvtjpeg_gray.jpg"
#define CVTJPEG_SAMPLE_IMG_SRC_WIDTH		1920
#define CVTJPEG_SAMPLE_IMG_SRC_HEIGHT		1080

#define MEDIAN_BLUR_SAMPLE_RESULT_PATH	"/mnt/sd/TestData/output/medianblur.yuv"
#define MEDIAN_BLUR_SAMPLE_IMG_SRC_WIDTH		1920
#define MEDIAN_BLUR_SAMPLE_IMG_SRC_HEIGHT		1080

#define BOX_FILTER_SAMPLE_RESULT_PATH	"/mnt/sd/TestData/output/boxfilter.yuv"
#define BOX_FILTER_SAMPLE_IMG_SRC_WIDTH			1920
#define BOX_FILTER_SAMPLE_IMG_SRC_HEIGHT		1080

#define DILATE_SAMPLE_RESULT_PATH	"/mnt/sd/TestData/output/dilate.yuv"
#define DILATE_SAMPLE_IMG_SRC_WIDTH				1920
#define DILATE_SAMPLE_IMG_SRC_HEIGHT			1080

#define ERODE_SAMPLE_RESULT_PATH	"/mnt/sd/TestData/output/erode.yuv"
#define ERODE_SAMPLE_IMG_SRC_WIDTH				1920
#define ERODE_SAMPLE_IMG_SRC_HEIGHT				1080

#define LUT_SAMPLE_RESULT_PATH	"/mnt/sd/TestData/output/lut.yuv"
#define LUT_SAMPLE_IMG_SRC_WIDTH				1920
#define LUT_SAMPLE_IMG_SRC_HEIGHT				1080

#define MERGE_SAMPLE_RESULT_PATH	"/mnt/sd/TestData/output/grayscale.bmp"
#define MERGE_SAMPLE_IMG_SRC_WIDTH		1920
#define MERGE_SAMPLE_IMG_SRC_HEIGHT		1080

#define RGB2BGR_SAMPLE_INPUT_PATH	"/mnt/sd/TestData/input/rgb.raw"
#define RGB2BGR_SAMPLE_RESULT_PATH	"/mnt/sd/TestData/output/rgb2bgr.bmp"
#define RGB2BGR_SAMPLE_IMG_SRC_WIDTH			672
#define RGB2BGR_SAMPLE_IMG_SRC_HEIGHT			384

#define TASK_PARAM_MAX_SIZE 128

#define MAX_FILE_BUFFER_COUNT 4
#define MAX_FILE_BUFFER_LENGTH (1920 * 1080 * 3)
#define STEP_OF_MAT(x) ((((x).i_step)==CAMERA_CV_API_AUTO_STEP)?((x).s_size.i_width):((x).i_step))
#define DET(a00,a01,a10,a11) ((a00)*(a11)-(a01)*(a10))
#define MAX_DIM 8
#define HALF_DIM (MAX_DIM>>1)
#define USE_INVERSE_TRANSFORMATION 1
/*Definitions*/

/*Type definition*/
typedef void (*TaskFunc)();
typedef void (*ParamTaskFunc)(void* param);

typedef enum{
	IMG_TYPE_RGB, 
	IMG_TYPE_BGR
}IMG_TYPE;

typedef enum{
	OPERATION_AND,
	OPERATION_OR,
	OPERATION_NOT,
	OPERATION_XOR
} OPERATION_TYPE;

typedef struct{
	TaskFunc task;
	ParamTaskFunc paramTask;
	bool hasParam;
	uint8_t param[TASK_PARAM_MAX_SIZE];
}TASK_INFO_ST;

template <typename T>
struct Point_t{
	T X;
	T Y;
};

typedef Point_t<double> Point;
typedef Point_t<float> PointF;

class Semaphore
{
	unsigned int m_ThreadMaxNum;
	std::mutex m_Semaphore_mutex;
	std::condition_variable m_condition;
public:
	Semaphore(unsigned int ThreadMaxNum)
	{
		m_ThreadMaxNum = ThreadMaxNum;
	}

	~Semaphore()
	{
	}

	void Enter()
	{
		std::unique_lock<std::mutex> lock{ m_Semaphore_mutex };
		m_condition.wait(lock,
			[&]()->bool{return m_ThreadMaxNum > 0;}
		);
		--m_ThreadMaxNum;
	}

	void Leave()
	{
		std::lock_guard<std::mutex> lock{ m_Semaphore_mutex };
		++m_ThreadMaxNum;
		m_condition.notify_one();
	}
};
/*Type definition*/

/*Globals*/
static uint8_t* g_SampleBuffer;
static uint8_t* g_FileBuffer[MAX_FILE_BUFFER_COUNT];
static uint8_t *g_FrameBuffer[2];
static Semaphore* p_sem;
static std::queue<TASK_INFO_ST*> TaskQueue;
static std::thread *p_th = NULL;
static bool inited = false;
static std::mutex g_mutex;
static uint8_t g_CheckMatEnable = 0;
static uint8_t g_CheckPerfEnable = 0;
static uint8_t g_AdvanceTest = 0;
/*Globals*/


static uint32_t CameraCvSamplePerfGetCurrent(void)
{
	uint32_t  rt;
	struct  timeval    tv;
	struct  timezone   tz;

	gettimeofday(&tv,&tz);
	rt = (uint32_t)(((uint64_t)(tv.tv_sec) * 1000000) + (uint64_t)((tv.tv_usec) & 0xFFFFFFFF));
	return rt;
}

static void TaskHandler(void)
{
	while(inited)
	{
		p_sem->Enter();
		g_mutex.lock();
		TASK_INFO_ST* pTaskInfo = TaskQueue.front();
		TaskQueue.pop();
		g_mutex.unlock();
		if(pTaskInfo->hasParam)
		{
			pTaskInfo->paramTask(pTaskInfo->param);
		}
		else
		{
			int start = CameraCvSamplePerfGetCurrent();
			pTaskInfo->task();
			int end = CameraCvSamplePerfGetCurrent();
			if(g_CheckPerfEnable)
			{
				printf("CV API Cost Time: %d us\n", end - start);
			}
		}

		delete pTaskInfo;
	}
}

static void Invoke(ParamTaskFunc task, void *param, uint32_t size)
{
	if(inited)
	{
	
		TASK_INFO_ST *pTaskInfo = new TASK_INFO_ST;
		pTaskInfo->hasParam = true;
		pTaskInfo->paramTask = task;
		memcpy(pTaskInfo->param, param, size);
		g_mutex.lock();
		TaskQueue.push(pTaskInfo);
		g_mutex.unlock();
		p_sem->Leave();
	}
	else
	{
		printf("Please Init first\n");
	}
}

static void Invoke(TaskFunc task)
{
	if(inited)
	{
		TASK_INFO_ST *pTaskInfo = new TASK_INFO_ST;
		pTaskInfo->hasParam = false;
		pTaskInfo->task =task;
		g_mutex.lock();
		TaskQueue.push(pTaskInfo);
		g_mutex.unlock();
		p_sem->Leave();
	}
	else
	{
		printf("Please Init first\n");
	}
}

static Camera_Host_Result_Type CameraCvSample_StreamIn_FramePop(CameraHostAPI_StreamIn_GetVideoFrame_Res* ResPara)
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

int CameraCvSampleImread(const char* puc_filename, void *pv_buffer, uint32_t ui_max_len = (uint32_t)-1)
{
	FILE * pFile;
	long lSize;
	size_t result;

	pFile = fopen ( puc_filename , "rb" );
	if (pFile==NULL) {printf ("File error\n"); return 0;}

	// obtain file size:
	fseek (pFile , 0 , SEEK_END);
	lSize = ftell (pFile);
	rewind (pFile);

	if(lSize > (int64_t)ui_max_len) lSize = ui_max_len;

	if (pv_buffer == NULL) {
		printf ("Memory error\n"); 
		fclose(pFile);
		return 0;
	}

	// copy the file into the buffer:
	result = fread (pv_buffer,1,lSize,pFile);
	if ((int64_t)result != lSize) {
		printf("Reading error\n");
		fclose(pFile);
		return 0;
	}

	/* the whole file is now loaded in the memory buffer. */

	// terminate
	fclose (pFile);
	return lSize;
}

int CameraCvSampleReadToFileBuffer(const char* puc_filename, uint32_t ui_index)
{
	if(ui_index < MAX_FILE_BUFFER_COUNT)
	{
		int32_t result = CameraCvSampleImread(puc_filename, g_FileBuffer[ui_index], MAX_FILE_BUFFER_LENGTH);
		printf("Read %s = %d\n", puc_filename, result);
		return result;
	}
	else
	{
		printf("Invalid Index\n");
		return -1;
	}
}

int CameraCvSampleBufferWrite(const char* pc_filename, void *pv_buffer, int size)
{
	if(g_CheckMatEnable)
	{
		FILE * pFile = fopen ( pc_filename , "wb" );
		if(pFile)
		{
			fwrite(pv_buffer, sizeof(uint8_t), size, pFile);
			fclose (pFile);
		}
	
		printf("Write %s Done\n", pc_filename);
	}
	return 0;
}

int CameraCvSampleImwrite(const char* pc_filename, CAMERA_CV_API_MAT_ST &s_mat)
{
	if(g_CheckMatEnable)
	{
		FILE * pFile = fopen ( pc_filename , "wb" );
		if(pFile)
		{
			uint8_t* ptr = (uint8_t*)s_mat.pv_data;
			uint32_t size_per_pixel = ((s_mat.i_data_type >> 3) + 1) * (1<<(((s_mat.i_data_type)&0x07)>>1));
			int stride = STEP_OF_MAT(s_mat);
			
			for(int rows = 0; rows < s_mat.s_size.i_height;rows++)
			{
				int size = s_mat.s_size.i_width * size_per_pixel;
				fwrite(ptr, sizeof(uint8_t), size, pFile);
				ptr += size_per_pixel * stride;
			}
			fclose (pFile);
		}
	
		printf("Write %s Done\n", pc_filename);
	}
	return 0;
}

static void CameraCvSampleImwriteBmp(const char *filename, unsigned char *img, int w, int h, IMG_TYPE e_img_type = IMG_TYPE_RGB)
{
	FILE *f;

	int filesize = 54 + 3 * w * h;  //w is your image width, h is image height, both int

	if(e_img_type == IMG_TYPE_RGB)
	{
		for(int i=0; i<w; i++)
		{
		    for(int j=0; j<h; j++)
		    {
		        int x=i; int y=(h-1)-j;
		        uint8_t r = img[(x+y*w)*3+0];
		        uint8_t b = img[(x+y*w)*3+2];

		        img[(x+y*w)*3+2] = r;
		        img[(x+y*w)*3+0] = b;
		    }
		}
	}

	unsigned char bmpfileheader[14] = {'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0};
	unsigned char bmpinfoheader[40] = {40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0};
	unsigned char bmppad[3] = {0,0,0};

	bmpfileheader[ 2] = (unsigned char)(filesize    );
	bmpfileheader[ 3] = (unsigned char)(filesize>> 8);
	bmpfileheader[ 4] = (unsigned char)(filesize>>16);
	bmpfileheader[ 5] = (unsigned char)(filesize>>24);

	bmpinfoheader[ 4] = (unsigned char)(       w    );
	bmpinfoheader[ 5] = (unsigned char)(       w>> 8);
	bmpinfoheader[ 6] = (unsigned char)(       w>>16);
	bmpinfoheader[ 7] = (unsigned char)(       w>>24);
	bmpinfoheader[ 8] = (unsigned char)(       h    );
	bmpinfoheader[ 9] = (unsigned char)(       h>> 8);
	bmpinfoheader[10] = (unsigned char)(       h>>16);
	bmpinfoheader[11] = (unsigned char)(       h>>24);

	f = fopen(filename,"wb");
	if(f==NULL)
	{
		printf("File can not open %s", filename);
		return;
	}
	fwrite(bmpfileheader,1,14,f);
	fwrite(bmpinfoheader,1,40,f);
	for(int i=0; i<h; i++)
	{
	    fwrite(img+(w*(h-i-1)*3),3,w,f);
	    fwrite(bmppad,1,(4-(w*3)%4)%4,f);
	}

	fclose(f);
	printf("Write %s Done\n", filename);	
}

int CameraCvSampleInitMat(CAMERA_CV_API_MAT_ST &s_mat, void* pv_data, int i_width, int i_height, int i_type = CAMERA_CV_DATA_TYPE_8UC1, int i_step = CAMERA_CV_API_AUTO_STEP)
{
	s_mat.pv_data = pv_data;
	s_mat.i_data_type = i_type;
	s_mat.s_size.i_width = i_width;
	s_mat.s_size.i_height = i_height;
	s_mat.i_step = i_step;
	return 0;
}

static CAMERA_CV_API_MAT_ST operator & (CAMERA_CV_API_MAT_ST &s_mat, CAMERA_CV_API_ROI_ST rect)
{
	CAMERA_CV_API_MAT_ST ret = {0};
	int type = s_mat.i_data_type;
	int depth = (1<<(((type)&0x07)>>1));
	int channel = (type >> 3) + 1;
	int stride = STEP_OF_MAT(s_mat);
	int offset = (stride * rect.i_y + rect.i_x) * depth * channel;
	ret.pv_data = (uint8_t*)s_mat.pv_data + offset;
	ret.i_data_type = s_mat.i_data_type;
	ret.s_size.i_width = rect.i_width;
	ret.s_size.i_height = rect.i_height;
	ret.i_step = stride;
	
	return ret;
}

template<class T>
int CameraCvSampleDumpMat(T *data, CAMERA_CV_API_SIZE_ST &s_sz, int channel, int stride)
{
#define MAX_WIDTH_TO_SHOW 16
#define MAX_HEIGHT_TO_SHOW 16

	auto &height = s_sz.i_height;
	auto &width = s_sz.i_width;
	for(int y=0;y<height;y++)
	{
		if(y > MAX_HEIGHT_TO_SHOW)
		{
			printf("......");
			break;
		}

		for(int x=0;x<width;x++)
		{
			if(x > MAX_WIDTH_TO_SHOW)
			{
				printf("...");
				break;
			}
			
			if(channel > 1)
			{
				printf("(");
				for(int i=0;i<channel;i++)
				{
					int idx = (y * stride + x) * channel + i;
					if(typeid(T)==typeid(float))
						printf("%f " , (float)data[idx]);
					else
						printf("%3lld " , (int64_t)data[idx]);
					
				}
				printf(")");

			}
			else
			{
				if(typeid(T)==typeid(float))
					printf("%f, " , (float)data[y * stride + x]);
				else
					printf("%5lld, " , (int64_t)data[y * stride + x]);
				
			}
		}
		printf("\n");
	}
	return 0;
}

int CameraCvSampleShowMat(CAMERA_CV_API_MAT_ST &s_mat, const char* desc)
{
	if(g_CheckMatEnable)
	{
		printf("%s: %dx%d\n", desc, s_mat.s_size.i_width, s_mat.s_size.i_height); 
		int channel = (s_mat.i_data_type >> 3) + 1;
		int type = s_mat.i_data_type & 7;
		int stride = STEP_OF_MAT(s_mat);
		switch(type)
		{
			case CAMERA_CV_DATA_TYPE_8UC1:
				CameraCvSampleDumpMat((uint8_t*)s_mat.pv_data, s_mat.s_size, channel, stride);
				break;
			case CAMERA_CV_DATA_TYPE_8SC1:
				CameraCvSampleDumpMat((int8_t*)s_mat.pv_data, s_mat.s_size, channel, stride);
				break;
			case CAMERA_CV_DATA_TYPE_16UC1:
				CameraCvSampleDumpMat((uint16_t*)s_mat.pv_data, s_mat.s_size, channel, stride);
				break;
			case CAMERA_CV_DATA_TYPE_16SC1:
				CameraCvSampleDumpMat((int16_t*)s_mat.pv_data, s_mat.s_size, channel, stride);
				break;
			case CAMERA_CV_DATA_TYPE_32UC1:
				CameraCvSampleDumpMat((uint32_t*)s_mat.pv_data, s_mat.s_size, channel, stride);			
				break;					
			case CAMERA_CV_DATA_TYPE_32FC1:
				CameraCvSampleDumpMat((float*)s_mat.pv_data, s_mat.s_size, channel, stride);			
				break;					
			default:
				printf("Can not show mat with type %X\n", s_mat.i_data_type);
				break;

		}

		printf("\n");
	}
	
	return 0;
}

int CameraCvSampleShowKeyPoint(CAMERA_CV_API_KEY_POINT_VECTOR_ST &s_keypoints, const char* desc, uint8_t* status = NULL)
{
	if(g_CheckMatEnable)
	{
		int total = s_keypoints.count;
		if(status)
			total = std::count_if (status, status + total, [](uint8_t st){return st;});
		
		printf("%s: total(%d)\n", desc, total); 
		
		for(int idx = 0; idx < s_keypoints.count; idx++)
		{
			if(status == NULL || status[idx]) //status == false => missing
			{
				CAMERA_CV_API_KEY_POINT_ST* tar_key_point = &s_keypoints.ps_key_point[idx];
				
				printf("[%d] (%f, %f)\n", idx, tar_key_point->f_x, tar_key_point->f_y);
			}
		}
		
		printf("\n");
	}
	
	return 0;
}

int CameraCvSampleCvInit(void)
{
	int32_t result = camera_cv_api_init();
	return result;
}

int CameraCvSampleCvDeinit(void)
{
	int32_t result = camera_cv_api_deinit();
	return result;
}

int CameraCvSampleMemoryFree(CAMERA_CV_API_MAT_ST &s_mat)
{
	int32_t result = camera_cv_api_mem_free(&s_mat.pv_data);
	return result;
}

int CameraCvSampleMemoryFree(CAMERA_CV_API_JPEG_RESULT_ST &s_jpeg_result)
{
	int32_t result = camera_cv_api_mem_free(&s_jpeg_result.pv_data);
	return result;
}

int CameraCvSampleMemoryFree(CAMERA_CV_API_KEY_POINT_VECTOR_ST &s_keypoint)
{
	int32_t result = camera_cv_api_mem_free((void**)&s_keypoint.ps_key_point);
	return result;
}

int CameraCvSampleMemoryFree(CAMERA_CV_API_MATCH_VECTOR_ST &s_match)
{
	int32_t result = camera_cv_api_mem_free((void**)&s_match.ps_matches);
	return result;
}

int CameraCvSampleInit(void)
{
	if(!inited) 
	{
		p_sem = new Semaphore(0);
		g_SampleBuffer = new uint8_t[MAX_FILE_BUFFER_LENGTH];
		for(auto &fileBuf:g_FileBuffer)
		{
			fileBuf = new uint8_t[MAX_FILE_BUFFER_LENGTH];
		}
		
		CameraCvSampleImread(DEFAULT_SAMPLE_IMG_PATH, g_SampleBuffer, MAX_FILE_BUFFER_LENGTH);
		CameraCvSampleCvInit();
		
		inited = true;
		printf("create thread\n");
		p_th = new std::thread(TaskHandler);

		Invoke([]{
			CameraHostAPI_StreamIn_GetVideoFrame_Res ResPara;
			auto popFrameResult = CameraCvSample_StreamIn_FramePop(&ResPara);	
			if(!popFrameResult)
			{
				g_FrameBuffer[0] = ResPara.Frame[0].DataPtr;
				g_FrameBuffer[1] = ResPara.Frame[1].DataPtr;
				printf("Frame Buffer: %p %p\n", g_FrameBuffer[0], g_FrameBuffer[1]);
			}
		});
	}
	else
	{
		printf("please deinit first\n");
	}
	
	return 0;
}

int CameraCvSampleDeinit(void)
{
	if(inited)
	{
		Invoke([]{
			inited = false;
			g_mutex.lock();
			TaskQueue = std::queue<TASK_INFO_ST*>();
			g_mutex.unlock();
			CameraCvSampleCvDeinit();
			
			delete [] g_SampleBuffer;
			for(auto &fileBuf:g_FileBuffer)
			{
				delete [] fileBuf;
			}

		});
		p_th->join();
		delete p_th;
		delete p_sem;

		p_th = NULL;
		p_sem = NULL;
	}
	else
	{
		printf("please init first\n");
	}
	return 0;
}

int CameraCvSampleWarpAffine(void)
{
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_MAT_ST dst;	
	CAMERA_CV_API_MAT_ST transform;
	CAMERA_CV_API_SIZE_ST dsize;

	float AffineTransformation[2][3] = {
		{0.0, (WARP_AFFINE_SAMPLE_IMG_SRC_WIDTH - 1) / (WARP_AFFINE_SAMPLE_IMG_DST_HEIGHT - 1), 0.0},
		{-(WARP_AFFINE_SAMPLE_IMG_SRC_HEIGHT - 1) / (WARP_AFFINE_SAMPLE_IMG_DST_WIDTH - 1), 0.0, (WARP_AFFINE_SAMPLE_IMG_SRC_HEIGHT - 1)}
	};

	CameraCvSampleInitMat(src, g_SampleBuffer, WARP_AFFINE_SAMPLE_IMG_SRC_WIDTH, WARP_AFFINE_SAMPLE_IMG_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_8UC1);
	CameraCvSampleInitMat(transform, AffineTransformation, 3, 2, CAMERA_CV_DATA_TYPE_32FC1);	
	dsize.i_width = WARP_AFFINE_SAMPLE_IMG_DST_WIDTH;
	dsize.i_height = WARP_AFFINE_SAMPLE_IMG_DST_HEIGHT;

	int32_t result = camera_cv_api_warp_affine(&src, &dst, &transform, &dsize);

	if(!result)
	{
		CameraCvSampleImwrite(WARP_AFFINE_SAMPLE_RESULT_PATH, dst);
		CameraCvSampleMemoryFree(dst);
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_ROI_ST rect;
			CAMERA_CV_API_MAT_ST subSrc;
			float subWidth = WARP_AFFINE_SAMPLE_IMG_SRC_WIDTH / 2;
			float subHeight = WARP_AFFINE_SAMPLE_IMG_SRC_HEIGHT / 2;
			
			rect.i_x = (int)(WARP_AFFINE_SAMPLE_IMG_SRC_WIDTH / 3);
			rect.i_y = (int)(WARP_AFFINE_SAMPLE_IMG_SRC_HEIGHT / 3);
			rect.i_width = (int)subWidth;
			rect.i_height = (int)subHeight;
			
			subSrc = src & rect; //region mask

			AffineTransformation[0][1] = (subWidth - 1) / (WARP_AFFINE_SAMPLE_IMG_DST_HEIGHT - 1);
			AffineTransformation[1][0] = -(subHeight - 1) / (WARP_AFFINE_SAMPLE_IMG_DST_WIDTH - 1);
			AffineTransformation[1][2] = (subHeight - 1);
			
			result = camera_cv_api_warp_affine(&subSrc, &dst, &transform, &dsize);
			
			if(!result)
			{
				CameraCvSampleImwrite(WARP_AFFINE_SAMPLE_RESULT_PATH "_adv.yuv", dst);
				CameraCvSampleMemoryFree(dst);
			}
		}
	}
	
	return result;
}

static double CameraCvSampleDeterminant(double mat[MAX_DIM][MAX_DIM], int dim)
{
	double submat[MAX_DIM][MAX_DIM];
	double det = 0;
	if(dim == 3)
	{
		det = mat[0][0] * mat[1][1] * mat[2][2] 
			+ mat[0][1] * mat[1][2] * mat[2][0] 
			+ mat[0][2] * mat[1][0] * mat[2][1] 
			- mat[0][0] * mat[1][2] * mat[2][1] 
			- mat[0][1] * mat[1][0] * mat[2][2] 
			- mat[0][2] * mat[1][1] * mat[2][0]; 
		return det;
	}
	
	for(int i=0;i<dim;i++)
	{
		int j2 = 0;
		for(int j=0;j<dim;j++)
		{
			if(j == i) continue;
			
			for(int k=1;k<dim;k++)
			{
				submat[j2][k-1]=mat[j][k];
			}
			j2++;
		}
		
		int coff = (i & 1) ? -1 : 1;
		det += mat[i][0] * coff * CameraCvSampleDeterminant(submat, dim-1);
	}
	return det;
}

static void CamerCvSampleInverseHalf(double src[MAX_DIM][MAX_DIM], double dst[MAX_DIM][HALF_DIM])
{
	double det_inv = 1.0 / CameraCvSampleDeterminant(src, MAX_DIM);
	double submat[MAX_DIM][MAX_DIM];
	
	
	int dim = MAX_DIM;
	
	for(int i=0;i<HALF_DIM;i++)
	{
		
		for(int j=0;j<dim;j++)
		{
			int i2=0;
			for(int m=0;m<dim;m++)
			{
				if(m==i) continue;
				int j2=0;
				for(int n=0;n<dim;n++)
				{
					if(n==j) continue;
					submat[i2][j2] = src[m][n];
					
					j2++;
				}
				
				i2++;
			}
			int coff = ((i+j)&1)?-1:1;
			
			dst[j][i] = CameraCvSampleDeterminant(submat, dim-1) * det_inv * coff;
		}
		
	}

}

static void CameraCvSampleGetInveseTransform(Point p[4], float transfrom[3][3])
{
	double &X0 = p[0].X;
	double &X1 = p[1].X;
	double &X2 = p[2].X;
	double &X3 = p[3].X;
	
	double &Y0 = p[0].Y;
	double &Y1 = p[1].Y;
	double &Y2 = p[2].Y;
	double &Y3 = p[3].Y;

	double mat[MAX_DIM][MAX_DIM]={
		{X1, Y1, 1, 0, 0, 0, -X1, -Y1},
		{0, 0, 0, X2, Y2, 1, -X2, -Y2},
		{X3, Y3, 1, 0, 0, 0, -X3, -Y3},
		{0, 0, 0, X3, Y3, 1, -X3, -Y3},
		{X0, Y0, 1, 0, 0, 0, 0, 0},
		{0, 0, 0, X0, Y0, 1, 0, 0},
		{0, 0, 0, X1, Y1, 1, 0, 0},
		{X2, Y2, 1, 0, 0, 0, 0, 0}
	};
	
	double inv_mat[MAX_DIM][HALF_DIM];
	
	CamerCvSampleInverseHalf(mat, inv_mat);
	
	float *pTranform = (float*)&transfrom[0][0];
	
	for(int i=0;i<MAX_DIM;i++)
	{
		double sum = 0;
		for(int j=0;j<HALF_DIM;j++)
		{
			sum += inv_mat[i][j];
		}
		*pTranform++ = sum;
	}
	*pTranform = 1.0f;
}

static void CameraCvSampleWarpPerspeciveTransfromation(Point &src, Point &dst, float tranform[3][3])
{
	double ratio = 1.0 / (tranform[2][0] * src.X + tranform[2][1] * src.Y + tranform[2][2]);
	dst.X = (tranform[0][0] * src.X + tranform[0][1] * src.Y + tranform[0][2]) * ratio;
	dst.Y = (tranform[1][0] * src.X + tranform[1][1] * src.Y + tranform[1][2]) * ratio;
}

static void ConvertToInversePerspectiveTransfrom(float tranform[3][3], float inverse_tranform[3][3])
{
	Point p[4];
	Point input;
	
	//left top
	input.X = 0;
	input.Y = 0;
	CameraCvSampleWarpPerspeciveTransfromation(input, p[0], tranform);
	
	//right top
	input.X = 1;
	input.Y = 0;
	CameraCvSampleWarpPerspeciveTransfromation(input, p[1], tranform);

	//left down
	input.X = 0;
	input.Y = 1;
	CameraCvSampleWarpPerspeciveTransfromation(input, p[2], tranform);	
	
	//right down
	input.X = 1;
	input.Y = 1;
	CameraCvSampleWarpPerspeciveTransfromation(input, p[3], tranform);	
	
	CameraCvSampleGetInveseTransform(p, inverse_tranform);
}

static void CameraCvSampleGetPerspectiveTransform(PointF p[4], float W, float H, float transform[3][3])
{
	float &a = transform[0][0];
	float &b = transform[0][1];
	float &c = transform[0][2];
	float &d = transform[1][0];
	float &e = transform[1][1];
	float &f = transform[1][2];
	float &g = transform[2][0];
	float &h = transform[2][1];
	float &i = transform[2][2];
	
	i = 1.0;
			
	g = DET(
		p[1].X + p[2].X - p[0].X - p[3].X,
		p[3].X - p[2].X,
		p[1].Y + p[2].Y - p[0].Y - p[3].Y,
		p[3].Y - p[2].Y
	) / W /
	DET(
		p[3].X - p[1].X,
		p[3].X - p[2].X,
		p[3].Y - p[1].Y,
		p[3].Y - p[2].Y
	);
	
	h = DET(
		p[3].X - p[1].X,
		p[1].X + p[2].X - p[0].X - p[3].X,
		p[3].Y - p[1].Y,
		p[1].Y + p[2].Y - p[0].Y - p[3].Y
	) / H /
	DET(
		p[3].X - p[1].X,
		p[3].X - p[2].X,
		p[3].Y - p[1].Y,
		p[3].Y - p[2].Y
	);
	
	a = (p[1].X - p[0].X) / W + g * p[1].X;
	b = (p[2].X - p[0].X) / H + h * p[2].X;
	c = p[0].X;
	
	d = (p[1].Y - p[0].Y) / W + g * p[1].Y;
	e = (p[2].Y - p[0].Y) / H + h * p[2].Y;
	f = p[0].Y;
	
}

int CameraCvSampleWarpPerspective(int TestSampleId)
{
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_MAT_ST dst;	
	CAMERA_CV_API_MAT_ST transform;
	CAMERA_CV_API_SIZE_ST dsize;
	const int WARP_PERSPECTIVE_SAMPLE_IMG_DST_WIDTH = 400;
	const int WARP_PERSPECTIVE_SAMPLE_IMG_DST_HEIGHT = 240;

	typedef struct{
		const char* SamplePath;
		const char* ResultPath;
		int SampleWidth;
		int SampleHeight;
		PointF corner[4];
	}SampleInformationStruct;

	SampleInformationStruct sampleInfo;

	float PespectiveTransformation[3][3] = {
			{1,0,0},
			{0,1,0},
			{0,0,1}
	};

	switch(TestSampleId)
	{
		case 0:
			sampleInfo.SamplePath = "/mnt/sd/TestData/input/car/9R8027.yuv";
			sampleInfo.ResultPath = "/mnt/sd/TestData/output/car/BL_240_400_400x240_9R8027_license_plate.yuv";
			sampleInfo.SampleWidth = 500;
			sampleInfo.SampleHeight = 332;
			sampleInfo.corner[0].X = 162;
			sampleInfo.corner[0].Y = 126;
			sampleInfo.corner[1].X = 357;
			sampleInfo.corner[1].Y = 146;
			sampleInfo.corner[2].X = 154;
			sampleInfo.corner[2].Y = 222;
			sampleInfo.corner[3].X = 348;
			sampleInfo.corner[3].Y = 250;
			break;
			

		case 1:
			sampleInfo.SamplePath = "/mnt/sd/TestData/input/car/0502DA.yuv";
			sampleInfo.ResultPath = "/mnt/sd/TestData/output/car/BL_240_400_400x240_0502DA_license_plate.yuv";
			sampleInfo.SampleWidth = 438;
			sampleInfo.SampleHeight = 686;
			sampleInfo.corner[0].X = 32;
			sampleInfo.corner[0].Y = 250;
			sampleInfo.corner[1].X = 421;
			sampleInfo.corner[1].Y = 251;
			sampleInfo.corner[2].X = 20;
			sampleInfo.corner[2].Y = 431;
			sampleInfo.corner[3].X = 400;
			sampleInfo.corner[3].Y = 428;
			break;

		case 2:
			sampleInfo.SamplePath = "/mnt/sd/TestData/input/car/AAA6385.yuv";
			sampleInfo.ResultPath = "/mnt/sd/TestData/output/car/BL_240_400_400x240_AAA6385_license_plate.yuv";
			sampleInfo.SampleWidth = 600;
			sampleInfo.SampleHeight = 386;
			sampleInfo.corner[0].X = 31;
			sampleInfo.corner[0].Y = 114;
			sampleInfo.corner[1].X = 515;
			sampleInfo.corner[1].Y = 12;
			sampleInfo.corner[2].X = 101;
			sampleInfo.corner[2].Y = 381;
			sampleInfo.corner[3].X = 543;
			sampleInfo.corner[3].Y = 209;
			break;
		default:
			printf("unknown sample id\n");
			return -1;

	}

	if(USE_INVERSE_TRANSFORMATION) {
		//before test procedure, do the follow steps
		//1. put input data in directory of SD card TestData/input/car
		//2. create directory TestData/output/car in SD card
		CameraCvSampleGetPerspectiveTransform(sampleInfo.corner, WARP_PERSPECTIVE_SAMPLE_IMG_DST_WIDTH, WARP_PERSPECTIVE_SAMPLE_IMG_DST_HEIGHT, PespectiveTransformation);
	}
	else {
		
		float OrgPespectiveTransformation[3][3] = {
			{-6.03294, -5.02745, 2010.98},
			{2.54451, -13.3587, 2557.23},
			{0.00606037, -0.0206395, 1}
		};
		ConvertToInversePerspectiveTransfrom(OrgPespectiveTransformation, PespectiveTransformation);

	}
	int size = sampleInfo.SampleWidth * sampleInfo.SampleHeight;
	uint8_t *SampleBuffer = new uint8_t[size];
	CameraCvSampleImread(sampleInfo.SamplePath, SampleBuffer, size);

	CameraCvSampleInitMat(src, SampleBuffer, sampleInfo.SampleWidth, sampleInfo.SampleHeight, CAMERA_CV_DATA_TYPE_8UC1);
	CameraCvSampleInitMat(transform, PespectiveTransformation, 3, 3, CAMERA_CV_DATA_TYPE_32FC1);	
	dsize.i_width = WARP_PERSPECTIVE_SAMPLE_IMG_DST_WIDTH;
	dsize.i_height = WARP_PERSPECTIVE_SAMPLE_IMG_DST_HEIGHT;

	int start = CameraCvSamplePerfGetCurrent();
	int32_t result = camera_cv_api_warp_perspective(&src, &dst, &transform, &dsize);
	int end = CameraCvSamplePerfGetCurrent();

	if(!result)
	{
		printf("Diff = %d\n", end - start);
		CameraCvSampleImwrite(sampleInfo.ResultPath, dst);
		CameraCvSampleMemoryFree(dst);
	}

	delete [] SampleBuffer;
	
	return result;
}

int CameraCvSampleResize(void)
{
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_MAT_ST dst;	
	CAMERA_CV_API_SIZE_ST dsize;
	CameraCvSampleInitMat(src, g_SampleBuffer, RESIZE_SAMPLE_IMG_SRC_WIDTH, RESIZE_SAMPLE_IMG_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_8UC1);
	dsize.i_width = RESIZE_SAMPLE_IMG_DST_WIDTH;
	dsize.i_height = RESIZE_SAMPLE_IMG_DST_HEIGHT;
	int32_t result = camera_cv_api_resize(&src, &dst, &dsize);
	
	printf("Resize Result = %d\n", result);	

	if(!result)
	{
		CameraCvSampleImwrite(RESIZE_SAMPLE_RESULT_PATH, dst);
		CameraCvSampleMemoryFree(dst);
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_ROI_ST rect;
			CAMERA_CV_API_MAT_ST subSrc;
			
			rect.i_x = RESIZE_SAMPLE_IMG_SRC_WIDTH / 3;
			rect.i_y = RESIZE_SAMPLE_IMG_SRC_HEIGHT / 3;
			rect.i_width = RESIZE_SAMPLE_IMG_SRC_WIDTH / 2;
			rect.i_height = RESIZE_SAMPLE_IMG_SRC_HEIGHT / 2;
			
			subSrc = src & rect; //region mask
			
			dsize.i_width = RESIZE_SAMPLE_IMG_DST_WIDTH / 2;
			dsize.i_height = RESIZE_SAMPLE_IMG_DST_HEIGHT / 2;

			result = camera_cv_api_resize(&subSrc, &dst, &dsize);
			
			if(!result)
			{
				CameraCvSampleImwrite(RESIZE_SAMPLE_RESULT_PATH "_adv.yuv", dst);
				CameraCvSampleMemoryFree(dst);
			}
		}
	}
	return result;

}

int CameraCvSampleFeatureDetect(std::string test)
{
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_KEY_POINT_VECTOR_ST key_pts;	
	
	CAMERA_CV_API_FEATURE_2D_ST* inst;
	if(test == "FAST")
	{
		int threshold = 100;
		inst = camera_cv_api_fast_create(threshold, (int)true);
	}
	else if(test == "GFTT")
	{
		int maxCorner = 100;
		double quality = 0.05;
		double minDistance = 30;
		double harrisK = 0.07;
		inst = camera_cv_api_gftt_create(maxCorner, quality, minDistance, harrisK);
	}
	else if(test == "ORB")
	{
		int nFeatures = 100;
		int scoreType = CAMERA_CV_API_ORB_FAST_SCORE;
		int fastThreshold = 100;
		inst = camera_cv_api_orb_create(nFeatures, scoreType, fastThreshold);
	}
	else
	{
		printf("Unknown test %s\n", test.c_str());
		return -1;
	}
	
	CameraCvSampleInitMat(src, g_SampleBuffer, FEATURE_DETECT_SAMPLE_IMG_SRC_WIDTH, FEATURE_DETECT_SAMPLE_IMG_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_8UC1);
	
	int32_t result = camera_cv_api_feature_detect(inst, &src, &key_pts);
	
	if(!result)
	{
		CameraCvSampleShowKeyPoint(key_pts, (test + " result").c_str());
		CameraCvSampleMemoryFree(key_pts);
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_ROI_ST rect;
			CAMERA_CV_API_MAT_ST subSrc;
			
			rect.i_x = 400;
			rect.i_y = 700;
			rect.i_width = 480;
			rect.i_height = 320;
			
			subSrc = src & rect; //region mask
			
			result = camera_cv_api_feature_detect(inst, &subSrc, &key_pts);
			
			if(!result)
			{
				CameraCvSampleShowKeyPoint(key_pts, (test + " advance test result").c_str());
				CameraCvSampleMemoryFree(key_pts);
			}
		}
	}
	
	camera_cv_api_feature_destroy(&inst);
	
	return result;
}

int CameraCvSampleFeatureDetectAndCompute(void)
{
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_MAT_ST descriptor;
	CAMERA_CV_API_KEY_POINT_VECTOR_ST key_pts;	
	
	CAMERA_CV_API_FEATURE_2D_ST* inst;

	int nFeatures = 1024;
	int scoreType = CAMERA_CV_API_ORB_FAST_SCORE;
	int fastThreshold = 75;
	inst = camera_cv_api_orb_create(nFeatures, scoreType, fastThreshold);
	
	CameraCvSampleInitMat(src, g_SampleBuffer, FEATURE_DETECT_SAMPLE_IMG_SRC_WIDTH, FEATURE_DETECT_SAMPLE_IMG_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_8UC1);
	
	int32_t result = camera_cv_api_feature_detect_and_compute(inst, &src, &key_pts, &descriptor);
	
	if(!result)
	{
		CameraCvSampleShowKeyPoint(key_pts, "ORB keypoint");
		CameraCvSampleMemoryFree(key_pts);
		
		CameraCvSampleShowMat(descriptor, "ORB descriptor");
		CameraCvSampleMemoryFree(descriptor);
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_ROI_ST rect;
			CAMERA_CV_API_MAT_ST subSrc;
			
			rect.i_x = 400;
			rect.i_y = 700;
			rect.i_width = 480;
			rect.i_height = 320;
			
			subSrc = src & rect; //region mask
			
			result = camera_cv_api_feature_detect_and_compute(inst, &subSrc, &key_pts, &descriptor);
			
			if(!result)
			{
				CameraCvSampleShowKeyPoint(key_pts, "ORB advance test keypoint");
				CameraCvSampleMemoryFree(key_pts);

				CameraCvSampleShowMat(descriptor, "ORB advance test descriptor");
				CameraCvSampleMemoryFree(descriptor);
			}
		}
	}
	
	camera_cv_api_feature_destroy(&inst);
	
	return result;
}

int CameraCvSampleBFMatcher(void)
{
	CAMERA_CV_API_MAT_ST trainMat, queryMat;
	CAMERA_CV_API_MAT_ST trainDesc, queryDesc;
	CAMERA_CV_API_MATCH_VECTOR_ST matches;
	
	CAMERA_CV_API_MAT_ST trans;
	CAMERA_CV_API_SIZE_ST dsize;
	
	CAMERA_CV_API_KEY_POINT_VECTOR_ST queryKeyPoints, trainKeyPoints;
	std::vector<CAMERA_CV_API_MAT_ST*> needToFreeMat;
	std::vector<CAMERA_CV_API_KEY_POINT_VECTOR_ST*> needToFreeKeyPoint;
	
	int32_t result = 0;

	//parameters setting
	int width = 1920;
	int height = 1080;
	const int L = 304;
	const float L_inv = 1.0f / L;
	const float radius = L * 0.707f;
	const float x0 = 440.0f;
	const float y0 = 735.0f;
	const float x1 = x0 + radius * -0.8191f;
	const float y1 = y0 - radius * 0.5735f;
	const float x2 = x0 + radius * 0.5735f;
	const float y2 = y0 - radius * 0.8191f;
	const float x3 = x0 + radius * -0.5735f;
	const float y3 = y0 - radius * -0.8191f;
	
	float transform[2][3] = {0};
	
	transform[0][0] = (x2 - x1) * L_inv;
	transform[0][1] = (x3 - x1) * L_inv;
	transform[0][2] = x1;
	transform[1][0] = (y2 - y1) * L_inv;
	transform[1][1] = (y3 - y1) * L_inv;
	transform[1][2] = y1;

	
	//ORB
	CAMERA_CV_API_FEATURE_2D_ST* inst;
	int nFeatures = 1024;
	int scoreType = CAMERA_CV_API_ORB_FAST_SCORE;
	int fastThreshold = 75;
	inst = camera_cv_api_orb_create(nFeatures, scoreType, fastThreshold);
	//init mat
	CameraCvSampleInitMat(trainMat, g_SampleBuffer, width, height, CAMERA_CV_DATA_TYPE_8UC1);
	CameraCvSampleInitMat(trans, transform, 3, 2, CAMERA_CV_DATA_TYPE_32FC1);	
	
	//generate queryMat
	dsize.i_width = L;
	dsize.i_height = L;
	result = camera_cv_api_warp_affine(&trainMat, &queryMat, &trans, &dsize);
	
	if(result)
	{
		printf("warp affine error\n");
		goto cv_api_fail;
	}
	else
	{
		printf("success warp\n");
		needToFreeMat.push_back(&queryMat);
	}
	
	//get query ORB descriptor
	result = camera_cv_api_feature_detect_and_compute(inst, &queryMat, &queryKeyPoints, &queryDesc);

	if(result)
	{
		printf("Query Mat ORB fail\n");
		goto cv_api_fail;
	}
	else
	{
		printf("Query Mat ORB success\n");
		needToFreeMat.push_back(&queryDesc);
		needToFreeKeyPoint.push_back(&queryKeyPoints);
	}
	
	//get train ORB descriptor
	result = camera_cv_api_feature_detect_and_compute(inst, &trainMat, &trainKeyPoints, &trainDesc);

	if(result)
	{
		printf("Train Mat ORB fail\n");
		goto cv_api_fail;
	}
	else
	{
		printf("Train Mat ORB success\n");
		needToFreeMat.push_back(&trainDesc);
		needToFreeKeyPoint.push_back(&trainKeyPoints);
	}
	
	//BF matcher
	result = camera_cv_api_bf_matcher_match(&queryDesc, &trainDesc, &matches);

	if(result)
	{
		printf("Match fail\n");
		goto cv_api_fail;
	}
	else
	{
		printf("Match success\n");
		
		if(g_CheckMatEnable)
		{
			for(int idx = 0;idx < matches.count;idx++)
			{
				int qIdx = matches.ps_matches[idx].i_query_idx;
				int tIdx = matches.ps_matches[idx].i_train_idx;
				
				CAMERA_CV_API_KEY_POINT_ST* qKeyPts = &queryKeyPoints.ps_key_point[qIdx];
				CAMERA_CV_API_KEY_POINT_ST* tKeyPts = &trainKeyPoints.ps_key_point[tIdx];
				printf("[%d] (%f, %f) <-> (%f, %f)\n", idx, qKeyPts->f_x, qKeyPts->f_y, tKeyPts->f_x, tKeyPts->f_y);
			}
		}
		CameraCvSampleMemoryFree(matches);
	}
	
	printf("All function success\n");
	
cv_api_fail:
	for(auto &item:needToFreeMat)
	{
		CameraCvSampleMemoryFree(*item);
	}

	for(auto &item:needToFreeKeyPoint)
	{
		CameraCvSampleMemoryFree(*item);
	}
	
	camera_cv_api_feature_destroy(&inst);
	
	return result;
	
}

int CameraCvSampleBFMatcherRadiusMatch(void)
{
	CAMERA_CV_API_MAT_ST trainMat, queryMat;
	CAMERA_CV_API_MAT_ST trainDesc, queryDesc;
	CAMERA_CV_API_MATCH_VECTOR_ST matches;
	
	CAMERA_CV_API_MAT_ST trans;
	CAMERA_CV_API_SIZE_ST dsize;
	
	CAMERA_CV_API_KEY_POINT_VECTOR_ST queryKeyPoints, trainKeyPoints;
	std::vector<CAMERA_CV_API_MAT_ST*> needToFreeMat;
	std::vector<CAMERA_CV_API_KEY_POINT_VECTOR_ST*> needToFreeKeyPoint;
	
	int32_t result = 0;

	//parameters setting
	int width = 1920;
	int height = 1080;
	const int L = 304;
	const float L_inv = 1.0f / L;
	const float radius = L * 0.707f;
	const float x0 = 440.0f;
	const float y0 = 735.0f;
	const float x1 = x0 + radius * -0.8191f;
	const float y1 = y0 - radius * 0.5735f;
	const float x2 = x0 + radius * 0.5735f;
	const float y2 = y0 - radius * 0.8191f;
	const float x3 = x0 + radius * -0.5735f;
	const float y3 = y0 - radius * -0.8191f;
	
	float transform[2][3] = {0};
	
	transform[0][0] = (x2 - x1) * L_inv;
	transform[0][1] = (x3 - x1) * L_inv;
	transform[0][2] = x1;
	transform[1][0] = (y2 - y1) * L_inv;
	transform[1][1] = (y3 - y1) * L_inv;
	transform[1][2] = y1;

	
	//ORB
	CAMERA_CV_API_FEATURE_2D_ST* inst;
	int nFeatures = 1024;
	int scoreType = CAMERA_CV_API_ORB_FAST_SCORE;
	int fastThreshold = 75;
	inst = camera_cv_api_orb_create(nFeatures, scoreType, fastThreshold);
	//init mat
	CameraCvSampleInitMat(trainMat, g_SampleBuffer, width, height, CAMERA_CV_DATA_TYPE_8UC1);
	CameraCvSampleInitMat(trans, transform, 3, 2, CAMERA_CV_DATA_TYPE_32FC1);	
	
	//generate queryMat
	dsize.i_width = L;
	dsize.i_height = L;
	result = camera_cv_api_warp_affine(&trainMat, &queryMat, &trans, &dsize);
	
	if(result)
	{
		printf("warp affine error\n");
		goto cv_api_fail;
	}
	else
	{
		printf("success warp\n");
		needToFreeMat.push_back(&queryMat);
	}
	
	//get query ORB descriptor
	result = camera_cv_api_feature_detect_and_compute(inst, &queryMat, &queryKeyPoints, &queryDesc);

	if(result)
	{
		printf("Query Mat ORB fail\n");
		goto cv_api_fail;
	}
	else
	{
		printf("Query Mat ORB success\n");
		needToFreeMat.push_back(&queryDesc);
		needToFreeKeyPoint.push_back(&queryKeyPoints);
	}
	
	//get train ORB descriptor
	result = camera_cv_api_feature_detect_and_compute(inst, &trainMat, &trainKeyPoints, &trainDesc);

	if(result)
	{
		printf("Train Mat ORB fail\n");
		goto cv_api_fail;
	}
	else
	{
		printf("Train Mat ORB success\n");
		needToFreeMat.push_back(&trainDesc);
		needToFreeKeyPoint.push_back(&trainKeyPoints);
	}
	
	//BF matcher
	result = camera_cv_api_bf_matcher_radius_match(&queryDesc, &trainDesc, &matches, 90, 0.5); 
	//min distance = 90 <- filter out 256 bit hamming distance greater than this value
	//ratio = 0.5      <- filter out which best distance / better distance > ratio

	if(result)
	{
		printf("Match fail\n");
		goto cv_api_fail;
	}
	else
	{
		printf("Match success\n");
		
		if(g_CheckMatEnable)
		{
			for(int idx = 0;idx < matches.count;idx++)
			{
				int qIdx = matches.ps_matches[idx].i_query_idx;
				int tIdx = matches.ps_matches[idx].i_train_idx;
				
				CAMERA_CV_API_KEY_POINT_ST* qKeyPts = &queryKeyPoints.ps_key_point[qIdx];
				CAMERA_CV_API_KEY_POINT_ST* tKeyPts = &trainKeyPoints.ps_key_point[tIdx];
				printf("[%d] (%f, %f) <-> (%f, %f)\n", idx, qKeyPts->f_x, qKeyPts->f_y, tKeyPts->f_x, tKeyPts->f_y);
			}
		}
		CameraCvSampleMemoryFree(matches);
	}
	
	printf("All function success\n");
	
cv_api_fail:
	for(auto &item:needToFreeMat)
	{
		CameraCvSampleMemoryFree(*item);
	}

	for(auto &item:needToFreeKeyPoint)
	{
		CameraCvSampleMemoryFree(*item);
	}
	
	camera_cv_api_feature_destroy(&inst);
	
	return result;
	
}

int CamerCvSampleCalcOpticalFlowPyrLk(void)
{
	CAMERA_CV_API_MAT_ST prevMat, nextMat;
	CAMERA_CV_API_KEY_POINT_VECTOR_ST prevKeyPoints, nextKeyPoints;
	
	CAMERA_CV_API_MAT_ST trans;
	CAMERA_CV_API_SIZE_ST dsize;
	CAMERA_CV_API_SIZE_ST winSize;
	
	std::vector<CAMERA_CV_API_MAT_ST*> needToFreeMat;
	std::vector<CAMERA_CV_API_KEY_POINT_VECTOR_ST*> needToFreeKeyPoint;
	uint8_t* status = NULL;
	
	int32_t result = 0;
	//FAST
	CAMERA_CV_API_FEATURE_2D_ST* inst;
	int threshold = 100;
	inst = camera_cv_api_fast_create(threshold, (int)true);

	//parameters setting
	const int width = 1920;
	const int height = 1080;
	const float x1 = width / 10;
	const float y1 = height / 10;
	const float x2 = width - x1;
	const float y2 = y1;
	const float x3 = x1;
	const float y3 = height - y1;
	
	float transform[2][3] = {0};
	
	transform[0][0] = (x2 - x1) / width;
	transform[0][1] = (x3 - x1) / height;
	transform[0][2] = x1;
	transform[1][0] = (y2 - y1) / width;
	transform[1][1] = (y3 - y1) / height;
	transform[1][2] = y1;

	//init mat
	CameraCvSampleInitMat(prevMat, g_SampleBuffer, width, height, CAMERA_CV_DATA_TYPE_8UC1);
	CameraCvSampleInitMat(trans, transform, 3, 2, CAMERA_CV_DATA_TYPE_32FC1);	
	
	//generate nextMat
	dsize.i_width = width;
	dsize.i_height = height;
	result = camera_cv_api_warp_affine(&prevMat, &nextMat, &trans, &dsize);
	
	if(result)
	{
		printf("warp affine error\n");
		goto cv_api_fail;
	}
	else
	{
		printf("success warp\n");
		needToFreeMat.push_back(&nextMat);
	}
	
	//get FAST result of prevMat
	result = camera_cv_api_feature_detect(inst, &prevMat, &prevKeyPoints);

	if(result)
	{
		printf("FAST fail\n");
		goto cv_api_fail;
	}
	else
	{
		printf("FAST success\n");
		needToFreeKeyPoint.push_back(&prevKeyPoints);
	}
	
	//CalcOpticalFlowPyrLk
	winSize.i_width = 16;
	winSize.i_height = 16;
	
	result = camera_cv_api_calc_optical_flow_pyr_lk(&prevMat, &nextMat, &prevKeyPoints, &nextKeyPoints, &status, &winSize, 3);

	if(result)
	{
		printf("CalcOpticalFlowPyrLk fail\n");
		goto cv_api_fail;
	}
	else
	{
		printf("Tracking success\n");
		
		CameraCvSampleShowKeyPoint(prevKeyPoints, "Previous keypoint");
		CameraCvSampleShowKeyPoint(nextKeyPoints, "Next keypoint", status);
		
		needToFreeKeyPoint.push_back(&nextKeyPoints);
		camera_cv_api_mem_free((void**)&status);
	}
	
	printf("All function success\n");
	
cv_api_fail:
	for(auto &item:needToFreeMat)
	{
		CameraCvSampleMemoryFree(*item);
	}

	for(auto &item:needToFreeKeyPoint)
	{
		CameraCvSampleMemoryFree(*item);
	}
	
	camera_cv_api_feature_destroy(&inst);
	
	return result;
}

int CameraCvSampleCvtColor(void)
{
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_MAT_ST dst;	
	CameraCvSampleInitMat(src, g_SampleBuffer, CVTCOLOR_SAMPLE_IMG_SRC_WIDTH, CVTCOLOR_SAMPLE_IMG_SRC_HEIGHT);
	int32_t result = camera_cv_api_cvt_color(&src, &dst, CAMERA_CV_CONVERT_NV12_2_RGB888Packed);

	printf("Convert Result = %d\n", result);	

	if(!result)
	{
		if(g_AdvanceTest)
		{
			//NV12 => full frame convert RGB => a part of frame convert gray
			CAMERA_CV_API_ROI_ST roi;
			CAMERA_CV_API_MAT_ST grayDst;
			CAMERA_CV_API_CONVERT_CODE_ET e_code = CAMERA_CV_CONVERT_RGB888Packed_2_GRAY;
			
			roi.i_x = CVTCOLOR_SAMPLE_IMG_SRC_WIDTH / 3;
			roi.i_y = CVTCOLOR_SAMPLE_IMG_SRC_HEIGHT / 3;
			roi.i_width = CVTCOLOR_SAMPLE_IMG_SRC_WIDTH / 2;
			roi.i_height = CVTCOLOR_SAMPLE_IMG_SRC_HEIGHT / 2;
			
			result = camera_cv_api_sub_img_cvt_color(&dst, &grayDst, &e_code, &roi, 1);
			
			printf("Convert roi result = %d\n", result);
			
			if(!result)
			{
				CameraCvSampleImwrite(CVTCOLOR_SAMPLE_RESULT_PATH "_gray.yuv", grayDst);
				CameraCvSampleMemoryFree(grayDst);
			}
		}
		
		CameraCvSampleImwrite(CVTCOLOR_SAMPLE_RESULT_PATH "_rgb.rgb", dst);
		CameraCvSampleMemoryFree(dst);
	}
	return result;
}

int CameraCvSampleMerge(void)
{
	CAMERA_CV_API_MAT_ST src[4];
	CAMERA_CV_API_MAT_ST dst;	
	const int MERGE_SAMPLE_WIDTH   = 3;
	const int MERGE_SAMPLE_HEIGHT  = 5;	
	for(int i=0;i<4;i++)
	{
		CameraCvSampleInitMat(src[i], new uint8_t[MERGE_SAMPLE_WIDTH * MERGE_SAMPLE_HEIGHT], MERGE_SAMPLE_WIDTH, MERGE_SAMPLE_HEIGHT);
		memset(src[i].pv_data, 20 * i, MERGE_SAMPLE_WIDTH * MERGE_SAMPLE_HEIGHT);
		uint8_t *p = (uint8_t*)src[i].pv_data;
		for(int j=0; j < MERGE_SAMPLE_WIDTH * MERGE_SAMPLE_HEIGHT;j++)
			p[j] += j;
	}

	int32_t result = camera_cv_api_merge(src, 4, &dst);

	if(!result)
	{
		CameraCvSampleShowMat(dst, "Result After merge four buffer");
		CameraCvSampleMemoryFree(dst);
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_MAT_ST subSrc[4];
			
			for(int i=0;i<4;i++)
			{
				char label[64];
				CAMERA_CV_API_ROI_ST rect;
				snprintf(label, sizeof(label), "Advance test src %d", i);
				rect.i_x = i % (MERGE_SAMPLE_WIDTH - 1);
				rect.i_y = i % (MERGE_SAMPLE_HEIGHT - 1);
				rect.i_width = 2;
				rect.i_height = 2;
				
				subSrc[i] = src[i] & rect; //region mask
				CameraCvSampleShowMat(subSrc[i], label);
			}
			
			result = camera_cv_api_merge(subSrc, 4, &dst);
			
			if(!result)
			{
				CameraCvSampleShowMat(dst, "Advance test");
				CameraCvSampleMemoryFree(dst);
			}
		}
	}

	for(int i=0;i<4;i++)
		delete [] (uint8_t*)src[i].pv_data;

	return result;
}

int CameraCvSampleSplit(void)
{
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_MAT_ST dsts[4];	
	const int SPLIT_CHANNEL = 4;
	const int SPLIT_SAMPLE_WIDTH   = 3;
	const int SPLIT_SAMPLE_HEIGHT  = 5;	
	
	uint8_t *InputData = new uint8_t[SPLIT_SAMPLE_WIDTH * SPLIT_CHANNEL * SPLIT_SAMPLE_HEIGHT];
	CameraCvSampleInitMat(src, InputData, SPLIT_SAMPLE_WIDTH, SPLIT_SAMPLE_HEIGHT, CAMERA_CV_DATA_TYPE_8UC4);
	
	for(int i=0;i<SPLIT_SAMPLE_WIDTH * SPLIT_CHANNEL * SPLIT_SAMPLE_HEIGHT;i++)
	{
		InputData[i] = i + 1;
	}
	
	CameraCvSampleShowMat(src, "Split input buffer");

	int32_t result = camera_cv_api_split(&src, dsts);

	if(!result)
	{
		for(CAMERA_CV_API_MAT_ST &dst: dsts)
		{
			CameraCvSampleShowMat(dst, "Result After split buffer");
			CameraCvSampleMemoryFree(dst);
		}
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_MAT_ST subSrc;
			
			CAMERA_CV_API_ROI_ST rect;
		
			rect.i_x = 1;
			rect.i_y = 1;
			rect.i_width = 2;
			rect.i_height = 2;
			
			subSrc = src & rect; //region mask
			CameraCvSampleShowMat(subSrc, "Advance test src");
			
			result = camera_cv_api_split(&subSrc, dsts);
			
			if(!result)
			{
				for(CAMERA_CV_API_MAT_ST &dst: dsts)
				{
					CameraCvSampleShowMat(dst, "Advance test");
					CameraCvSampleMemoryFree(dst);
				}
			}
		}
	}

	delete [] InputData;

	return result;
}

int CameraCvSampleVConcat(void)
{
	CAMERA_CV_API_MAT_ST src, src_lu, src_rd;
	CAMERA_CV_API_MAT_ST dst;
	CAMERA_CV_API_ROI_ST rect;
	
	CameraCvSampleInitMat(src, g_SampleBuffer, VCONCAT_SAMPLE_IMG_SRC_WIDTH, VCONCAT_SAMPLE_IMG_SRC_HEIGHT);
	
	//LU
	rect.i_x = 0;
	rect.i_y = 0;
	rect.i_width = 640;
	rect.i_height = 480;
	
	src_lu = src & rect; //region mask	

	//RD
	rect.i_x = 1280;
	rect.i_y = 600;
	rect.i_width = 640;
	rect.i_height = 480;
	
	src_rd = src & rect; //region mask	
	
	int32_t result = camera_cv_api_vconcat(&src_lu, &src_rd, &dst);

	if(!result)
	{	
		CameraCvSampleImwrite(VCONCAT_SAMPLE_RESULT_PATH, dst); // 640 * 960 PP400
		CameraCvSampleMemoryFree(dst);
	}
	return result;
}

int CameraCvSampleHConcat(void)
{
	CAMERA_CV_API_MAT_ST src, src_lu, src_rd;
	CAMERA_CV_API_MAT_ST dst;
	CAMERA_CV_API_ROI_ST rect;
	
	CameraCvSampleInitMat(src, g_SampleBuffer, HCONCAT_SAMPLE_IMG_SRC_WIDTH, HCONCAT_SAMPLE_IMG_SRC_HEIGHT);
	
	//LU
	rect.i_x = 0;
	rect.i_y = 0;
	rect.i_width = 640;
	rect.i_height = 480;
	
	src_lu = src & rect; //region mask	

	//RD
	rect.i_x = 1280;
	rect.i_y = 600;
	rect.i_width = 640;
	rect.i_height = 480;
	
	src_rd = src & rect; //region mask	
	
	int32_t result = camera_cv_api_hconcat(&src_lu, &src_rd, &dst);

	if(!result)
	{	
		CameraCvSampleImwrite(HCONCAT_SAMPLE_RESULT_PATH, dst); // 1280 * 480 PP400
		CameraCvSampleMemoryFree(dst);
	}
	return result;
}

int CameraCvSampleConvertTo(void)
{
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_MAT_ST dst;	
	const int CONVERT_TO_SAMPLE_WIDTH   = 5;
	const int CONVERT_TO_SAMPLE_HEIGHT  = 4;	
	uint8_t convert_to_sample_buffer[CONVERT_TO_SAMPLE_HEIGHT][CONVERT_TO_SAMPLE_WIDTH]={0};

	for(int y=0;y<CONVERT_TO_SAMPLE_HEIGHT;y++)
	{
		for(int x=0;x<CONVERT_TO_SAMPLE_WIDTH;x++)
		{
			convert_to_sample_buffer[y][x] = x + y * 2;
		}
	}

	CameraCvSampleInitMat(src, (void*)convert_to_sample_buffer, CONVERT_TO_SAMPLE_WIDTH, CONVERT_TO_SAMPLE_HEIGHT, CAMERA_CV_DATA_TYPE_8UC1);

	CameraCvSampleShowMat(src, "Src Data");

	int32_t result = camera_cv_api_convert_to(&src, &dst, CAMERA_CV_BITDEPTH_16S, 2.0f);

	if(!result)
	{
		CameraCvSampleShowMat(dst, "After Convert To Result");
		CameraCvSampleMemoryFree(dst);
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_ROI_ST rect;
			CAMERA_CV_API_MAT_ST subSrc;
			
			rect.i_x = 1;
			rect.i_y = 2;
			rect.i_width = 3;
			rect.i_height = 2;
			
			subSrc = src & rect; //region mask
			
			CameraCvSampleShowMat(subSrc, "SubSrc Data");

			result = camera_cv_api_convert_to(&subSrc, &dst, CAMERA_CV_BITDEPTH_16S, 4.0f);
			
			if(!result)
			{
				CameraCvSampleShowMat(dst, "SubSrc Convert To Result");
				CameraCvSampleMemoryFree(dst);
			}
		}
	}
	
	return result;
}

int CameraCvSampleSplitAndMerge(void)
{
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_MAT_ST plane[3];
	CAMERA_CV_API_MAT_ST dst;
	const int IMAGE_WIDTH = 1920;
	const int IMAGE_HEIGHT = 1080;
	int32_t result = 0;
	
	//put RGB packed raw data in SD card first
	//read raw data to memory
	CameraCvSampleReadToFileBuffer(RGB2BGR_SAMPLE_INPUT_PATH, 0); // read to g_FileBuffer[0]
	
	CameraCvSampleInitMat(src, g_FileBuffer[0], IMAGE_WIDTH, IMAGE_HEIGHT, CAMERA_CV_DATA_TYPE_8UC3);
	
	result = camera_cv_api_split(&src, plane);
	
	if(result)
	{
		printf("split fail\n");
		return result;
	}
	
	std::swap(plane[0], plane[2]); // change rgb order to bgr order
	
	result = camera_cv_api_merge(plane, 3, &dst);

	if(!result)
	{
		CameraCvSampleImwriteBmp(RGB2BGR_SAMPLE_RESULT_PATH, (uint8_t*)dst.pv_data, dst.s_size.i_width, dst.s_size.i_height, IMG_TYPE_BGR);
		CameraCvSampleMemoryFree(dst);
	}
	
	for(CAMERA_CV_API_MAT_ST &mat: plane)
		CameraCvSampleMemoryFree(mat);
	
	return result;
}

int CameraCvSampleMeidanBlur(void)
{
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_MAT_ST dst;	

	int padding = 1;

	CameraCvSampleInitMat(src, g_SampleBuffer, MEDIAN_BLUR_SAMPLE_IMG_SRC_WIDTH, MEDIAN_BLUR_SAMPLE_IMG_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_8UC1);
	
	int32_t result = camera_cv_api_median_blur(&src, &dst, 3, padding);
	
	if(!result)
	{
		CameraCvSampleImwrite(MEDIAN_BLUR_SAMPLE_RESULT_PATH, dst);
		CameraCvSampleMemoryFree(dst);
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_ROI_ST rect;
			CAMERA_CV_API_MAT_ST subSrc;
			
			rect.i_x = MEDIAN_BLUR_SAMPLE_IMG_SRC_WIDTH / 3 - padding;
			rect.i_y = MEDIAN_BLUR_SAMPLE_IMG_SRC_HEIGHT / 3 - padding;
			rect.i_width = MEDIAN_BLUR_SAMPLE_IMG_SRC_WIDTH / 2 + padding * 2;
			rect.i_height = MEDIAN_BLUR_SAMPLE_IMG_SRC_HEIGHT / 2 + padding * 2;
			
			subSrc = src & rect; //region mask

			result = camera_cv_api_median_blur(&subSrc, &dst, 3, padding);
			
			if(!result)
			{
				CameraCvSampleImwrite(MEDIAN_BLUR_SAMPLE_RESULT_PATH "_adv.yuv", dst);
				CameraCvSampleMemoryFree(dst);
			}
		}
	}
	
	return result;	
}

int CameraCvSampleBoxFilter(void)
{
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_MAT_ST dst;	
	CAMERA_CV_API_SIZE_ST ksize;
	int padding = 1;

	CameraCvSampleInitMat(src, g_SampleBuffer, BOX_FILTER_SAMPLE_IMG_SRC_WIDTH, BOX_FILTER_SAMPLE_IMG_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_8UC1);
	
	ksize.i_width = ksize.i_height = 3;

	int32_t result = camera_cv_api_box_filter(&src, &dst, &ksize, padding);

	if(!result)
	{
		CameraCvSampleImwrite(BOX_FILTER_SAMPLE_RESULT_PATH, dst);
		CameraCvSampleMemoryFree(dst);
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_ROI_ST rect;
			CAMERA_CV_API_MAT_ST subSrc;

			rect.i_x = BOX_FILTER_SAMPLE_IMG_SRC_WIDTH / 3 - padding;
			rect.i_y = BOX_FILTER_SAMPLE_IMG_SRC_HEIGHT / 3 - padding;
			rect.i_width = BOX_FILTER_SAMPLE_IMG_SRC_WIDTH / 2 + padding * 2;
			rect.i_height = BOX_FILTER_SAMPLE_IMG_SRC_HEIGHT / 2 + padding * 2;
			
			subSrc = src & rect; //region mask

			result = camera_cv_api_median_blur(&subSrc, &dst, 3, padding);
			
			if(!result)
			{
				CameraCvSampleImwrite(BOX_FILTER_SAMPLE_RESULT_PATH "_adv.yuv", dst);
				CameraCvSampleMemoryFree(dst);
			}
		}
	}

	return result;	
}

int CameraCvSampleDilate(void)
{
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_MAT_ST kernel;
	CAMERA_CV_API_MAT_ST dst;	
	int padding = 1;

	uint8_t KernelArr[3][3] = {
			{0,1,0},
			{1,1,1},
			{0,1,0}
	};

	CameraCvSampleInitMat(src, g_SampleBuffer, DILATE_SAMPLE_IMG_SRC_WIDTH, DILATE_SAMPLE_IMG_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_8UC1);
	CameraCvSampleInitMat(kernel, KernelArr, 3, 3, CAMERA_CV_DATA_TYPE_8UC1);		

	int32_t result = camera_cv_api_dilate(&src, &dst, &kernel, padding);

	if(!result)
	{
		CameraCvSampleImwrite(DILATE_SAMPLE_RESULT_PATH, dst);
		CameraCvSampleMemoryFree(dst);
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_ROI_ST rect;
			CAMERA_CV_API_MAT_ST subSrc;

			rect.i_x = DILATE_SAMPLE_IMG_SRC_WIDTH / 3 - padding;
			rect.i_y = DILATE_SAMPLE_IMG_SRC_HEIGHT / 3 - padding;
			rect.i_width = DILATE_SAMPLE_IMG_SRC_WIDTH / 2 + padding * 2;
			rect.i_height = DILATE_SAMPLE_IMG_SRC_HEIGHT / 2 + padding * 2;
			
			subSrc = src & rect; //region mask

			result = camera_cv_api_dilate(&subSrc, &dst, &kernel, padding);
			
			if(!result)
			{
				CameraCvSampleImwrite(DILATE_SAMPLE_RESULT_PATH "_adv.yuv", dst);
				CameraCvSampleMemoryFree(dst);
			}
		}
	}

	return result;	
}

int CameraCvSampleErode(void)
{
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_MAT_ST kernel;
	CAMERA_CV_API_MAT_ST dst;	
	int padding = 1;

	uint8_t KernelArr[3][3] = {
			{1,0,1},
			{0,1,0},
			{1,0,1}
	};

	CameraCvSampleInitMat(src, g_SampleBuffer, ERODE_SAMPLE_IMG_SRC_WIDTH, ERODE_SAMPLE_IMG_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_8UC1);
	CameraCvSampleInitMat(kernel, KernelArr, 3, 3, CAMERA_CV_DATA_TYPE_8UC1);		

	int32_t result = camera_cv_api_erode(&src, &dst, &kernel, padding);

	if(!result)
	{
		CameraCvSampleImwrite(ERODE_SAMPLE_RESULT_PATH, dst);
		CameraCvSampleMemoryFree(dst);
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_ROI_ST rect;
			CAMERA_CV_API_MAT_ST subSrc;
	
			rect.i_x = ERODE_SAMPLE_IMG_SRC_WIDTH / 3 - padding;
			rect.i_y = ERODE_SAMPLE_IMG_SRC_HEIGHT / 3 - padding;
			rect.i_width = ERODE_SAMPLE_IMG_SRC_WIDTH / 2 + padding * 2;
			rect.i_height = ERODE_SAMPLE_IMG_SRC_HEIGHT / 2 + padding * 2;
			
			subSrc = src & rect; //region mask

			result = camera_cv_api_erode(&subSrc, &dst, &kernel, padding);
			
			if(!result)
			{
				CameraCvSampleImwrite(ERODE_SAMPLE_RESULT_PATH "_adv.yuv", dst);
				CameraCvSampleMemoryFree(dst);
			}
		}
	}

	return result;	
}

int CameraCvSampleSobel(void)
{
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_MAT_ST dst;	
	
	const int padding = 1;
	const int SOBEL_SAMPLE_SRC_WIDTH = 10;
	const int SOBEL_SAMPLE_SRC_HEIGHT = 12;		
	uint8_t sobel_sample_buffer[SOBEL_SAMPLE_SRC_HEIGHT + 2 * padding][SOBEL_SAMPLE_SRC_WIDTH + 2 * padding] = {0};

	for(int y=0;y<SOBEL_SAMPLE_SRC_HEIGHT;y++)
	{
		for(int x=0;x<SOBEL_SAMPLE_SRC_WIDTH;x++)
		{
			sobel_sample_buffer[y + padding][x  + padding] = rand() & 0xFF;
		}
	}

	CameraCvSampleInitMat(src, sobel_sample_buffer, SOBEL_SAMPLE_SRC_WIDTH + 2 * padding, SOBEL_SAMPLE_SRC_HEIGHT + 2 * padding, CAMERA_CV_DATA_TYPE_8UC1);	

	CameraCvSampleShowMat(src, "Sobel Input Buffer");

	int32_t result = camera_cv_api_sobel(&src, &dst, CAMERA_CV_BITDEPTH_16S, 1, 0, padding);	//calculation x direction sobel filter
	
	if(!result)
	{	
		CameraCvSampleShowMat(dst, "show sobel result");
		CameraCvSampleMemoryFree(dst);
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_ROI_ST rect;
			CAMERA_CV_API_MAT_ST subSrc;
	
			rect.i_x = SOBEL_SAMPLE_SRC_WIDTH / 3 - padding;
			rect.i_y = SOBEL_SAMPLE_SRC_HEIGHT / 3 - padding;
			rect.i_width = SOBEL_SAMPLE_SRC_WIDTH / 2 + padding * 2;
			rect.i_height = SOBEL_SAMPLE_SRC_HEIGHT / 2 + padding * 2;
			
			subSrc = src & rect; //region mask
			
			CameraCvSampleShowMat(subSrc, "SubSrc Data");

			result = camera_cv_api_sobel(&subSrc, &dst, CAMERA_CV_BITDEPTH_16S, 0, 1, padding);	//calculation y direction sobel filter
			
			if(!result)
			{
				CameraCvSampleShowMat(dst, "show subSrc sobel result");
				CameraCvSampleMemoryFree(dst);
			}
		}
	}

	return result;
}

int CameraCvSampleAbs(void)
{
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_MAT_ST dst;
	const int ABS_SAMPLE_SRC_WIDTH = 5;
	const int ABS_SAMPLE_SRC_HEIGHT = 3;			
	int8_t abs_sample_buffer[ABS_SAMPLE_SRC_HEIGHT][ABS_SAMPLE_SRC_WIDTH]={
		{5, -6, 17, 98, -20},
		{0, 95, -103, -1, -5},
		{5, -3, 1, -53, 68},

	};
	CameraCvSampleInitMat(src, abs_sample_buffer, ABS_SAMPLE_SRC_WIDTH, ABS_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_8SC1);

	CameraCvSampleShowMat(src, "Abs Input Buffer");

	int32_t result = camera_cv_api_abs(&src, &dst, CAMERA_CV_DATA_TYPE_8UC1);

	if(!result)
	{
		CameraCvSampleShowMat(dst, "show abs result");
		CameraCvSampleMemoryFree(dst);
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_ROI_ST rect;
			CAMERA_CV_API_MAT_ST subSrc;
			
			rect.i_x = 2;
			rect.i_y = 1;
			rect.i_width = 3;
			rect.i_height = 2;
			
			subSrc = src & rect; //region mask
			
			CameraCvSampleShowMat(subSrc, "SubSrc Data");

			result = camera_cv_api_abs(&subSrc, &dst, CAMERA_CV_DATA_TYPE_8UC1);
			
			if(!result)
			{
				CameraCvSampleShowMat(dst, "SubSrc abs result");
				CameraCvSampleMemoryFree(dst);
			}
		}
	}

	return result;
}

int CameraCvSampleAbsDiff(void)
{
	CAMERA_CV_API_MAT_ST src1;
	CAMERA_CV_API_MAT_ST src2;
	CAMERA_CV_API_MAT_ST dst;
	const int ABS_DIFF_SAMPLE_SRC_WIDTH = 5;
	const int ABS_DIFF_SAMPLE_SRC_HEIGHT = 3;			
	int16_t abs_diff_sample_buffer1[ABS_DIFF_SAMPLE_SRC_HEIGHT][ABS_DIFF_SAMPLE_SRC_WIDTH]={
		{5885, -622, 1117, 9828, -2260},
		{0, 9225, -12203, -781, -599},
		{595, -366, 551, -153, 4168},

	};
	
	int16_t abs_diff_sample_buffer2[ABS_DIFF_SAMPLE_SRC_HEIGHT][ABS_DIFF_SAMPLE_SRC_WIDTH]={
		{335, -576, 1637, 98, -3320},
		{120, 4195, -103, -1, -7535},
		{590, -345, 61, -5463, 31238},

	};
	CameraCvSampleInitMat(src1, abs_diff_sample_buffer1, ABS_DIFF_SAMPLE_SRC_WIDTH, ABS_DIFF_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_16SC1);
	CameraCvSampleInitMat(src2, abs_diff_sample_buffer2, ABS_DIFF_SAMPLE_SRC_WIDTH, ABS_DIFF_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_16SC1);	

	CameraCvSampleShowMat(src1, "abs diff buffer1");
	CameraCvSampleShowMat(src2, "abs diff buffer2");		

	int32_t result = camera_cv_api_abs_diff(&src1, &src2, &dst, CAMERA_CV_DATA_TYPE_16UC1);

	if(!result)
	{
		CameraCvSampleShowMat(dst, "show abs diff result");
		CameraCvSampleMemoryFree(dst);
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_ROI_ST rect_rd, rect_lu;
			CAMERA_CV_API_MAT_ST subSrc_rd, subSrc_lu;
			
			rect_lu.i_x = 0;
			rect_lu.i_y = 0;
			rect_lu.i_width = 3;
			rect_lu.i_height = 2;

			rect_rd.i_x = 2;
			rect_rd.i_y = 1;
			rect_rd.i_width = 3;
			rect_rd.i_height = 2;
			
			subSrc_rd = src1 & rect_rd; //region mask
			subSrc_lu = src1 & rect_lu; //region mask
			
			CameraCvSampleShowMat(subSrc_rd, "SubSrc buffer1");
			CameraCvSampleShowMat(subSrc_lu, "SubSrc buffer2");

			result = camera_cv_api_abs_diff(&subSrc_rd, &subSrc_lu, &dst, CAMERA_CV_DATA_TYPE_16UC1);
			
			if(!result)
			{
				CameraCvSampleShowMat(dst, "SubSrc absdiff result");
				CameraCvSampleMemoryFree(dst);
			}
		}
	}

	return result;
}

int CameraCvSampleAdd(void)
{
	CAMERA_CV_API_MAT_ST src1;
	CAMERA_CV_API_MAT_ST src2;
	CAMERA_CV_API_MAT_ST dst;
	const int ADD_SAMPLE_SRC_WIDTH = 5;
	const int ADD_SAMPLE_SRC_HEIGHT = 3;			
	int16_t add_sample_buffer1[ADD_SAMPLE_SRC_HEIGHT][ADD_SAMPLE_SRC_WIDTH]={
		{5885, -622, 1117, 9828, -2260},
		{0, 9225, -12203, -781, -599},
		{595, -366, 551, -153, 4168},

	};
	
	int16_t add_sample_buffer2[ADD_SAMPLE_SRC_HEIGHT][ADD_SAMPLE_SRC_WIDTH]={
		{335, -576, 1637, 98, -3320},
		{120, 4195, -103, -1, -7535},
		{590, -345, 61, -5463, 31238},

	};
	CameraCvSampleInitMat(src1, add_sample_buffer1, ADD_SAMPLE_SRC_WIDTH, ADD_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_16SC1);
	CameraCvSampleInitMat(src2, add_sample_buffer2, ADD_SAMPLE_SRC_WIDTH, ADD_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_16SC1);	

	int32_t result = camera_cv_api_add(&src1, &src2, &dst, CAMERA_CV_DATA_TYPE_16SC1);

	CameraCvSampleShowMat(src1, "add buffer1");
	CameraCvSampleShowMat(src2, "add buffer2");		

	if(!result)
	{
		CameraCvSampleShowMat(dst, "show add result");
		CameraCvSampleMemoryFree(dst);
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_ROI_ST rect_rd, rect_lu;
			CAMERA_CV_API_MAT_ST subSrc_rd, subSrc_lu;
			
			rect_lu.i_x = 0;
			rect_lu.i_y = 0;
			rect_lu.i_width = 3;
			rect_lu.i_height = 2;

			rect_rd.i_x = 2;
			rect_rd.i_y = 1;
			rect_rd.i_width = 3;
			rect_rd.i_height = 2;
			
			subSrc_rd = src1 & rect_rd; //region mask
			subSrc_lu = src1 & rect_lu; //region mask
			
			CameraCvSampleShowMat(subSrc_rd, "SubSrc buffer1");
			CameraCvSampleShowMat(subSrc_lu, "SubSrc buffer2");

			result = camera_cv_api_add(&subSrc_rd, &subSrc_lu, &dst, CAMERA_CV_DATA_TYPE_16SC1);
			
			if(!result)
			{
				CameraCvSampleShowMat(dst, "SubSrc add result");
				CameraCvSampleMemoryFree(dst);
			}
		}
	}

	return result;
}

int CameraCvSampleAddWeight(void)
{
	CAMERA_CV_API_MAT_ST src1;
	CAMERA_CV_API_MAT_ST src2;
	CAMERA_CV_API_MAT_ST dst;
	const int ADD_WEIGHT_SAMPLE_SRC_WIDTH = 5;
	const int ADD_WEIGHT_SAMPLE_SRC_HEIGHT = 3;			
	int16_t add_weight_sample_buffer1[ADD_WEIGHT_SAMPLE_SRC_HEIGHT][ADD_WEIGHT_SAMPLE_SRC_WIDTH]={
		{5885, -622, 1117, 9828, -2260},
		{0, 9225, -12203, -781, -599},
		{595, -366, 551, -153, 4168},

	};
	
	int16_t add_weight_sample_buffer2[ADD_WEIGHT_SAMPLE_SRC_HEIGHT][ADD_WEIGHT_SAMPLE_SRC_WIDTH]={
		{335, -576, 1637, 98, -3320},
		{120, 4195, -103, -1, -7535},
		{590, -345, 61, -5463, 31238},

	};
	CameraCvSampleInitMat(src1, add_weight_sample_buffer1, ADD_WEIGHT_SAMPLE_SRC_WIDTH, ADD_WEIGHT_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_16SC1);
	CameraCvSampleInitMat(src2, add_weight_sample_buffer2, ADD_WEIGHT_SAMPLE_SRC_WIDTH, ADD_WEIGHT_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_16SC1);	

	int32_t result = camera_cv_api_add_weighted(&src1, 2.0, &src2, -1.0, 100, &dst, CAMERA_CV_DATA_TYPE_16SC1);
	//dst = src1 * 2 - src2 + 100

	CameraCvSampleShowMat(src1, "add weight buffer1");
	CameraCvSampleShowMat(src2, "add weight buffer2");		

	if(!result)
	{
		CameraCvSampleShowMat(dst, "show add weight result");
		CameraCvSampleMemoryFree(dst);
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_ROI_ST rect_rd, rect_lu;
			CAMERA_CV_API_MAT_ST subSrc_rd, subSrc_lu;
			
			rect_lu.i_x = 0;
			rect_lu.i_y = 0;
			rect_lu.i_width = 3;
			rect_lu.i_height = 2;

			rect_rd.i_x = 2;
			rect_rd.i_y = 1;
			rect_rd.i_width = 3;
			rect_rd.i_height = 2;
			
			subSrc_rd = src1 & rect_rd; //region mask
			subSrc_lu = src1 & rect_lu; //region mask
			
			CameraCvSampleShowMat(subSrc_rd, "SubSrc buffer1");
			CameraCvSampleShowMat(subSrc_lu, "SubSrc buffer2");

			result = camera_cv_api_add_weighted(&subSrc_rd, 2.0, &subSrc_lu, -1.0, 100, &dst, CAMERA_CV_DATA_TYPE_16SC1);
			
			if(!result)
			{
				CameraCvSampleShowMat(dst, "SubSrc add weight result");
				CameraCvSampleMemoryFree(dst);
			}
		}
	}

	return result;	
}

int CameraCvSampleBitwiseOperation(OPERATION_TYPE operation)
{
	CAMERA_CV_API_MAT_ST src1;
	CAMERA_CV_API_MAT_ST src2;
	CAMERA_CV_API_MAT_ST dst;
	const int BITWISE_OP_SAMPLE_SRC_WIDTH = 5;
	const int BITWISE_OP_SAMPLE_SRC_HEIGHT = 3;			
	uint8_t bitwise_sample_buffer1[BITWISE_OP_SAMPLE_SRC_HEIGHT][BITWISE_OP_SAMPLE_SRC_WIDTH]={
		{58, 22, 117, 28, 60},
		{0, 25, 203, 81, 99},
		{55, 66, 51, 153, 68},

	};

	uint8_t bitwise_sample_buffer2[BITWISE_OP_SAMPLE_SRC_HEIGHT][BITWISE_OP_SAMPLE_SRC_WIDTH]={
		{35, 76, 13, 98, 20},
		{120, 95, 103, 1, 35},
		{50, 45, 61, 63, 38},

	};
	CameraCvSampleInitMat(src1, bitwise_sample_buffer1, BITWISE_OP_SAMPLE_SRC_WIDTH, BITWISE_OP_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_8UC1);
	CameraCvSampleInitMat(src2, bitwise_sample_buffer2, BITWISE_OP_SAMPLE_SRC_WIDTH, BITWISE_OP_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_8UC1);	

	int32_t result = -1;
	const char* oper_result_desc = "";

	switch(operation)
	{
		case OPERATION_AND:
			result = camera_cv_api_bitwise_and(&src1, &src2, &dst);
			oper_result_desc = "show bitwise and result";
			break;
		case OPERATION_OR:
			result = camera_cv_api_bitwise_or(&src1, &src2, &dst);
			oper_result_desc = "show bitwise or result";
			break;
		case OPERATION_NOT:
			result = camera_cv_api_bitwise_not(&src1, &dst);
			oper_result_desc = "show bitwise not result";
			break;
		case OPERATION_XOR:
			result = camera_cv_api_bitwise_xor(&src1, &src2, &dst);
			oper_result_desc = "show bitwise xor result";
			break;
	}


	CameraCvSampleShowMat(src1, "bitwise op buffer1");
	CameraCvSampleShowMat(src2, "bitwise op buffer2");		

	if(!result)
	{
		CameraCvSampleShowMat(dst, oper_result_desc);
		CameraCvSampleMemoryFree(dst);
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_ROI_ST rect_rd, rect_lu;
			CAMERA_CV_API_MAT_ST subSrc_rd, subSrc_lu;
			
			rect_lu.i_x = 0;
			rect_lu.i_y = 0;
			rect_lu.i_width = 3;
			rect_lu.i_height = 2;

			rect_rd.i_x = 2;
			rect_rd.i_y = 1;
			rect_rd.i_width = 3;
			rect_rd.i_height = 2;
			
			subSrc_rd = src1 & rect_rd; //region mask
			subSrc_lu = src1 & rect_lu; //region mask
			
			CameraCvSampleShowMat(subSrc_rd, "SubSrc buffer1");
			CameraCvSampleShowMat(subSrc_lu, "SubSrc buffer2");

			switch(operation)
			{
				case OPERATION_AND:
					result = camera_cv_api_bitwise_and(&subSrc_rd, &subSrc_lu, &dst);
					break;
				case OPERATION_OR:
					result = camera_cv_api_bitwise_or(&subSrc_rd, &subSrc_lu, &dst);
					break;
				case OPERATION_NOT:
					result = camera_cv_api_bitwise_not(&subSrc_rd, &dst);
					break;
				case OPERATION_XOR:
					result = camera_cv_api_bitwise_xor(&subSrc_rd, &subSrc_lu, &dst);
					break;
			}
			
			if(!result)
			{
				CameraCvSampleShowMat(dst, oper_result_desc);
				CameraCvSampleMemoryFree(dst);
			}
		}
	}

	return result;	
}

int CameraCvSampleCountNonZero(void)
{
	CAMERA_CV_API_MAT_ST src;
	const int COUNT_NON_ZERO_SAMPLE_SRC_WIDTH = 5;
	const int COUNT_NON_ZERO_SAMPLE_SRC_HEIGHT = 3;			
	uint8_t count_non_zero_sample_buffer[COUNT_NON_ZERO_SAMPLE_SRC_HEIGHT][COUNT_NON_ZERO_SAMPLE_SRC_WIDTH]={
		{58, 22, 0, 28, 60},
		{0, 25, 203, 81, 0},
		{55, 66, 51, 0, 68},

	};

	int count = -1;
	
	CameraCvSampleInitMat(src, count_non_zero_sample_buffer, COUNT_NON_ZERO_SAMPLE_SRC_WIDTH, COUNT_NON_ZERO_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_8UC1);
	CameraCvSampleShowMat(src, "count non zero sample buffer");
	
	int32_t result = camera_cv_api_count_non_zero(&src, &count);
	
	if(!result)
	{
		printf("non zero count = %d\n", count);
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_ROI_ST rect;
			CAMERA_CV_API_MAT_ST subSrc;
			
			rect.i_x = 2;
			rect.i_y = 1;
			rect.i_width = 3;
			rect.i_height = 2;
			
			subSrc = src & rect; //region mask
			
			CameraCvSampleShowMat(subSrc, "SubSrc Data");

			result = camera_cv_api_count_non_zero(&subSrc, &count);
			
			if(!result)
			{
				printf("subSrc non zero count = %d\n", count);
			}
		}
	}

	return result;
}

int CameraCvSampleExp(void)
{
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_MAT_ST dst;	
	const int EXP_SAMPLE_SRC_WIDTH = 8;
	const int EXP_SAMPLE_SRC_HEIGHT = 8;			
	uint16_t exp_sample_buffer[EXP_SAMPLE_SRC_HEIGHT][EXP_SAMPLE_SRC_WIDTH]={0};
	uint16_t num = 0;
	uint16_t *p_num = (uint16_t*)exp_sample_buffer;
	for(int idx = 0; idx <EXP_SAMPLE_SRC_WIDTH * EXP_SAMPLE_SRC_HEIGHT;idx ++)
	{
		*p_num++ = num;
		num += 16;
	}

	CameraCvSampleInitMat(src, exp_sample_buffer, EXP_SAMPLE_SRC_WIDTH, EXP_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_16UC1);
	CameraCvSampleShowMat(src, "exp sample buffer");	

	int32_t result = camera_cv_api_exp(&src, &dst);
		
	if(!result)
	{
		CameraCvSampleShowMat(dst, "exp result");
		CameraCvSampleMemoryFree(dst);
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_ROI_ST rect;
			CAMERA_CV_API_MAT_ST subSrc;
			
			rect.i_x = 4;
			rect.i_y = 3;
			rect.i_width = 3;
			rect.i_height = 4;
			
			subSrc = src & rect; //region mask
			
			CameraCvSampleShowMat(subSrc, "SubSrc Data");

			result = camera_cv_api_exp(&subSrc, &dst);
			
			if(!result)
			{
				CameraCvSampleShowMat(dst, "subSrc exp result");
				CameraCvSampleMemoryFree(dst);
			}
		}
	}

	return result;
}

int CameraCvSampleLut(void)
{
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_MAT_ST lut;
	CAMERA_CV_API_MAT_ST dst;
	uint8_t lookup_table[256];
	for(int i=0;i<256;i++)
	{
		lookup_table[i] = (i >> 4)<<4;
	}

	CameraCvSampleInitMat(src, g_SampleBuffer, LUT_SAMPLE_IMG_SRC_WIDTH, LUT_SAMPLE_IMG_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_8UC1);
	CameraCvSampleInitMat(lut, lookup_table, 256, 1, CAMERA_CV_DATA_TYPE_8UC1);

	int32_t result = camera_cv_api_lut(&src, &lut, &dst);

	if(!result)
	{
		CameraCvSampleImwrite(LUT_SAMPLE_RESULT_PATH, dst);
		CameraCvSampleMemoryFree(dst);
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_ROI_ST rect;
			CAMERA_CV_API_MAT_ST subSrc;
			
			rect.i_x = LUT_SAMPLE_IMG_SRC_WIDTH / 3;
			rect.i_y = LUT_SAMPLE_IMG_SRC_HEIGHT / 3;
			rect.i_width = LUT_SAMPLE_IMG_SRC_WIDTH / 2;
			rect.i_height = LUT_SAMPLE_IMG_SRC_HEIGHT / 2;
			
			subSrc = src & rect; //region mask

			result = camera_cv_api_lut(&subSrc, &lut, &dst);
			
			if(!result)
			{
				CameraCvSampleImwrite(LUT_SAMPLE_RESULT_PATH "_adv.yuv", dst);
				CameraCvSampleMemoryFree(dst);
			}
		}
	}
	return result;
}

int CameraCvSampleMagnitude(void)
{
	CAMERA_CV_API_MAT_ST x;
	CAMERA_CV_API_MAT_ST y;
	CAMERA_CV_API_MAT_ST dst;
	const int MAGNITUDE_SAMPLE_SRC_WIDTH = 3;
	const int MAGNITUDE_SAMPLE_SRC_HEIGHT = 2;			
	/*
	3,4,5
	5,12,13
	7,24,25
	8,15,17
	9,40,41
	11,60,61 //cv api result == 60
	*/
	int16_t magnitude_sample_buffer1[MAGNITUDE_SAMPLE_SRC_HEIGHT][MAGNITUDE_SAMPLE_SRC_WIDTH]={
		{3, -12, 7},
		{8, 40, -60},
	};
	
	int16_t magnitude_sample_buffer2[MAGNITUDE_SAMPLE_SRC_HEIGHT][MAGNITUDE_SAMPLE_SRC_WIDTH]={
		{4, -5, 24},
		{15, 9, -11},
	};
	
	CameraCvSampleInitMat(x, magnitude_sample_buffer1, MAGNITUDE_SAMPLE_SRC_WIDTH, MAGNITUDE_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_16SC1);
	CameraCvSampleInitMat(y, magnitude_sample_buffer2, MAGNITUDE_SAMPLE_SRC_WIDTH, MAGNITUDE_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_16SC1);	

	int32_t result = camera_cv_api_magnitude(&x, &y, &dst, CAMERA_CV_DATA_TYPE_16SC1);

	CameraCvSampleShowMat(x, "X buffer");
	CameraCvSampleShowMat(y, "Y buffer");		

	if(!result)
	{
		CameraCvSampleShowMat(dst, "show magnitude result");
		CameraCvSampleMemoryFree(dst);
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_ROI_ST rect_rd, rect_lu;
			CAMERA_CV_API_MAT_ST subSrc_rd, subSrc_lu;
			
			rect_lu.i_x = 0;
			rect_lu.i_y = 0;
			rect_lu.i_width = 2;
			rect_lu.i_height = 2;

			rect_rd.i_x = 1;
			rect_rd.i_y = 0;
			rect_rd.i_width = 2;
			rect_rd.i_height = 2;
			
			subSrc_rd = y & rect_rd; //region mask
			subSrc_lu = x & rect_lu; //region mask
			
			CameraCvSampleShowMat(subSrc_rd, "SubSrc X");
			CameraCvSampleShowMat(subSrc_lu, "SubSrc Y");

			result = camera_cv_api_magnitude(&subSrc_rd, &subSrc_lu, &dst, CAMERA_CV_DATA_TYPE_16SC1);
			
			if(!result)
			{
				CameraCvSampleShowMat(dst, "SubSrc add weight result");
				CameraCvSampleMemoryFree(dst);
			}
		}
	}

	return result;	
}

int CameraCvSampleMinMax(bool getMin)
{
	CAMERA_CV_API_MAT_ST src1;
	CAMERA_CV_API_MAT_ST src2;
	CAMERA_CV_API_MAT_ST dst;
	const int MIN_MAX_SAMPLE_SRC_WIDTH = 5;
	const int MIN_MAX_SAMPLE_SRC_HEIGHT = 3;			
	uint8_t min_max_sample_buffer1[MIN_MAX_SAMPLE_SRC_HEIGHT][MIN_MAX_SAMPLE_SRC_WIDTH]={
		{58, 22, 117, 28, 60},
		{0, 25, 203, 81, 99},
		{55, 66, 51, 153, 68},

	};

	uint8_t min_max_sample_buffer2[MIN_MAX_SAMPLE_SRC_HEIGHT][MIN_MAX_SAMPLE_SRC_WIDTH]={
		{35, 76, 13, 98, 20},
		{120, 95, 103, 1, 35},
		{50, 45, 61, 63, 38},

	};
	CameraCvSampleInitMat(src1, min_max_sample_buffer1, MIN_MAX_SAMPLE_SRC_WIDTH, MIN_MAX_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_8UC1);
	CameraCvSampleInitMat(src2, min_max_sample_buffer2, MIN_MAX_SAMPLE_SRC_WIDTH, MIN_MAX_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_8UC1);	

	int32_t result = -1;
	const char* min_max_result_desc = "";

	if(getMin)
	{		
		result = camera_cv_api_min(&src1, &src2, &dst);
		min_max_result_desc = "show min result";
	}
	else
	{
		result = camera_cv_api_max(&src1, &src2, &dst);
		min_max_result_desc = "show max result";
	}


	CameraCvSampleShowMat(src1, "compare buffer1");
	CameraCvSampleShowMat(src2, "compare buffer2");		

	if(!result)
	{
		CameraCvSampleShowMat(dst, min_max_result_desc);
		CameraCvSampleMemoryFree(dst); 
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_ROI_ST rect_rd, rect_lu;
			CAMERA_CV_API_MAT_ST subSrc_rd, subSrc_lu;
			
			rect_lu.i_x = 0;
			rect_lu.i_y = 0;
			rect_lu.i_width = 3;
			rect_lu.i_height = 2;

			rect_rd.i_x = 2;
			rect_rd.i_y = 1;
			rect_rd.i_width = 3;
			rect_rd.i_height = 2;
			
			subSrc_rd = src1 & rect_rd; //region mask
			subSrc_lu = src2 & rect_lu; //region mask
			
			CameraCvSampleShowMat(subSrc_rd, "SubSrc buffer1");
			CameraCvSampleShowMat(subSrc_lu, "SubSrc buffer2");
			
			if(getMin)
			{		
				result = camera_cv_api_min(&subSrc_rd, &subSrc_lu, &dst);
			}
			else
			{
				result = camera_cv_api_max(&subSrc_rd, &subSrc_lu, &dst);
			}
			
			if(!result)
			{
				CameraCvSampleShowMat(dst, min_max_result_desc);
				CameraCvSampleMemoryFree(dst);
			}
		}
	}

	return result;	
}

int CameraCvSampleMinMaxLoc(void)
{

	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_MAT_ST minloc;
	CAMERA_CV_API_MAT_ST maxloc;
	float minVal, maxVal;
	const int MIN_MAX_LOC_SAMPLE_SRC_WIDTH = 5;
	const int MIN_MAX_LOC_SAMPLE_SRC_HEIGHT = 3;

	uint8_t min_max_loc_sample_buffer[MIN_MAX_LOC_SAMPLE_SRC_HEIGHT][MIN_MAX_LOC_SAMPLE_SRC_WIDTH]={
		{58, 22, 117, 28, 60},
		{0, 25, 203, 81, 99},
		{55, 66, 51, 153, 68},
	};

	CameraCvSampleInitMat(src, min_max_loc_sample_buffer, MIN_MAX_LOC_SAMPLE_SRC_WIDTH, MIN_MAX_LOC_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_8UC1);
	CameraCvSampleShowMat(src, "min max loc buffer");

	int32_t result = camera_cv_api_min_max_loc(&src, &minVal, &maxVal, &minloc, &maxloc);

	if(!result)
	{
		CameraCvSampleShowMat(minloc, "Min Location");
		printf("Min Value = %f\n", minVal);
		CameraCvSampleShowMat(maxloc, "Max Location");		
		printf("Max Value = %f\n", maxVal);
		CameraCvSampleMemoryFree(minloc);
		CameraCvSampleMemoryFree(maxloc);
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_ROI_ST rect;
			CAMERA_CV_API_MAT_ST subSrc;
			
			rect.i_x = 2;
			rect.i_y = 1;
			rect.i_width = 3;
			rect.i_height = 2;
			
			subSrc = src & rect; //region mask
			
			CameraCvSampleShowMat(subSrc, "SubSrc Data");

			result = camera_cv_api_min_max_loc(&subSrc, &minVal, &maxVal, &minloc, &maxloc);
			
			if(!result)
			{
				CameraCvSampleShowMat(minloc, "subSrc Min Location");
				printf("Min Value = %f\n", minVal);
				CameraCvSampleShowMat(maxloc, "subSrc Max Location");		
				printf("Max Value = %f\n", maxVal);
				CameraCvSampleMemoryFree(minloc);
				CameraCvSampleMemoryFree(maxloc);
			}
		}
	}

	return result;
}

int CameraCvSampleMultiply(void)
{
	CAMERA_CV_API_MAT_ST src1;
	CAMERA_CV_API_MAT_ST src2;
	CAMERA_CV_API_MAT_ST dst;
	const int MULTIPLY_SAMPLE_SRC_WIDTH = 5;
	const int MULTIPLY_SAMPLE_SRC_HEIGHT = 3;			
	uint8_t multiply_sample_buffer1[MULTIPLY_SAMPLE_SRC_HEIGHT][MULTIPLY_SAMPLE_SRC_WIDTH]={
		{58, 22, 117, 28, 60},
		{0, 25, 203, 81, 99},
		{55, 66, 51, 153, 68},

	};

	uint8_t multiply_sample_buffer2[MULTIPLY_SAMPLE_SRC_HEIGHT][MULTIPLY_SAMPLE_SRC_WIDTH]={
		{35, 76, 13, 98, 20},
		{120, 95, 103, 1, 35},
		{50, 45, 61, 63, 38},

	};
	CameraCvSampleInitMat(src1, multiply_sample_buffer1, MULTIPLY_SAMPLE_SRC_WIDTH, MULTIPLY_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_8UC1);
	CameraCvSampleInitMat(src2, multiply_sample_buffer2, MULTIPLY_SAMPLE_SRC_WIDTH, MULTIPLY_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_8UC1); 

	int32_t result = camera_cv_api_multiply(&src1, &src2, &dst, 1.0f/64.0f, CAMERA_CV_DATA_TYPE_8UC1);

	CameraCvSampleShowMat(src1, "multiply buffer1");
	CameraCvSampleShowMat(src2, "multiply buffer2"); 	

	if(!result)
	{
		CameraCvSampleShowMat(dst, "multiply result");
		CameraCvSampleMemoryFree(dst);
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_ROI_ST rect_rd, rect_lu;
			CAMERA_CV_API_MAT_ST subSrc_rd, subSrc_lu;
			
			rect_lu.i_x = 0;
			rect_lu.i_y = 0;
			rect_lu.i_width = 3;
			rect_lu.i_height = 2;

			rect_rd.i_x = 2;
			rect_rd.i_y = 1;
			rect_rd.i_width = 3;
			rect_rd.i_height = 2;
			
			subSrc_rd = src1 & rect_rd; //region mask
			subSrc_lu = src1 & rect_lu; //region mask
			
			CameraCvSampleShowMat(subSrc_rd, "SubSrc buffer1");
			CameraCvSampleShowMat(subSrc_lu, "SubSrc buffer2");
			
			camera_cv_api_multiply(&subSrc_rd, &subSrc_lu, &dst, 1.0f/32.0f, CAMERA_CV_DATA_TYPE_8UC1);
			
			if(!result)
			{
				CameraCvSampleShowMat(dst, "subSrc multiply result");
				CameraCvSampleMemoryFree(dst);
			}
		}
	}

	return result;	
}

int CameraCvSamplePhase(void)
{
	CAMERA_CV_API_MAT_ST x;
	CAMERA_CV_API_MAT_ST y;
	CAMERA_CV_API_MAT_ST dst;
	const int PHASE_SAMPLE_SRC_WIDTH = 3;
	const int PHASE_SAMPLE_SRC_HEIGHT = 2;			

	int16_t phase_sample_buffer1[PHASE_SAMPLE_SRC_HEIGHT][PHASE_SAMPLE_SRC_WIDTH]={
		{3, -12, 7},
		{8, 40, -60},
	};
	
	int16_t phase_sample_buffer2[PHASE_SAMPLE_SRC_HEIGHT][PHASE_SAMPLE_SRC_WIDTH]={
		{4, -5, 24},
		{15, 9, -11},
	};
	
	CameraCvSampleInitMat(x, phase_sample_buffer1, PHASE_SAMPLE_SRC_WIDTH, PHASE_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_16SC1);
	CameraCvSampleInitMat(y, phase_sample_buffer2, PHASE_SAMPLE_SRC_WIDTH, PHASE_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_16SC1);	

	int32_t result = camera_cv_api_phase(&x, &y, &dst);

	CameraCvSampleShowMat(x, "X buffer");
	CameraCvSampleShowMat(y, "Y buffer");		

	if(!result)
	{
		CameraCvSampleShowMat(dst, "show phase(real deg = output deg * 360 / 256) result");
		CameraCvSampleMemoryFree(dst);
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_ROI_ST rect_rd, rect_lu;
			CAMERA_CV_API_MAT_ST subSrc_rd, subSrc_lu;
			
			rect_lu.i_x = 0;
			rect_lu.i_y = 0;
			rect_lu.i_width = 2;
			rect_lu.i_height = 2;

			rect_rd.i_x = 1;
			rect_rd.i_y = 0;
			rect_rd.i_width = 2;
			rect_rd.i_height = 2;
			
			subSrc_rd = x & rect_rd; //region mask
			subSrc_lu = y & rect_lu; //region mask
			
			CameraCvSampleShowMat(subSrc_rd, "SubSrc x");
			CameraCvSampleShowMat(subSrc_lu, "SubSrc y");
			
			result = camera_cv_api_phase(&subSrc_rd, &subSrc_lu, &dst);
			
			if(!result)
			{
				CameraCvSampleShowMat(dst, "show phase(real deg = output deg * 360 / 256) result");
				CameraCvSampleMemoryFree(dst);
			}
		}
	}

	return result;	
}

int CameraCvSampleScaleAdd(void)
{
	CAMERA_CV_API_MAT_ST src1;
	CAMERA_CV_API_MAT_ST src2;
	CAMERA_CV_API_MAT_ST dst;
	const int SCALE_ADD_SAMPLE_SRC_WIDTH = 5;
	const int SCALE_ADD_SAMPLE_SRC_HEIGHT = 3;			
	int16_t scale_add_sample_buffer1[SCALE_ADD_SAMPLE_SRC_HEIGHT][SCALE_ADD_SAMPLE_SRC_WIDTH]={
		{5885, -622, 1117, 9828, -2260},
		{0, 9225, -12203, -781, -599},
		{595, -366, 551, -153, 4168},

	};
	
	int16_t scale_add_sample_buffer2[SCALE_ADD_SAMPLE_SRC_HEIGHT][SCALE_ADD_SAMPLE_SRC_WIDTH]={
		{335, -576, 1637, 98, -3320},
		{120, 4195, -103, -1, -7535},
		{590, -345, 61, -5463, 31238},

	};
	CameraCvSampleInitMat(src1, scale_add_sample_buffer1, SCALE_ADD_SAMPLE_SRC_WIDTH, SCALE_ADD_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_16SC1);
	CameraCvSampleInitMat(src2, scale_add_sample_buffer2, SCALE_ADD_SAMPLE_SRC_WIDTH, SCALE_ADD_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_16SC1);	

	int32_t result = camera_cv_api_scale_add(&src1, 2.0, &src2, &dst, CAMERA_CV_DATA_TYPE_16SC1);
	//dst = src1 * 2 + src2

	CameraCvSampleShowMat(src1, "scale add buffer1");
	CameraCvSampleShowMat(src2, "scale add buffer2");		

	if(!result)
	{
		CameraCvSampleShowMat(dst, "show scale add result");
		CameraCvSampleMemoryFree(dst);
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_ROI_ST rect_rd, rect_lu;
			CAMERA_CV_API_MAT_ST subSrc_rd, subSrc_lu;
			
			rect_lu.i_x = 0;
			rect_lu.i_y = 0;
			rect_lu.i_width = 3;
			rect_lu.i_height = 2;

			rect_rd.i_x = 2;
			rect_rd.i_y = 1;
			rect_rd.i_width = 3;
			rect_rd.i_height = 2;
			
			subSrc_rd = src1 & rect_rd; //region mask
			subSrc_lu = src1 & rect_lu; //region mask
			
			CameraCvSampleShowMat(subSrc_rd, "SubSrc buffer1");
			CameraCvSampleShowMat(subSrc_lu, "SubSrc buffer2");
			
			result = camera_cv_api_scale_add(&subSrc_rd, 3.0, &subSrc_lu, &dst, CAMERA_CV_DATA_TYPE_16SC1);
			
			if(!result)
			{
				CameraCvSampleShowMat(dst, "show subSrc scale add result");
				CameraCvSampleMemoryFree(dst);
			}
		}
	}

	return result;	
}

int CameraCvSampleSort(void)
{
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_MAT_ST dst;
	uint32_t sort_sample_buffer[30];
	for(auto &v:sort_sample_buffer)
	{
		v = rand();
	}

	CameraCvSampleInitMat(src, sort_sample_buffer, sizeof(sort_sample_buffer)/sizeof(*sort_sample_buffer), 1, CAMERA_CV_DATA_TYPE_32UC1);
	CameraCvSampleShowMat(src, "sort buffer");

	int32_t result = camera_cv_api_sort(&src, &dst, 0);

	if(!result)
	{
		CameraCvSampleShowMat(dst, "show sort result");
		CameraCvSampleMemoryFree(dst);
	}

	return result;	
}

int CameraCvSampleSubtract(void)
{
	CAMERA_CV_API_MAT_ST src1;
	CAMERA_CV_API_MAT_ST src2;
	CAMERA_CV_API_MAT_ST dst;
	const int SUBTRACT_SAMPLE_SRC_WIDTH = 5;
	const int SUBTRACT_SAMPLE_SRC_HEIGHT = 3;			
	uint8_t subtract_sample_buffer1[SUBTRACT_SAMPLE_SRC_HEIGHT][SUBTRACT_SAMPLE_SRC_WIDTH]={
		{58, 22, 117, 28, 60},
		{0, 25, 203, 81, 99},
		{55, 66, 51, 153, 68},

	};

	uint8_t subtract_sample_buffer2[SUBTRACT_SAMPLE_SRC_HEIGHT][SUBTRACT_SAMPLE_SRC_WIDTH]={
		{35, 76, 13, 98, 20},
		{120, 95, 103, 1, 35},
		{50, 45, 61, 63, 38},

	};
	CameraCvSampleInitMat(src1, subtract_sample_buffer1, SUBTRACT_SAMPLE_SRC_WIDTH, SUBTRACT_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_8UC1);
	CameraCvSampleInitMat(src2, subtract_sample_buffer2, SUBTRACT_SAMPLE_SRC_WIDTH, SUBTRACT_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_8UC1); 

	int32_t result = camera_cv_api_subtract(&src1, &src2, &dst, CAMERA_CV_DATA_TYPE_16SC1);

	CameraCvSampleShowMat(src1, "subtract buffer1");
	CameraCvSampleShowMat(src2, "subtract buffer2"); 	

	if(!result)
	{
		CameraCvSampleShowMat(dst, "subtract result");
		CameraCvSampleMemoryFree(dst);
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_ROI_ST rect_rd, rect_lu;
			CAMERA_CV_API_MAT_ST subSrc_rd, subSrc_lu;
			
			rect_lu.i_x = 0;
			rect_lu.i_y = 0;
			rect_lu.i_width = 3;
			rect_lu.i_height = 2;

			rect_rd.i_x = 2;
			rect_rd.i_y = 1;
			rect_rd.i_width = 3;
			rect_rd.i_height = 2;
			
			subSrc_rd = src1 & rect_rd; //region mask
			subSrc_lu = src1 & rect_lu; //region mask
			
			CameraCvSampleShowMat(subSrc_rd, "SubSrc buffer1");
			CameraCvSampleShowMat(subSrc_lu, "SubSrc buffer2");
			
			result = camera_cv_api_subtract(&subSrc_rd, &subSrc_lu, &dst, CAMERA_CV_DATA_TYPE_16SC1);
			
			if(!result)
			{
				CameraCvSampleShowMat(dst, "show subSrc subtract result");
				CameraCvSampleMemoryFree(dst);
			}
		}
	}

	return result;	
}

int CameraCvSampleSum(void)
{
	CAMERA_CV_API_MAT_ST src;
	const int SUM_SAMPLE_SRC_WIDTH = 5;
	const int SUM_SAMPLE_SRC_HEIGHT = 3;
	float sum = 0;

	int16_t sum_sample_buffer[SUM_SAMPLE_SRC_HEIGHT][SUM_SAMPLE_SRC_WIDTH]={
		{5885, -622, 1117, 9828, -2260},
		{0, 9225, -12203, -781, -599},
		{595, -366, 551, -153, 4168},
	};

	CameraCvSampleInitMat(src, sum_sample_buffer, SUM_SAMPLE_SRC_WIDTH, SUM_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_16SC1);
	CameraCvSampleShowMat(src, "sum sample buffer");

	int32_t result = camera_cv_api_sum(&src, &sum);

	if(!result)
	{
		printf("Sum = %f\n", sum);
		
		if(g_AdvanceTest)
		{
			CAMERA_CV_API_ROI_ST rect;
			CAMERA_CV_API_MAT_ST subSrc;
			
			rect.i_x = 2;
			rect.i_y = 1;
			rect.i_width = 3;
			rect.i_height = 2;
			
			subSrc = src & rect; //region mask
			
			CameraCvSampleShowMat(subSrc, "SubSrc Data");

			result = camera_cv_api_sum(&subSrc, &sum);
			
			if(!result)
			{
				printf("Sum = %f\n", sum);
			}
		}
	}

	return result;
}

int CameraCvSampleAcc(void)
{
	CAMERA_CV_API_MAT_ST src1;
	CAMERA_CV_API_MAT_ST src2;
	CAMERA_CV_API_MAT_ST src3;	
	CAMERA_CV_API_MAT_ST junk;		
	const int ACC_SAMPLE_SRC_WIDTH = 5;
	const int ACC_SAMPLE_SRC_HEIGHT = 3;			
	int16_t acc_sample_acc_buffer[ACC_SAMPLE_SRC_HEIGHT][ACC_SAMPLE_SRC_WIDTH]={
		{58, 22, 117, 28, 60},
		{0, 25, 203, 81, 99},
		{55, 66, 51, 153, 68},

	};

	uint8_t acc_sample_add_buffer1[ACC_SAMPLE_SRC_HEIGHT][ACC_SAMPLE_SRC_WIDTH]={
		{35, 76, 13, 98, 20},
		{120, 95, 103, 1, 35},
		{50, 45, 61, 63, 38},

	};

	uint8_t acc_sample_add_buffer2[ACC_SAMPLE_SRC_HEIGHT][ACC_SAMPLE_SRC_WIDTH]={
		{120, 95, 103, 1, 35},
		{50, 45, 61, 63, 38},
		{35, 76, 13, 98, 20},
		
	};
	
	int32_t result = -1;
		
	CameraCvSampleInitMat(src1, acc_sample_acc_buffer, ACC_SAMPLE_SRC_WIDTH, ACC_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_16SC1);
	CameraCvSampleInitMat(src2, acc_sample_add_buffer1, ACC_SAMPLE_SRC_WIDTH, ACC_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_8UC1); 
	CameraCvSampleInitMat(src3, acc_sample_add_buffer2, ACC_SAMPLE_SRC_WIDTH, ACC_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_8UC1); 

	CameraCvSampleShowMat(src1, "Src1");
	CameraCvSampleShowMat(src2, "Src2"); 	
	CameraCvSampleShowMat(src3, "Src3"); 		

	result = camera_cv_api_accumulate(&src1, &src2, &junk);

	if(!result)
	{
		CameraCvSampleShowMat(src1, "Add once");
	}

	result = camera_cv_api_accumulate(&src1, &src3, &junk);

	if(!result)
	{
		CameraCvSampleShowMat(src1, "Add twice");
	}
	
	if(g_AdvanceTest)
	{
		CAMERA_CV_API_ROI_ST rect_rd, rect_lu;
		CAMERA_CV_API_MAT_ST subSrc_rd, subSrc_lu;
		
		rect_lu.i_x = 0;
		rect_lu.i_y = 0;
		rect_lu.i_width = 3;
		rect_lu.i_height = 2;

		rect_rd.i_x = 2;
		rect_rd.i_y = 1;
		rect_rd.i_width = 3;
		rect_rd.i_height = 2;
		
		subSrc_rd = src1 & rect_rd; //region mask
		subSrc_lu = src2 & rect_lu; //region mask
		
		CameraCvSampleShowMat(subSrc_rd, "SubSrc buffer1");
		CameraCvSampleShowMat(subSrc_lu, "SubSrc buffer2");
		
		result = camera_cv_api_accumulate(&subSrc_rd, &subSrc_lu, &junk);
		
		if(!result)
		{
			CameraCvSampleShowMat(src1, "Src 1");
			CameraCvSampleShowMat(src2, "Src 2");
		}
	}

	return result;	
}

int CameraCvSampleAccSquare(void)
{
	CAMERA_CV_API_MAT_ST src1;
	CAMERA_CV_API_MAT_ST src2;
	CAMERA_CV_API_MAT_ST src3;	
	CAMERA_CV_API_MAT_ST junk;		
	const int ACC_SAMPLE_SRC_WIDTH = 5;
	const int ACC_SAMPLE_SRC_HEIGHT = 3;			
	int16_t acc_sample_acc_buffer[ACC_SAMPLE_SRC_HEIGHT][ACC_SAMPLE_SRC_WIDTH]={
		{58, 22, 117, 28, 60},
		{0, 25, 203, 81, 99},
		{55, 66, 51, 153, 68},

	};

	uint8_t acc_sample_add_buffer1[ACC_SAMPLE_SRC_HEIGHT][ACC_SAMPLE_SRC_WIDTH]={
		{35, 76, 13, 98, 20},
		{120, 95, 103, 1, 35},
		{50, 45, 61, 63, 38},

	};

	uint8_t acc_sample_add_buffer2[ACC_SAMPLE_SRC_HEIGHT][ACC_SAMPLE_SRC_WIDTH]={
		{120, 95, 103, 1, 35},
		{50, 45, 61, 63, 38},
		{35, 76, 13, 98, 20},
		
	};
	
	int32_t result = -1;
		
	CameraCvSampleInitMat(src1, acc_sample_acc_buffer, ACC_SAMPLE_SRC_WIDTH, ACC_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_16SC1);
	CameraCvSampleInitMat(src2, acc_sample_add_buffer1, ACC_SAMPLE_SRC_WIDTH, ACC_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_8UC1); 
	CameraCvSampleInitMat(src3, acc_sample_add_buffer2, ACC_SAMPLE_SRC_WIDTH, ACC_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_8UC1); 

	CameraCvSampleShowMat(src1, "Src1");
	CameraCvSampleShowMat(src2, "Src2"); 	
	CameraCvSampleShowMat(src3, "Src3"); 		

	result = camera_cv_api_accumulate_square(&src1, &src2, &junk);

	if(!result)
	{
		CameraCvSampleShowMat(src1, "AddSqr once");
	}

	result = camera_cv_api_accumulate_square(&src1, &src3, &junk);

	if(!result)
	{
		CameraCvSampleShowMat(src1, "AddSqr twice");
	}

	if(g_AdvanceTest)
	{
		CAMERA_CV_API_ROI_ST rect_rd, rect_lu;
		CAMERA_CV_API_MAT_ST subSrc_rd, subSrc_lu;
		
		rect_lu.i_x = 0;
		rect_lu.i_y = 0;
		rect_lu.i_width = 3;
		rect_lu.i_height = 2;

		rect_rd.i_x = 2;
		rect_rd.i_y = 1;
		rect_rd.i_width = 3;
		rect_rd.i_height = 2;
		
		subSrc_rd = src1 & rect_rd; //region mask
		subSrc_lu = src2 & rect_lu; //region mask
		
		CameraCvSampleShowMat(subSrc_rd, "SubSrc buffer1");
		CameraCvSampleShowMat(subSrc_lu, "SubSrc buffer2");
		
		result = camera_cv_api_accumulate_square(&subSrc_rd, &subSrc_lu, &junk);
		
		if(!result)
		{
			CameraCvSampleShowMat(src1, "Src 1");
			CameraCvSampleShowMat(src2, "Src 2");
		}
	}
	
	return result;	
}

int CameraCvSampleAccWeight(void)
{
	CAMERA_CV_API_MAT_ST src1;
	CAMERA_CV_API_MAT_ST src2;
	CAMERA_CV_API_MAT_ST src3;	
	CAMERA_CV_API_MAT_ST junk;		
	const int ACC_SAMPLE_SRC_WIDTH = 5;
	const int ACC_SAMPLE_SRC_HEIGHT = 3;			
	uint8_t acc_sample_acc_buffer[ACC_SAMPLE_SRC_HEIGHT][ACC_SAMPLE_SRC_WIDTH]={
		{58, 22, 117, 28, 60},
		{0, 25, 203, 81, 99},
		{55, 66, 51, 153, 68},

	};

	uint8_t acc_sample_add_buffer1[ACC_SAMPLE_SRC_HEIGHT][ACC_SAMPLE_SRC_WIDTH]={
		{35, 76, 13, 98, 20},
		{120, 95, 103, 1, 35},
		{50, 45, 61, 63, 38},

	};

	uint8_t acc_sample_add_buffer2[ACC_SAMPLE_SRC_HEIGHT][ACC_SAMPLE_SRC_WIDTH]={
		{120, 95, 103, 1, 35},
		{50, 45, 61, 63, 38},
		{35, 76, 13, 98, 20},
		
	};
	
	int32_t result = -1;
		
	CameraCvSampleInitMat(src1, acc_sample_acc_buffer, ACC_SAMPLE_SRC_WIDTH, ACC_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_8UC1);
	CameraCvSampleInitMat(src2, acc_sample_add_buffer1, ACC_SAMPLE_SRC_WIDTH, ACC_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_8UC1); 
	CameraCvSampleInitMat(src3, acc_sample_add_buffer2, ACC_SAMPLE_SRC_WIDTH, ACC_SAMPLE_SRC_HEIGHT, CAMERA_CV_DATA_TYPE_8UC1); 

	CameraCvSampleShowMat(src1, "Src1");
	CameraCvSampleShowMat(src2, "Src2"); 	
	CameraCvSampleShowMat(src3, "Src3"); 		

	result = camera_cv_api_accumulate_weighted(&src1, &src2, &junk, 1.0f/2.0f);

	if(!result)
	{
		CameraCvSampleShowMat(src1, "Add Weight once");
	}

	result = camera_cv_api_accumulate_weighted(&src1, &src3, &junk, 1.0f/3.0f);

	if(!result)
	{
		CameraCvSampleShowMat(src1, "Add Weight twice");
	}
	
	if(g_AdvanceTest)
	{
		CAMERA_CV_API_ROI_ST rect_rd, rect_lu;
		CAMERA_CV_API_MAT_ST subSrc_rd, subSrc_lu;
		
		rect_lu.i_x = 0;
		rect_lu.i_y = 0;
		rect_lu.i_width = 3;
		rect_lu.i_height = 2;

		rect_rd.i_x = 2;
		rect_rd.i_y = 1;
		rect_rd.i_width = 3;
		rect_rd.i_height = 2;
		
		subSrc_rd = src1 & rect_rd; //region mask
		subSrc_lu = src2 & rect_lu; //region mask
		
		CameraCvSampleShowMat(subSrc_rd, "SubSrc buffer1");
		CameraCvSampleShowMat(subSrc_lu, "SubSrc buffer2");
		
		result = camera_cv_api_accumulate_weighted(&subSrc_rd, &subSrc_lu, &junk, 0.5f);
		
		if(!result)
		{
			CameraCvSampleShowMat(src1, "Src 1");
			CameraCvSampleShowMat(src2, "Src 2");
		}
	}

	return result;	
}

typedef enum{
	COLOR_BASE_I422 = 0x01,
	COLOR_BASE_IYUV = 0x02,		
	COLOR_BASE_NV12 = 0x03,
	COLOR_BASE_NV21 = 0x04,	
	COLOR_BASE_RGB = 0x05,
	COLOR_BASE_UYVY = 0x06,
	COLOR_BASE_YUV4 = 0x07,
	COLOR_BASE_YUYV = 0x08,
	COLOR_BASE_BGR = 0x09,	
	COLOR_BASE_GRAY = 0x0C,	
	COLOR_BASE_RGBPL = 0x15,
	COLOR_BASE_BGRPL = 0x19
}COLOR_BASE_T;

static float get_size_ratio(uint32_t ui_cv_color_base)
{
	switch(ui_cv_color_base)
	{
 		case COLOR_BASE_I422:
		case COLOR_BASE_UYVY:
		case COLOR_BASE_YUYV:
			return 2.0f;
		case COLOR_BASE_IYUV:
		case COLOR_BASE_NV12:
		case COLOR_BASE_NV21:
			return 1.5f;
		case COLOR_BASE_RGB:
		case COLOR_BASE_YUV4:
		case COLOR_BASE_BGR:
		case COLOR_BASE_RGBPL:
		case COLOR_BASE_BGRPL:
			return 3.0f;
		case COLOR_BASE_GRAY:
			return 1.0f;
		default:
			printf("Wrong color base %X\n", ui_cv_color_base);
			return 0.0;
	}
}

int CameraCvSampleCvtColor(void *p)
{
	//index width height code
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_MAT_ST dst;	
	int *ParamList = (int*)p;
	int index = ParamList[0];
	int width = ParamList[1];
	int height = ParamList[2];
	int code = ParamList[3];

	if(index < MAX_FILE_BUFFER_COUNT && index>=0)
		CameraCvSampleInitMat(src, g_FileBuffer[index], width, height);
	else
		CameraCvSampleInitMat(src, g_SampleBuffer, width, height);
	
	int32_t result = camera_cv_api_cvt_color(&src, &dst, (CAMERA_CV_API_CONVERT_CODE_ET)code);

	printf("Convert Result = %d\n", result);	

	if(!result)
	{
		CameraCvSampleBufferWrite(CVTCOLOR_SAMPLE_RESULT_PATH, dst.pv_data, dst.s_size.i_width * dst.s_size.i_height * get_size_ratio(code & 0xFF));
		CameraCvSampleMemoryFree(dst);
	}
	return result;
}

int CameraCvSampleCvtJpeg(void)
{
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_JPEG_RESULT_ST dst;	
	CAMERA_CV_API_ROI_ST roi;
	CAMERA_CV_API_SIZE_ST dsize;
	roi.i_x = 0;
	roi.i_y = 0;
	roi.i_width = CVTJPEG_SAMPLE_IMG_SRC_WIDTH;
	roi.i_height = CVTJPEG_SAMPLE_IMG_SRC_HEIGHT;
	dsize.i_width = CVTJPEG_SAMPLE_IMG_SRC_WIDTH;
	dsize.i_height = CVTJPEG_SAMPLE_IMG_SRC_HEIGHT;

	CameraCvSampleInitMat(src, g_SampleBuffer, CVTJPEG_SAMPLE_IMG_SRC_WIDTH, CVTJPEG_SAMPLE_IMG_SRC_HEIGHT);
	int32_t result = camera_cv_api_sub_img_cvt_jpeg_and_resize(&src, &dst, CAMERA_CV_CONVERT_JPEG_NV12_2_JPEG, CAMERA_CV_JPEG_QUALITY_MEDIAN, &roi, &dsize, 1);
	
	printf("Convert Result = %d\n", result);	

	if(!result)
	{
		CameraCvSampleBufferWrite(CVTJPEG_SAMPLE_RESULT_PATH, dst.pv_data, dst.i_data_size);
		CameraCvSampleMemoryFree(dst);
	}
	return result;
}

int CameraCvSampleCvtJpeg_Gray(void)
{
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_MAT_ST src_arr[4] = {0};
	CAMERA_CV_API_MAT_ST rgbMat;
	CAMERA_CV_API_JPEG_RESULT_ST dst;	
	CAMERA_CV_API_ROI_ST roi;
	CAMERA_CV_API_SIZE_ST dsize;
	int32_t result = -1;
	roi.i_x = 0;
	roi.i_y = 0;
	roi.i_width = CVTJPEG_SAMPLE_IMG_SRC_WIDTH;
	roi.i_height = CVTJPEG_SAMPLE_IMG_SRC_HEIGHT;
	dsize.i_width = CVTJPEG_SAMPLE_IMG_SRC_WIDTH;
	dsize.i_height = CVTJPEG_SAMPLE_IMG_SRC_HEIGHT;

	CameraCvSampleInitMat(src, g_SampleBuffer, CVTJPEG_SAMPLE_IMG_SRC_WIDTH, CVTJPEG_SAMPLE_IMG_SRC_HEIGHT);
	
	src_arr[0] = src_arr[1] = src_arr[2] = src;
	result = camera_cv_api_merge(src_arr, 3, &rgbMat);
	
	printf("Merge Result = %d\n", result);	
	
	if(result)
	{
		return result;
	}
	
	result = camera_cv_api_sub_img_cvt_jpeg_and_resize(&rgbMat, &dst, CAMERA_CV_CONVERT_JPEG_RGB888Packed_2_JPEG, CAMERA_CV_JPEG_QUALITY_MEDIAN, &roi, &dsize, 1);
	
	CameraCvSampleMemoryFree(rgbMat);
	
	printf("Convert Result = %d\n", result);	

	if(!result)
	{
		CameraCvSampleBufferWrite(CVTJPEG_GRAY_SAMPLE_RESULT_PATH, dst.pv_data, dst.i_data_size);
		CameraCvSampleMemoryFree(dst);
	}
	return result;
}

int CameraCvSampleSubImageCvtColor(void *p)
{
	//index width height code
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_MAT_ST dst;	
	int *ParamList = (int*)p;
	int index = ParamList[0];
	int width = ParamList[1];
	int height = ParamList[2];
	int roi_x = ParamList[3];
	int roi_y = ParamList[4];
	int roi_w = ParamList[5];
	int roi_h = ParamList[6];
	int code = ParamList[7];

	char output_filename[128];
	CAMERA_CV_API_ROI_ST roi;
	CAMERA_CV_API_CONVERT_CODE_ET e_code = (CAMERA_CV_API_CONVERT_CODE_ET)code;
	snprintf(output_filename, sizeof(output_filename),"/mnt/sd/TestData/output/%dx%d_%d_%d_%X_%d.yuv", roi_w, roi_h, roi_x, roi_y, code, index+1);
	roi.i_x = roi_x;
	roi.i_y = roi_y;
	roi.i_width = roi_w;
	roi.i_height = roi_h;

	if(index < MAX_FILE_BUFFER_COUNT && index>=0)
		CameraCvSampleInitMat(src, g_FileBuffer[index], width, height);
	else if(index == -1)
		CameraCvSampleInitMat(src, g_FrameBuffer[0], width, height);
	else
		CameraCvSampleInitMat(src, g_SampleBuffer, width, height);
	
	int32_t result = camera_cv_api_sub_img_cvt_color(&src, &dst, &e_code, &roi, 1);

	printf("Convert Result = %d\n", result);	

	if(!result)
	{
		CameraCvSampleBufferWrite(output_filename, dst.pv_data, dst.s_size.i_width * dst.s_size.i_height * get_size_ratio(code & 0xFF));
		CameraCvSampleMemoryFree(dst);
	}
	return result;
}

int CameraCvSampleMultipleSubImageCvtColor(void *p)
{
	//index width height code
	const int output_count = 16;
	const int roi_size = 400;
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_MAT_ST dst[output_count];	
	int *ParamList = (int*)p;
	int index = ParamList[0];
	int width = ParamList[1];
	int height = ParamList[2];
	int code = ParamList[3];

	char output_filename[128];
	CAMERA_CV_API_ROI_ST rois[output_count];
	CAMERA_CV_API_CONVERT_CODE_ET e_codes[output_count];
	int x[output_count]={0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400, 1500};
	int y[output_count]={0, 50, 100, 150, 200, 250, 300, 350, 400, 450, 500, 550, 600, 650, 0, 50};

	for(int i=0;i<output_count;i++)
	{
		e_codes[i] = (CAMERA_CV_API_CONVERT_CODE_ET)code;
		rois[i].i_height = roi_size;
		rois[i].i_width = roi_size;
		rois[i].i_x = x[i];
		rois[i].i_y = y[i];
	}

	if(index < MAX_FILE_BUFFER_COUNT && index>=0)
		CameraCvSampleInitMat(src, g_FileBuffer[index], width, height);
	else if(index == -1)
		CameraCvSampleInitMat(src, g_FrameBuffer[0], width, height);
	else
		CameraCvSampleInitMat(src, g_SampleBuffer, width, height);

	int start = CameraCvSamplePerfGetCurrent();
	int32_t result = camera_cv_api_sub_img_cvt_color(&src, dst, e_codes, rois, output_count);
	int end = CameraCvSamplePerfGetCurrent();

	printf("Convert Result = %d Diff = %d\n", result, end-start);	

	if(!result)
	{
		for(int i=0;i<output_count;i++)
		{
			snprintf(output_filename, sizeof(output_filename),"/mnt/sd/TestData/output/%dx%d_%d_%d_%X_%d.yuv", roi_size, roi_size, x[i], y[i], code, index+1);
			CameraCvSampleBufferWrite(output_filename, dst[i].pv_data, dst[i].s_size.i_width * dst[i].s_size.i_height * get_size_ratio(code & 0xFF));
			CameraCvSampleMemoryFree(dst[i]);
		}
	}
	
	return result;
}

int CameraCvSampleSubImageCvtColorAndResize(void *p)
{
	//index width height code
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_MAT_ST dst;	
	int *ParamList = (int*)p;
	int index = ParamList[0];
	int width = ParamList[1];
	int height = ParamList[2];
	int roi_x = ParamList[3];
	int roi_y = ParamList[4];
	int roi_w = ParamList[5];
	int roi_h = ParamList[6];
	int d_width = ParamList[7];
	int d_height = ParamList[8];
	int code = ParamList[9];

	char output_filename[128];
	CAMERA_CV_API_ROI_ST roi;
	CAMERA_CV_API_CONVERT_CODE_ET e_code = (CAMERA_CV_API_CONVERT_CODE_ET)code;
	CAMERA_CV_API_SIZE_ST dsize;
	snprintf(output_filename, sizeof(output_filename),"/mnt/sd/TestData/output/%dx%d_%d_%d_%d_%d_%X_%d.rgb", d_width, d_height, roi_w, roi_h, roi_x, roi_y, code, index+1);
	roi.i_x = roi_x;
	roi.i_y = roi_y;
	roi.i_width = roi_w;
	roi.i_height = roi_h;
	dsize.i_width = d_width;
	dsize.i_height = d_height;

	if(index < MAX_FILE_BUFFER_COUNT && index>=0)
		CameraCvSampleInitMat(src, g_FileBuffer[index], width, height);
	else if(index == -1)
		CameraCvSampleInitMat(src, g_FrameBuffer[0], width, height);
	else
		CameraCvSampleInitMat(src, g_SampleBuffer, width, height);
	
	int32_t result = camera_cv_api_sub_img_cvt_color_and_resize(&src, &dst, &e_code, &roi, &dsize, 1);

	printf("Convert Result = %d\n", result);	

	if(!result)
	{
		CameraCvSampleBufferWrite(output_filename, dst.pv_data, dst.s_size.i_width * dst.s_size.i_height * get_size_ratio(code & 0xFF));
		CameraCvSampleMemoryFree(dst);
	}
	return result;
}

int CameraCvSampleSubImageCvtJpeg(void *p)
{
	//index width height code
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_JPEG_RESULT_ST dst;	
	int *ParamList = (int*)p;
	int index = ParamList[0];
	int width = ParamList[1];
	int height = ParamList[2];
	int roi_x = ParamList[3];
	int roi_y = ParamList[4];
	int roi_w = ParamList[5];
	int roi_h = ParamList[6];
	int d_width = ParamList[7];
	int d_height = ParamList[8];
	int code = ParamList[9];

	char output_filename[128];
	CAMERA_CV_API_ROI_ST roi;
	CAMERA_CV_API_CONVERT_JPEG_CODE_ET e_code = (CAMERA_CV_API_CONVERT_JPEG_CODE_ET)code;
	CAMERA_CV_API_SIZE_ST dsize;
	snprintf(output_filename, sizeof(output_filename),"/mnt/sd/TestData/output/%dx%d_%d_%d_%d_%d_%X_%d.jpg", d_width, d_height, roi_w, roi_h, roi_x, roi_y, code, index+1);
	roi.i_x = roi_x;
	roi.i_y = roi_y;
	roi.i_width = roi_w;
	roi.i_height = roi_h;
	dsize.i_width = d_width;
	dsize.i_height = d_height;

	if(index < MAX_FILE_BUFFER_COUNT && index>=0)
		CameraCvSampleInitMat(src, g_FileBuffer[index], width, height);
	else if(index == -1)
		CameraCvSampleInitMat(src, g_FrameBuffer[0], width, height);
	else
		CameraCvSampleInitMat(src, g_SampleBuffer, width, height);
	
	printf("Start Convert\n");	

	int start = CameraCvSamplePerfGetCurrent();
	
	int32_t result = camera_cv_api_sub_img_cvt_jpeg_and_resize(&src, &dst, e_code, CAMERA_CV_JPEG_QUALITY_MEDIAN, &roi, &dsize, 1);

	int end = CameraCvSamplePerfGetCurrent();

	printf("Convert Result = %d diff = %d\n", result, end - start);	

	if(!result)
	{
		CameraCvSampleBufferWrite(output_filename, dst.pv_data, dst.i_data_size);
		printf("Save Jpeg :%s\n", output_filename);
		CameraCvSampleMemoryFree(dst);
	}
	return result;
}


int CameraCvSampleWarpAffine(void *p)
{
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_MAT_ST dst;	
	CAMERA_CV_API_MAT_ST transform;
	CAMERA_CV_API_SIZE_ST dsize;
	float AffineTransformation[2][3] = {0};

	float *ParamList = (float*)p;
	float *pAffine = (float*)AffineTransformation;
	int index = ParamList[0];
	int SrcWidth = ParamList[1];
	int SrcHeight = ParamList[2];
	int DstWidth = ParamList[3];
	int DstHeight = ParamList[4];
	int Channel = ParamList[5];
	for(int idx = 0;idx < 6; idx++)
		*pAffine++ = ParamList[6+ idx];
	
	int Type = CAMERA_CV_DATA_TYPE_8UC1;
	switch(Channel)
	{
		default:
		case 1:
			Type = CAMERA_CV_DATA_TYPE_8UC1;
			break;
		case 2:
			Type = CAMERA_CV_DATA_TYPE_8UC2;
			break;
		case 3:
			Type = CAMERA_CV_DATA_TYPE_8UC3;
			break;
	}

	if(index < MAX_FILE_BUFFER_COUNT && index>=0)
		CameraCvSampleInitMat(src, g_FileBuffer[index], SrcWidth, SrcHeight, Type);
	else if(index == -1)
		CameraCvSampleInitMat(src, g_FrameBuffer[0], SrcWidth, SrcHeight, Type);
	else
		CameraCvSampleInitMat(src, g_SampleBuffer, SrcWidth, SrcHeight, Type);

	CameraCvSampleInitMat(transform, AffineTransformation, 3, 2, CAMERA_CV_DATA_TYPE_32FC1);	
	dsize.i_width = DstWidth;
	dsize.i_height = DstHeight;

	int32_t result = camera_cv_api_warp_affine(&src, &dst, &transform, &dsize);

	if(!result)
	{
		char filename[128];
		snprintf(filename, sizeof(filename), "/mnt/sd/TestData/output/warp_%dx%d_%dx%d.raw", DstWidth, DstHeight, SrcWidth, SrcHeight);
		CameraCvSampleImwrite(filename, dst);
		CameraCvSampleMemoryFree(dst);
	}
	
	return result;
}

int CameraCvSampleResize(void *p)
{
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_MAT_ST dst;	
	CAMERA_CV_API_SIZE_ST dsize;
	int *ParamList = (int*)p;
	int index = ParamList[0];
	int SrcWidth = ParamList[1];
	int SrcHeight = ParamList[2];
	int DstWidth = ParamList[3];
	int DstHeight = ParamList[4];
	int Channel = ParamList[5];

	int type = CAMERA_CV_DATA_TYPE_8UC1 & ((Channel -1)<<3);

	if(index < MAX_FILE_BUFFER_COUNT && index>=0)
		CameraCvSampleInitMat(src, g_FileBuffer[index], SrcWidth, SrcHeight, type);
	else if(index == -1)
		CameraCvSampleInitMat(src, g_FrameBuffer[0], SrcWidth, SrcHeight, type);
	else
		CameraCvSampleInitMat(src, g_SampleBuffer, SrcWidth, SrcHeight, type);
	
	dsize.i_width = DstWidth;
	dsize.i_height = DstHeight;
	int32_t result = camera_cv_api_resize(&src, &dst, &dsize);
	
	printf("Resize Result = %d\n", result);	

	if(!result)
	{
		char filename[128];
		snprintf(filename, sizeof(filename), "/mnt/sd/TestData/output/resize_%dx%d_%dx%d_%X.raw", DstWidth, DstHeight, SrcWidth, SrcHeight, type);
		CameraCvSampleImwrite(filename, dst);
		CameraCvSampleMemoryFree(dst);
	}
	return result;

}

int CameraCvSampleMerge(void *p)
{
	CAMERA_CV_API_MAT_ST src[4];
	CAMERA_CV_API_MAT_ST dst;	
	int Index[4];
	
	int *ParamList = (int*)p;
	int Count = ParamList[0];
	int SrcWidth = ParamList[1];
	int SrcHeight = ParamList[2];
	for(int idx = 0;idx<Count;idx++)
	{
		Index[idx] = ParamList[3 + idx];
	}
	
	for(int idx=0;idx<Count;idx++)
	{
		int index = Index[idx];
		if(index < MAX_FILE_BUFFER_COUNT && index>=0)
			CameraCvSampleInitMat(src[idx], g_FileBuffer[index], SrcWidth, SrcHeight, CAMERA_CV_DATA_TYPE_8UC1);
		else if(index == -1)
			CameraCvSampleInitMat(src[idx], g_FrameBuffer[0], SrcWidth, SrcHeight, CAMERA_CV_DATA_TYPE_8UC1);
		else
			CameraCvSampleInitMat(src[idx], g_SampleBuffer, SrcWidth, SrcHeight, CAMERA_CV_DATA_TYPE_8UC1);	
	}

	int32_t result = camera_cv_api_merge(src, Count, &dst);

	if(!result)
	{
		char filename[128];
		snprintf(filename, sizeof(filename), "/mnt/sd/TestData/output/merge_%dx%d.raw", SrcWidth, SrcHeight);
		CameraCvSampleImwrite(filename, dst);
		CameraCvSampleMemoryFree(dst);
	}
	return result;
}

int CameraCvSampleConvertTo(void *p)
{
	CAMERA_CV_API_MAT_ST src;
	CAMERA_CV_API_MAT_ST dst;	

	int *ParamList = (int*)p;
	int Index = ParamList[0];
	int SrcWidth = ParamList[1];
	int SrcHeight = ParamList[2];
	int SrcElemntSize = ParamList[3];
	int DstElemntSize = ParamList[4];
	int ShiftVal = ParamList[5];

	float alpha = ShiftVal > 0 ? (float)(1<<ShiftVal) : 1.0 / (1 << (-ShiftVal));

	int srcType = SrcElemntSize == 1 ? CAMERA_CV_DATA_TYPE_8UC1:CAMERA_CV_DATA_TYPE_16SC1;
	CAMERA_CV_API_BIT_DEPTH_ET dstType = DstElemntSize == 1 ? CAMERA_CV_BITDEPTH_8U:CAMERA_CV_BITDEPTH_16S;
	
	if(Index < MAX_FILE_BUFFER_COUNT && Index>=0)
		CameraCvSampleInitMat(src, g_FileBuffer[Index], SrcWidth, SrcHeight, srcType);
	else if(Index == -1)
		CameraCvSampleInitMat(src, g_FrameBuffer[0], SrcWidth, SrcHeight, srcType);
	else
		CameraCvSampleInitMat(src, g_SampleBuffer, SrcWidth, SrcHeight, srcType);	
	
	CameraCvSampleShowMat(src, "Src Data");

	int32_t result = camera_cv_api_convert_to(&src, &dst, dstType, alpha);

	if(!result)
	{
		CameraCvSampleShowMat(dst, "After Convert To Result");
		CameraCvSampleMemoryFree(dst);
	}
	
	return result;
}

void CameraCvSampleTestCommand(int i_test_id, uint8_t *puc_parameter)
{
	if(i_test_id != 1314)
		printf("Test id = %d\n", i_test_id);
	
	switch(i_test_id)
	{
		case 1:
			CameraCvSampleInit();
			printf("CameraCvSampleInit done\n");
			break;
		case 2:
			CameraCvSampleDeinit();
			break;

		case 3:
		{
			int index;
			std::string filename;
			std::stringstream ss;
			printf("param=%s\n", puc_parameter);
			ss.str((const char*)puc_parameter);
			ss >> index >> filename;
			printf("Index = %d File = %s\n", index, filename.c_str());
			CameraCvSampleReadToFileBuffer(filename.c_str(), index);
			break;
		}
		
		case 4:
		{
			int enable = 0;
			sscanf((const char*)puc_parameter, "%d", &enable);
			g_CheckMatEnable = enable;
			break;
		}

		case 5:
		{
			int enable = 0;
			sscanf((const char*)puc_parameter, "%d", &enable);
			g_CheckPerfEnable = enable;
			break;
		}

		case 6:
		{
			int enable = 0;
			sscanf((const char*)puc_parameter, "%d", &enable);
			g_AdvanceTest = enable;
			break;
		}
		
		case 10:
			Invoke([](){
				int rst = CameraCvSampleWarpAffine();
				if(rst)
					printf("CameraCvSampleWarpAffine err %d\n", rst);
			});
			break;

		case 210:
		{
			/*
			int index = ParamList[0];
			int SrcWidth = ParamList[1];
			int SrcHeight = ParamList[2];
			int DstWidth = ParamList[3];
			int DstHeight = ParamList[4];
			int Channel = ParamList[5];
			for(int idx = 0;idx < 6; idx++)
				*pAffine++ = ParamList[6+ idx];
			*/
			float input[12] = {0};
			printf("command = %s\n", puc_parameter);
			if(sscanf((const char*)puc_parameter, "%f%f%f%f%f%f%f%f%f%f%f%f", 
				input, input+1, input+2, input+3, input+4, input+5,
				input+6, input+7, input+8, input+9, input+10, input+11
			)==12)
			{
				Invoke([](void *p){
					int rst = CameraCvSampleWarpAffine(p);
					if(rst)
						printf("CameraCvSampleWarpAffine err %d\n", rst);
					else
						printf("CameraCvSampleWarpAffine OK\n");
				}, input, sizeof(input));
			}
			else
			{
				printf("Wrong input number");
			}
			
			break;
		}

		case 11:
			Invoke([](){
				int rst = CameraCvSampleResize();
				if(rst)
					printf("CameraCvSampleResize err %d\n", rst);
			});
			break;

		case 211:
		{
			/*
			int index = ParamList[0];
			int SrcWidth = ParamList[1];
			int SrcHeight = ParamList[2];
			int DstWidth = ParamList[3];
			int DstHeight = ParamList[4];
			int Channel = ParamList[5];
			*/
			int input[6]={0};
			printf("command = %s\n", puc_parameter);
			if(sscanf((const char*)puc_parameter, "%d%d%d%d%d%d", 
				input, input+1, input+2, input+3, input+4, input+5)==6)
			{
				Invoke([](void *p){
					int rst = CameraCvSampleResize(p);
					if(rst)
						printf("CameraCvSampleResize err %d\n", rst);
					else
						printf("CameraCvSampleResize OK\n");
				}, input, sizeof(input));
			}
			else
			{
				printf("Wrong input number");
			}
			
			break;
		}

		case 212:
		{
			int sample_id = 0;
			if(sscanf((const char*)puc_parameter, "%d", &sample_id)==1)
			{
				Invoke([](void *p){
					int *p_sample_id = (int*)p;
					int rst = CameraCvSampleWarpPerspective(*p_sample_id);
					if(rst)
						printf("CameraCvSampleWarpPerspective err %d\n", rst);
				}, &sample_id, sizeof(sample_id));
			}
			else
			{
				printf("err: please enter your test sample id\n");
			}
			break;
		}
		
		case 13:
			Invoke([](){
				int rst = CameraCvSampleFeatureDetect("FAST");
				if(rst)
					printf("FAST err %d\n", rst);
			});
			break;

		case 14:
			Invoke([](){
				int rst = CameraCvSampleFeatureDetect("GFTT");
				if(rst)
					printf("GFTT err %d\n", rst);
			});
			break;

		case 15:
			Invoke([](){
				int rst = CameraCvSampleFeatureDetect("ORB");
				if(rst)
					printf("ORB err %d\n", rst);
			});
			break;
			
		case 16:
			Invoke([](){
				int rst = CameraCvSampleFeatureDetectAndCompute();
				if(rst)
					printf("ORB detect & compute err %d\n", rst);
			});
			break;
			
		case 17:
			Invoke([](){
				int rst = CameraCvSampleBFMatcher();
				if(rst)
					printf("CameraCvSampleBFMatcher err %d\n", rst);
			});
			break;

		case 18:
			Invoke([](){
				int rst = CameraCvSampleBFMatcherRadiusMatch();
				if(rst)
					printf("CameraCvSampleBFMatcher err %d\n", rst);
			});
			break;

		case 19:
			Invoke([](){
				int rst = CamerCvSampleCalcOpticalFlowPyrLk();
				if(rst)
					printf("CamerCvSampleCalcOpticalFlowPyrLk err %d\n", rst);
			});
			break;
			
		case 20:
			Invoke([](){
				int rst = CameraCvSampleCvtColor();
				if(rst)
					printf("CameraCvSampleCvtColor err %d\n", rst);
			});
			break;

		case 120:
		{
			int input[4]={0};
			uint32_t format = 0;
			
			if(sscanf((const char*)puc_parameter, "%d %d %d %x", input, input+1, input+2, &format)==4)
			{
				input[3] = (int)format;
				Invoke([](void *p){
					int rst = CameraCvSampleCvtColor(p);
					if(rst)
						printf("CameraCvSampleCvtColor err %d\n", rst);
				}, input, sizeof(input));
			}
			else
			{
				printf("Wrong input number");
			}
		}
			break;
		case 220:
		{
			/*
			int *ParamList = (int*)p;
			int index = ParamList[0];
			int width = ParamList[1];
			int height = ParamList[2];
			int roi_x = ParamList[3];
			int roi_y = ParamList[4];
			int roi_w = ParamList[5];
			int roi_h = ParamList[6];
			int code = ParamList[7];

			*/
			int input[8]={0};
			uint32_t format = 0;
			printf("command = %s\n", puc_parameter);
			if(sscanf((const char*)puc_parameter, "%d%d%d%d%d%d%d%x", 
				input, input+1, input+2, input+3,
				input+4, input+5, input+6, &format)==8)
			{
				input[7] = (int)format;
				Invoke([](void *p){
					int rst = CameraCvSampleSubImageCvtColor(p);
					if(rst)
						printf("CameraCvSampleSubImageCvtColor err %d\n", rst);
					else
						printf("CameraCvSampleSubImageCvtColor OK\n");
				}, input, sizeof(input));
			}
			else
			{
				printf("Wrong input number");
			}
		}
			break;

		case 320:
		{
			/*
			int *ParamList = (int*)p;
			int index = ParamList[0];
			int width = ParamList[1];
			int height = ParamList[2];
			int code = ParamList[3];

			*/
			int input[8]={0};
			uint32_t format = 0;
			printf("command = %s\n", puc_parameter);
			if(sscanf((const char*)puc_parameter, "%d%d%d%x", 
				input, input+1, input+2, &format)==4)
			{
				input[3] = (int)format;
				Invoke([](void *p){
					int rst = CameraCvSampleMultipleSubImageCvtColor(p);
					if(rst)
						printf("CameraCvSampleMultipleSubImageCvtColor err %d\n", rst);
					else
						printf("CameraCvSampleMultipleSubImageCvtColor OK\n");
				}, input, sizeof(input));
			}
			else
			{
				printf("Wrong input number");
			}
		}
			break;
		
		case 420:
		{
			
			Invoke([](){
				int input[10]={0};
				input[0] = -2;
				input[1] = 1920;
				input[2] = 1080;
				input[3] = 0;
				input[4] = 0;
				input[5] = 1024;
				input[6] = 1024;
				input[7] = 512;
				input[8] = 512;
				input[9] = CAMERA_CV_CONVERT_NV12_2_RGB888Packed;

				int rst = CameraCvSampleSubImageCvtColorAndResize(input);
				if(rst)
					printf("CameraCvSampleSubImageCvtColorAndResize err %d\n", rst);
				else
					printf("CameraCvSampleSubImageCvtColorAndResize OK\n");
			});
		
		}
			break;
			
		case 21:
			Invoke([](){
				int rst = CameraCvSampleMerge();
				if(rst)
					printf("CameraCvSampleMerge err %d\n", rst);
			});
			break;

		case 221:
		{
			/*
			int *ParamList = (int*)p;
			int Count = ParamList[0];
			int SrcWidth = ParamList[1];
			int SrcHeight = ParamList[2];
			for(int idx = 0;idx<Count;idx++)
			{
				Index[idx] = ParamList[3 + idx];
			}
		
			*/
			int input[7]={0};
			printf("command = %s\n", puc_parameter);
			int args = sscanf((const char*)puc_parameter, "%d%d%d%d%d%d%d", 
				input, input+1, input+2, input+3,
				input+4, input+5, input+6);

			if((args >= 3) && (args > input[0] + 3))
			{
				Invoke([](void *p){
					int rst = CameraCvSampleMerge(p);
					if(rst)
						printf("CameraCvSampleMerge err %d\n", rst);
					else
						printf("CameraCvSampleMerge OK\n");
				}, input, sizeof(input));
			}
			else
			{
				printf("Wrong input number");
			}
		}
			break;

		case 22:
			Invoke([](){
				int rst = CameraCvSampleConvertTo();
				if(rst)
					printf("CameraCvSampleConvertTo err %d\n", rst);
			});
			break;

		case 222:
		{
			/*
			int *ParamList = (int*)p;
			int Index = ParamList[0];
			int SrcWidth = ParamList[1];
			int SrcHeight = ParamList[2];
			int SrcElemntSize = ParamList[3];
			int DstElemntSize = ParamList[4];
			int ShiftVal = ParamList[5];

			*/
			int input[8]={0};
			printf("command = %s\n", puc_parameter);
			if(sscanf((const char*)puc_parameter, "%d%d%d%d%d%d", 
				input, input+1, input+2, input+3, input+4, input+5)==6)
			{
				Invoke([](void *p){
					int rst = CameraCvSampleConvertTo(p);
					if(rst)
						printf("CameraCvSampleConvertTo err %d\n", rst);
					else
						printf("CameraCvSampleConvertTo OK\n");
				}, input, sizeof(input));
			}
			else
			{
				printf("Wrong input number");
			}
		}
			break;

		case 24:
			Invoke([](){
				int rst = CameraCvSampleSplitAndMerge();
				if(rst)
					printf("CameraCvSampleSplitAndMerge err %d\n", rst);
			});
			break;

		case 25:
			Invoke([](){
				int rst = CameraCvSampleCvtJpeg();
				if(rst)
					printf("CameraCvSampleCvtJpeg err %d\n", rst);
			});
			break;
		
		case 125:
		{
			/*
			int *ParamList = (int*)p;
			int index = ParamList[0];
			int width = ParamList[1];
			int height = ParamList[2];
			int roi_x = ParamList[3];
			int roi_y = ParamList[4];
			int roi_w = ParamList[5];
			int roi_h = ParamList[6];
			int d_width = ParamList[7];
			int d_height = ParamList[8];
			int code = ParamList[9];
			*/
			int input[10]={0};
			uint32_t format = 0;
			printf("command = %s\n", puc_parameter);
			if(sscanf((const char*)puc_parameter, "%d%d%d%d%d%d%d%d%d%x", 
				input, input+1, input+2, input+3,
				input+4, input+5, input+6, input+7,
				input+8,&format)==10)
			{
				input[9] = (int)format;
				Invoke([](void *p){
					int rst = CameraCvSampleSubImageCvtJpeg(p);
					if(rst)
						printf("CameraCvSampleSubImageCvtJpeg err %d\n", rst);
					else
						printf("CameraCvSampleSubImageCvtJpeg OK\n");
				}, input, sizeof(input));
			}
			else
			{
				printf("Wrong input number");
			}
		}
			break;	

		case 225:
			Invoke([](){
				int rst = CameraCvSampleCvtJpeg_Gray();
				if(rst)
					printf("CameraCvSampleCvtJpeg_Gray err %d\n", rst);
			});
			break;
			
		case 26:
			Invoke([](){
				int rst = CameraCvSampleSplit();
				if(rst)
					printf("CameraCvSampleSplit err %d\n", rst);
			});
			break;
		
		case 27:
			Invoke([](){
				int rst = CameraCvSampleVConcat();
				if(rst)
					printf("CameraCvSampleVConcat err %d\n", rst);
			});
			break;
		
		case 28:
			Invoke([](){
				int rst = CameraCvSampleHConcat();
				if(rst)
					printf("CameraCvSampleHConcat err %d\n", rst);
			});
			break;

		case 30:
			Invoke([](){
				int rst = CameraCvSampleMeidanBlur();
				if(rst)
					printf("CameraCvSampleMeidanBlur err %d\n", rst);
			});
			break;

		case 31:
			Invoke([](){
				int rst = CameraCvSampleBoxFilter();
				if(rst)
					printf("CameraCvSampleBoxFilter err %d\n", rst);
			});
			break;

		case 32:
			Invoke([](){
				int rst = CameraCvSampleDilate();
				if(rst)
					printf("CameraCvSampleDilate err %d\n", rst);
			});
			break;

		case 33:
			Invoke([](){
				int rst = CameraCvSampleErode();
				if(rst)
					printf("CameraCvSampleErode err %d\n", rst);
			});
			break;

		case 34:
			Invoke([](){
				int rst = CameraCvSampleSobel();
				if(rst)
					printf("CameraCvSampleSobel err %d\n", rst);
			});
			break;

		case 40:
			Invoke([](){
				int rst = CameraCvSampleAbs();
				if(rst)
					printf("CameraCvSampleAbs err %d\n", rst);
			});
			break;

		case 41:
			Invoke([](){
				int rst = CameraCvSampleAbsDiff();
				if(rst)
					printf("CameraCvSampleAbsDiff err %d\n", rst);
			});
			break;

		case 42:
			Invoke([](){
				int rst = CameraCvSampleAdd();
				if(rst)
					printf("CameraCvSampleAdd err %d\n", rst);
			});
			break;

		case 43:
			Invoke([](){
				int rst = CameraCvSampleAddWeight();
				if(rst)
					printf("CameraCvSampleAdd err %d\n", rst);
			});
			break;

		case 44:
			Invoke([](){
				int rst = CameraCvSampleBitwiseOperation(OPERATION_AND);
				if(rst)
					printf("CameraCvSampleBitwiseAnd err %d\n", rst);
			});
			break;

		case 45:
			Invoke([](){
				int rst = CameraCvSampleBitwiseOperation(OPERATION_NOT);
				if(rst)
					printf("CameraCvSampleBitwiseNot err %d\n", rst);
			});
			break;
			
		case 46:
			Invoke([](){
				int rst = CameraCvSampleBitwiseOperation(OPERATION_OR);
				if(rst)
					printf("CameraCvSampleBitwiseOr err %d\n", rst);
			});
			break;

		case 47:
			Invoke([](){
				int rst = CameraCvSampleBitwiseOperation(OPERATION_XOR);
				if(rst)
					printf("CameraCvSampleBitwiseXor err %d\n", rst);
			});
			break;

		case 48:
			Invoke([](){
				int rst = CameraCvSampleCountNonZero();
				if(rst)
					printf("CameraCvSampleBitwiseXor err %d\n", rst);
			});
			break;

		case 49:
			Invoke([](){
				int rst = CameraCvSampleExp();
				if(rst)
					printf("CameraCvSampleExp err %d\n", rst);
			});
			break;

		case 50:
			Invoke([](){
				int rst = CameraCvSampleLut();
				if(rst)
					printf("CameraCvSampleLut err %d\n", rst);
			});
			break;
			
		case 51:
			Invoke([](){
				int rst = CameraCvSampleMagnitude();
				if(rst)
					printf("CameraCvSampleMagnitude err %d\n", rst);
			});
			break;

		case 52:
			Invoke([](){
				int rst = CameraCvSampleMinMax(true);
				if(rst)
					printf("CameraCvSampleMin err %d\n", rst);
			});
			break;

		case 53:
			Invoke([](){
				int rst = CameraCvSampleMinMax(false);
				if(rst)
					printf("CameraCvSampleMin err %d\n", rst);
			});
			break;

		case 54:
			Invoke([](){
				int rst = CameraCvSampleMinMaxLoc();
				if(rst)
					printf("CameraCvSampleMinMaxLoc err %d\n", rst);
			});
			break;

		case 55:
			Invoke([](){
				int rst = CameraCvSampleMultiply();
				if(rst)
					printf("CameraCvSampleMultiply err %d\n", rst);
			});
			break;

		case 56:
			Invoke([](){
				int rst = CameraCvSamplePhase();
				if(rst)
					printf("CameraCvSamplePhase err %d\n", rst);
			});
			break;

		case 57:
			Invoke([](){
				int rst = CameraCvSampleScaleAdd();
				if(rst)
					printf("CameraCvSampleScaleAdd err %d\n", rst);
			});
			break;
			
		case 58:
			Invoke([](){
				int rst = CameraCvSampleSort();
				if(rst)
					printf("CameraCvSampleSort err %d\n", rst);
			});
			break;
			
		case 59:
			Invoke([](){
				int rst = CameraCvSampleSubtract();
				if(rst)
					printf("CameraCvSampleSubtract err %d\n", rst);
			});
			break;

		case 60:
			Invoke([](){
				int rst = CameraCvSampleSum();
				if(rst)
					printf("CameraCvSampleSum err %d\n", rst);
			});
			break;

		case 61:
			Invoke([](){
				int rst = CameraCvSampleAcc();
				if(rst)
					printf("CameraCvSampleAcc err %d\n", rst);
			});
			break;
		
		case 62:
			Invoke([](){
				int rst = CameraCvSampleAccSquare();
				if(rst)
					printf("CameraCvSampleAccSquare err %d\n", rst);
			});
			break;

		case 63:
			Invoke([](){
				int rst = CameraCvSampleAccWeight();
				if(rst)
					printf("CameraCvSampleAccWeight err %d\n", rst);
			});
			break;
			
		case 101:
			Invoke([](){
				int rst = CameraCvSampleCvInit();
				printf("Init:%d\n", rst);
				rst = CameraCvSampleCvDeinit();
				printf("Deinit:%d\n", rst);
				usleep(10000);
				CameraCvSampleTestCommand(101, NULL);
			});
			break;
			
		case 102:
		{
			int times = 1;
			sscanf((const char*)puc_parameter, "%d", &times);
			Invoke([](void *p){
				int loop = *(int*)p;
				for(int i=0;i<loop;i++)
				{
					printf("Loop:%d\n", i);
					int rst = CameraCvSampleCvInit();
					printf("Init:%d\n", rst);
					rst = CameraCvSampleCvDeinit();
					printf("Deinit:%d\n", rst);
				}
			}, &times, sizeof(times));
			break;
		}
			
		case 201:
			Invoke([](){
				CameraHostAPI_StreamIn_GetVideoFrame_Res ResPara;
				auto popFrameResult = CameraCvSample_StreamIn_FramePop(&ResPara);
				printf("Pop Frame Result = %d\n", (int)popFrameResult);
			});
			break;
		
		case 1314:
		{
			int input[2];
			/*
			times = input[0]
			command = input[1]
			*/
			sscanf((const char*)puc_parameter, "%d%d", input, input + 1);
			Invoke([](void *p){
				int *input = (int*)p;
				int loop = input[0];
				int command = input[1];
				uint8_t buffer[32];
				printf("Remain Loop:%d\n", loop);
				CameraCvSampleTestCommand(command, (uint8_t*)"");
				loop--;
				if(loop > 0)
				{
					snprintf((char*)buffer, sizeof(buffer), "%d %d", loop, command);
					CameraCvSampleTestCommand(1314, buffer);
				}
			}, input, sizeof(input));
			break;
		}
			

	}
}

