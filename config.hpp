#ifndef _CONFIG_H_
#define _CONFIG_H_

#define  DISABLE_ABGRP_IMPROVEMENT   0 //If 1, would not use .abgrp to improve the flow

#define  PLUGIN_TYPE_STRING         "MYRIAD"
#define  MAIN_STREAM_FRAME_WIDTH    1920
#define  MAIN_STREAM_FRAME_HEIGHT   1080
#define  SUB_STREAM_FRAME_WIDTH     300
#define  SUB_STREAM_FRAME_HEIGHT    300
#define  STREAM_FRAME_TO_AI_WIDTH   416
#define  STREAM_FRAME_TO_AI_HEIGHT  416
#define  POP_FRAME_INTERVAL         10000 //us
#define  MAX_FDFR_RESULT            10
#define  ENABLE_XLINK_PIPELINE      0
#define  ENABLE_RTP_META_UPDATE     0
#define  MAX_RTP_PACKET_SIZE        (64*1024)

#define  THRESHOLD_LPD              0.05
#define  THRESHOLD_OCR              0.05
#define  ALIGN_WIDTH                MAIN_STREAM_FRAME_WIDTH / STREAM_FRAME_TO_AI_WIDTH
#define  ALIGN_HEIGHT               MAIN_STREAM_FRAME_HEIGHT / STREAM_FRAME_TO_AI_HEIGHT

#endif /* _CONFIG_H_ */