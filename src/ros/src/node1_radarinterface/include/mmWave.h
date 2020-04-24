
/*
 * mmWave.h
 *
 * This file contains various defines used within this package.
 *
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/


#ifndef _TI_IWR14XX_
#define _TI_IWR14XX_

#include <iostream>
#include <iostream>
#include <cstdio>
#include "serial/serial.h"
#include "ros/ros.h"
#include <boost/thread.hpp>
#include <cstdint>

enum MmwDemo_Output_TLV_Types
{ 
    MMWDEMO_OUTPUT_MSG_POINT_CLOUD_2D = 6,

    MMWDEMO_OUTPUT_MSG_TARGET_LIST_2D,

    MMWDEMO_OUTPUT_MSG_TARGET_INDEX,
    
    UART_MSG_MMW_SIDE_INFO
};

enum SorterState{ READ_HEADER, 
    CHECK_TLV_TYPE,
    READ_POINT_CLOUD_2D, 
    READ_TARGET_LIST_2D, 
    READ_TARGET_INDEX,
    READ_SIDE_INFO,
    SWAP_BUFFERS
};

struct MmwDemo_output_message_frameHeader_t
    {
        uint32_t    version;

        uint32_t    platform;

        uint32_t    timestamp;

        uint32_t    packetLength;
        
        uint32_t    frameNumber;

        uint32_t    subframeNumber;

        uint32_t    chirpMargin;

        uint32_t    frameMargin;

        uint32_t    uartSentTime;

        uint32_t    trackProcessTime;
        
        uint16_t    numTLVs;

        uint16_t    checksum;
    };

struct MmwDemo_output_message_tlvHeader_t
    {
        uint32_t   type;     /*!< @brief Range index */
        uint32_t   length;   /*!< @brief Dopler index */
    };
    

struct MmwDemo_output_message_PointCloud_t
    {
        float range;
        
        float angle;

        float elev;

        float doppler;
    };

struct MmwDemo_output_message_SideInfo_t
    {
        uint16_t snr;
        
        uint16_t noise;
    };

    struct MmwDemo_output_message_TargetList_t
    {
        uint32_t    tid;

        float posX;
        
        float posY;

        float velX;

        float velY;

        float accX;

        float accY;
                
        float posZ;

        float velZ;

        float accZ;
    };

    struct MmwDemo_output_message_TargetIndex_t
    {
        uint8_t    targetID;
    };

    struct mmwDataPacket
    {   
        MmwDemo_output_message_frameHeader_t frameHeader;
        
        MmwDemo_output_message_tlvHeader_t tlvHeader;
        
        MmwDemo_output_message_PointCloud_t PointCloudData;

        MmwDemo_output_message_SideInfo_t SideInfoData;
        
        MmwDemo_output_message_TargetList_t TargetListData; 

        MmwDemo_output_message_TargetIndex_t TargetIndexData; 
    };

const uint8_t magicWord[8] = {2, 1, 4, 3, 6, 5, 8, 7};

const uint8_t color_mat[255][3] = {
{255,	0	,	0  },
{0,	    255	,	0  },
{0,	    0	,	255 },
{255,	204	,	255 },
{0	,	255	,	255},
{255,	178	,	102},
{128,	0	,	255},
{255,	0	,	191},
{255,	6	,	0  },
{255,	197	,	0  },
{122,	255	,	0  },
{0	,	255	,	70 },
{0	,	249	,	255},
{0	,	58	,	255},
{133,	0	,	255},
{255,	0	,	185},
{255,	12	,	0  },
{255,	203	,	0  },
{116,	255	,	0  },
{0	,	255	,	76 },
{0	,	243	,	255},
{0	,	52	,	255},
{139,	0	,	255},
{255,	0	,	179},
{255,	18	,	0  },
{255,	209	,	0  },
{110,	255	,	0  },
{0	,	255	,	82 },
{0	,	237	,	255},
{0	,	46	,	255},
{145,	0	,	255},
{255,	0	,	173},
{255,	24	,	0  },
{255,	215	,	0  },
{104,	255	,	0  },
{0	,	255	,	88 },
{0	,	231	,	255},
{0	,	40	,	255},
{151,	0	,	255},
{255,	0	,	167},
{255,	30	,	0  },
{255,	221	,	0  },
{98	,	255	,	0  },
{0	,	255	,	94 },
{0	,	225	,	255},
{0	,	34	,	255},
{157,	0	,	255},
{255,	0	,	161},
{255,	36	,	0  },
{255,	227	,	0  },
{92	,	255	,	0  },
{0	,	255	,	100},
{0	,	219	,	255},
{0	,	28	,	255},
{163,	0	,	255},
{255,	0	,	155},
{255,	42	,	0  },
{255,	233	,	0  },
{86	,	255	,	0  },
{0	,	255	,	106},
{0	,	213	,	255},
{0	,	22	,	255},
{169,	0	,	255},
{255,	0	,	149},
{255,	48	,	0  },
{255,	239	,	0  },
{80	,	255	,	0  },
{0	,	255	,	112},
{0	,	207	,	255},
{0	,	16	,	255},
{175,	0	,	255},
{255,	0	,	143},
{255,	54	,	0  },
{255,	245	,	0  },
{74	,	255	,	0  },
{0	,	255	,	118},
{0	,	201	,	255},
{0	,	10	,	255},
{181,	0	,	255},
{255,	0	,	137},
{255,	60	,	0  },
{255,	251	,	0  },
{68	,	255	,	0  },
{0	,	255	,	124},
{0	,	195	,	255},
{0	,	4	,	255},
{187,	0	,	255},
{255,	0	,	131},
{255,	66	,	0  },
{253,	255	,	0  },
{62	,	255	,	0  },
{0	,	255	,	129},
{0	,	189	,	255},
{2	,	0	,	255},
{193,	0	,	255},
{255,	0	,	126},
{255,	72	,	0  },
{247,	255	,	0  },
{56	,	255	,	0  },
{0	,	255	,	135},
{0	,	183	,	255},
{8	,	0	,	255},
{199,	0	,	255},
{255,	0	,	120},
{255,	78	,	0  },
{241,	255	,	0  },
{50	,	255	,	0  },
{0	,	255	,	141},
{0	,	177	,	255},
{14	,	0	,	255},
{205,	0	,	255},
{255,	0	,	114},
{255,	84	,	0  },
{235,	255	,	0  },
{44	,	255	,	0  },
{0	,	255	,	147},
{0	,	171	,	255},
{20	,	0	,	255},
{211,	0	,	255},
{255,	0	,	108},
{255,	90	,	0  },
{229,	255	,	0  },
{38	,	255	,	0  },
{0	,	255	,	153},
{0	,	165	,	255},
{26	,	0	,	255},
{217,	0	,	255},
{255,	0	,	102},
{255,	96	,	0  },
{223,	255	,	0  },
{32	,	255	,	0  },
{0	,	255	,	159},
{0	,	159	,	255},
{32	,	0	,	255},
{223,	0	,	255},
{255,	0	,	96 },
{255,	102	,	0  },
{217,	255	,	0  },
{26	,	255	,	0  },
{0	,	255	,	165},
{0	,	153	,	255},
{38	,	0	,	255},
{229,	0	,	255},
{255,	0	,	90 },
{255,	108	,	0  },
{211,	255	,	0  },
{20	,	255	,	0  },
{0	,	255	,	171},
{0	,	147	,	255},
{44	,	0	,	255},
{235,	0	,	255},
{255,	0	,	84 },
{255,	114	,	0  },
{205,	255	,	0  },
{14	,	255	,	0  },
{0	,	255	,	177},
{0	,	141	,	255},
{50	,	0	,	255},
{241,	0	,	255},
{255,	0	,	78 },
{255,	120	,	0  },
{199,	255	,	0  },
{8	,	255	,	0  },
{0	,	255	,	183},
{0	,	135	,	255},
{56	,	0	,	255},
{247,	0	,	255},
{255,	0	,	72 },
{255,	126	,	0  },
{193,	255	,	0  },
{2	,	255	,	0  },
{0	,	255	,	189},
{0	,	129	,	255},
{62	,	0	,	255},
{253,	0	,	255},
{255,	0	,	66 },
{255,	131	,	0  },
{187,	255	,	0  },
{0	,	255	,	4  },
{0	,	255	,	195},
{0	,	124	,	255},
{68	,	0	,	255},
{255,	0	,	251},
{255,	0	,	60 },
{255,	137	,	0  },
{181,	255	,	0  },
{0	,	255	,	10 },
{0	,	255	,	201},
{0	,	118	,	255},
{74	,	0	,	255},
{255,	0	,	245},
{255,	0	,	54 },
{255,	143	,	0  },
{175,	255	,	0  },
{0	,	255	,	16 },
{0	,	255	,	207},
{0	,	112	,	255},
{80	,	0	,	255},
{255,	0	,	239},
{255,	0	,	48 },
{255,	149	,	0  },
{169,	255	,	0  },
{0	,	255	,	22 },
{0	,	255	,	213},
{0	,	106	,	255},
{86	,	0	,	255},
{255,	0	,	233},
{255,	0	,	42 },
{255,	155	,	0  },
{163,	255	,	0  },
{0	,	255	,	28 },
{0	,	255	,	219},
{0	,	100	,	255},
{92	,	0	,	255},
{255,	0	,	227},
{255,	0	,	36 },
{255,	161	,	0  },
{157,	255	,	0  },
{0	,	255	,	34 },
{0	,	255	,	225},
{0	,	94	,	255},
{98	,	0	,	255},
{255,	0	,	221},
{255,	0	,	30 },
{255,	167	,	0  },
{151,	255	,	0  },
{0	,	255	,	40 },
{0	,	255	,	231},
{0	,	88	,	255},
{104,	0	,	255},
{255,	0	,	215},
{255,	0	,	24 },
{255,	173	,	0  },
{145,	255	,	0  },
{0	,	255	,	46 },
{0	,	255	,	237},
{0	,	82	,	255},
{110,	0	,	255},
{255,	0	,	209},
{255,	0	,	18 },
{255,	179	,	0  },
{139,	255	,	0  },
{0	,	255	,	52 },
{0	,	255	,	243},
{0	,	76	,	255},
{116,	0	,	255},
{255,	0	,	203},
{255,	0	,	12 },
{255,	185	,	0  },
{133,	255	,	0  },
{0	,	255	,	58 },
{0	,	255	,	249},
{0	,	70	,	255},
{122,	0	,	255},
{255,	0	,	197}
};

#endif






