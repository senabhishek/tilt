/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
*
* The information contained herein is property of Nordic Semiconductor ASA.
* Terms and conditions of usage are described in detail in NORDIC
* SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
*
* Licensees are granted free, non-transferable use of the information. NO
* WARRANTY of ANY KIND is provided. This heading must NOT be removed from
* the file.
*/

/**
* This file is autogenerated by nRFgo Studio 1.14.1.2369  
*/

#ifndef SETUP_MESSAGES_H__
#define SETUP_MESSAGES_H__

#include <hal_platform.h> 
#include <aci.h>
#define PIPE_GAP_DEVICE_NAME_SET 1
#define PIPE_UART_OVER_BTLE_UART_RX_RX 2
#define PIPE_UART_OVER_BTLE_UART_TX_TX 3
#define PIPE_DEVICE_INFORMATION_HARDWARE_REVISION_STRING_SET 4

#define NUMBER_OF_PIPES 4

#define SERVICES_PIPE_TYPE_MAPPING_CONTENT {\
  {ACI_STORE_LOCAL, ACI_SET},   \
  {ACI_STORE_LOCAL, ACI_RX},   \
  {ACI_STORE_LOCAL, ACI_TX},   \
  {ACI_STORE_LOCAL, ACI_SET},   \
}

#define GAP_PPCP_MAX_CONN_INT 0x12 /**< Maximum connection interval as a multiple of 1.25 msec , 0xFFFF means no specific value requested */
#define GAP_PPCP_MIN_CONN_INT  0x6 /**< Minimum connection interval as a multiple of 1.25 msec , 0xFFFF means no specific maximum*/
#define GAP_PPCP_SLAVE_LATENCY 0
#define GAP_PPCP_CONN_TIMEOUT 0xa /** Connection Supervision timeout multiplier as a multiple of 10msec, 0xFFFF means no specific value requested */

#define NB_SETUP_MESSAGES 21
#define SETUP_MESSAGES_CONTENT {\
    {0x00,\
        {\
            0x07,0x06,0x00,0x00,0x03,0x02,0x41,0xd7,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x04,0x01,0x01,0x00,0x00,0x06,0x00,0x00,\
            0x90,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x10,0x1c,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
            0x00,0x00,0x00,0x10,0x00,0x00,0x00,0x10,0x03,0x90,0x01,0xff,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x10,0x38,0xff,0xff,0x02,0x58,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x14,0x00,0x00,\
            0x00,0x10,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x05,0x06,0x10,0x54,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x00,0x04,0x04,0x02,0x02,0x00,0x01,0x28,0x00,0x01,0x00,0x18,0x04,0x04,0x05,0x05,0x00,\
            0x02,0x28,0x03,0x01,0x0e,0x03,0x00,0x00,0x2a,0x04,0x14,0x0a,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x1c,0x0a,0x00,0x03,0x2a,0x00,0x01,0x42,0x4c,0x45,0x20,0x53,0x68,0x69,0x65,0x6c,0x64,\
            0x04,0x04,0x05,0x05,0x00,0x04,0x28,0x03,0x01,0x02,0x05,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x38,0x01,0x2a,0x06,0x04,0x03,0x02,0x00,0x05,0x2a,0x01,0x01,0x00,0x00,0x04,0x04,0x05,\
            0x05,0x00,0x06,0x28,0x03,0x01,0x02,0x07,0x00,0x04,0x2a,0x06,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x54,0x04,0x09,0x08,0x00,0x07,0x2a,0x04,0x01,0x06,0x00,0x12,0x00,0x00,0x00,0x0a,0x00,\
            0x04,0x04,0x02,0x02,0x00,0x08,0x28,0x00,0x01,0x01,0x18,0x04,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x70,0x04,0x10,0x10,0x00,0x09,0x28,0x00,0x01,0x1e,0x94,0x8d,0xf1,0x48,0x31,0x94,0xba,\
            0x75,0x4c,0x3e,0x50,0x00,0x00,0x3d,0x71,0x04,0x04,0x13,0x13,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x8c,0x00,0x0a,0x28,0x03,0x01,0x04,0x0b,0x00,0x1e,0x94,0x8d,0xf1,0x48,0x31,0x94,0xba,\
            0x75,0x4c,0x3e,0x50,0x03,0x00,0x3d,0x71,0x44,0x10,0x14,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0xa8,0x00,0x0b,0x00,0x03,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
            0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x04,0x13,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0xc4,0x13,0x00,0x0c,0x28,0x03,0x01,0x10,0x0d,0x00,0x1e,0x94,0x8d,0xf1,0x48,0x31,0x94,\
            0xba,0x75,0x4c,0x3e,0x50,0x02,0x00,0x3d,0x71,0x14,0x00,0x14,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0xe0,0x00,0x00,0x0d,0x00,0x02,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
            0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x46,0x14,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0xfc,0x03,0x02,0x00,0x0e,0x29,0x02,0x01,0x00,0x00,0x04,0x04,0x02,0x02,0x00,0x0f,0x28,\
            0x00,0x01,0x0a,0x18,0x04,0x04,0x05,0x05,0x00,0x10,0x28,0x03,\
        },\
    },\
    {0x00,\
        {\
            0x1c,0x06,0x21,0x18,0x01,0x02,0x11,0x00,0x27,0x2a,0x04,0x04,0x09,0x01,0x00,0x11,0x2a,0x27,0x01,0x0a,\
            0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x40,0x00,0x2a,0x00,0x01,0x00,0x80,0x04,0x00,0x03,0x00,0x00,0x00,0x03,0x02,0x00,0x08,0x04,\
            0x00,0x0b,0x00,0x00,0x00,0x02,0x02,0x00,0x02,0x04,0x00,0x0d,\
        },\
    },\
    {0x00,\
        {\
            0x0f,0x06,0x40,0x1c,0x00,0x0e,0x2a,0x27,0x01,0x00,0x80,0x04,0x00,0x11,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x13,0x06,0x50,0x00,0x1e,0x94,0x8d,0xf1,0x48,0x31,0x94,0xba,0x75,0x4c,0x3e,0x50,0x00,0x00,0x3d,0x71,\
        },\
    },\
    {0x00,\
        {\
            0x0f,0x06,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x06,0x06,0xf0,0x00,0x03,0x6a,0x44,\
        },\
    },\
}

#endif
