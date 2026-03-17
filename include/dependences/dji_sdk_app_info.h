/**
 ********************************************************************
 * @file    dji_sdk_app_info.h
 * @brief   Public DJI SDK application info header.
 *
 * @copyright (c) 2018 DJI. All rights reserved.
 *
 * This checked-in header intentionally contains placeholder values only.
 * Real credentials should live in the local, git-ignored
 * "dji_sdk_app_info_local.h" file.
 *
 *********************************************************************
 */

#if defined(__has_include)
#if __has_include("dji_sdk_app_info_local.h")
#include "dji_sdk_app_info_local.h"
#endif
#endif

#ifndef DJI_SDK_APP_INFO_H
#define DJI_SDK_APP_INFO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/
/* Create your own DJI SDK application and fill real values in
 * include/dependences/dji_sdk_app_info_local.h.
 */
#define USER_APP_NAME               "your_app_name"
#define USER_APP_ID                 "your_app_id"
#define USER_APP_KEY                "your_app_key"
#define USER_APP_LICENSE            "your_app_license"
#define USER_DEVELOPER_ACCOUNT      "your_developer_account"
#define USER_BAUD_RATE              "your_baud_rate"

#ifdef __cplusplus
}
#endif

#endif  // DJI_SDK_APP_INFO_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
