/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

#ifndef EbInitialRateControlReorderQueue_h
#define EbInitialRateControlReorderQueue_h

#include "EbDefinitions.h"
#include "EbSystemResourceManager.h"
#include "EbRateControlTables.h"
#include "EbPictureControlSet.h"

/************************************************
 * Initial Rate Control Reorder Queue Entry
 ************************************************/
typedef struct InitialRateControlReorderEntry_s {
    uint64_t                          picture_number;
    EbObjectWrapper              *parentPcsWrapperPtr;
} InitialRateControlReorderEntry_t;

extern EbErrorType InitialRateControlReorderEntryCtor(
    InitialRateControlReorderEntry_t   **entry_dbl_ptr,
    uint32_t                               picture_number);


/************************************************
 * High Level Rate Control Histogram Queue Entry
 ************************************************/
typedef struct HlRateControlHistogramEntry_s {
    uint64_t                          picture_number;
#if RC
    int16_t                          life_count;
    EbBool                         passed_to_hlrc;
    EbBool                         is_coded;
    uint64_t                       total_num_bits_coded;
#else
    int16_t                          lifeCount;
    EbBool                         passedToHlrc;
    EbBool                         isCoded;
    uint64_t                          totalNumBitsCoded;
#endif
    EbObjectWrapper              *parentPcsWrapperPtr;
    EbBool                         end_of_sequence_flag;
    uint64_t                          pred_bits_ref_qp[MAX_REF_QP_NUM];
    EB_SLICE                        slice_type;
    uint32_t                          temporal_layer_index;
    uint32_t                        frames_in_sw;


    // Motion Estimation Distortion and OIS Historgram
    uint16_t                         *me_distortion_histogram;
    uint16_t                         *ois_distortion_histogram;
    uint32_t                          full_sb_count;
} HlRateControlHistogramEntry_t;

extern EbErrorType HlRateControlHistogramEntryCtor(
    HlRateControlHistogramEntry_t   **entry_dbl_ptr,
    uint32_t                            picture_number);

#endif //EbInitialRateControlReorderQueue_h
