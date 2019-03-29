/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

#ifndef EbSimpleAppContext_h
#define EbSimpleAppContext_h

#include "EbSvtAv1Enc.h"
#include <stdio.h>

typedef struct EbConfig
{
    /****************************************
     * File I/O
     ****************************************/
    FILE                        *config_file;
    FILE                        *input_file;
    FILE                        *bitstream_file;
    FILE                        *recon_file;
    FILE                        *error_log_file;
    FILE                        *buffer_file;

    FILE                        *qp_file;

    uint8_t                      use_qp_file;

    int64_t                      frame_rate;
    int64_t                      frame_rate_numerator;
    int64_t                      frame_rate_denominator;
    int64_t                      injector_frame_rate;
    uint32_t                     injector;
    uint32_t                     speed_control_flag;
    uint32_t                     encoder_bit_depth;
    uint32_t                     compressed_ten_bit_format;
    uint32_t                     source_width;
    uint32_t                     source_height;

    uint32_t                     input_padded_width;
    uint32_t                     input_padded_height;

    int64_t                      frames_to_be_encoded;
    int64_t                      frames_encoded;
    int64_t                      buffered_input;
    uint8_t                   **sequence_buffer;

    uint8_t                     latency_mode;

    /****************************************
     * // Interlaced Video
     ****************************************/
    uint8_t                     interlaced_video;
    uint8_t                     separate_fields;

    /*****************************************
     * Coding Structure
     *****************************************/
    uint32_t                    base_layer_switch_mode;
    uint8_t                     enc_mode;
    int64_t                     intra_period;
    uint32_t                    intra_refresh_type;
    uint32_t                    hierarchical_levels;
    uint32_t                    pred_structure;


    /****************************************
     * Quantization
     ****************************************/
    uint32_t                    qp;

    /****************************************
     * DLF
     ****************************************/
    uint8_t                     disable_dlf_flag;

    /****************************************
     * ME Tools
     ****************************************/
    uint8_t                     use_default_me_hme;
    uint8_t                     enable_hme_flag;
    uint8_t                     enable_hme_level0_flag;
    uint8_t                     enable_hme_level1_flag;
    uint8_t                     enable_hme_level2_flag;

    /****************************************
     * ME Parameters
     ****************************************/
    uint32_t                    search_area_width;
    uint32_t                    search_area_height;

    /****************************************
     * HME Parameters
     ****************************************/
    uint32_t                    number_hme_search_region_in_width ;
    uint32_t                    number_hme_search_region_in_height;
    uint32_t                    hme_level0_total_search_area_width;
    uint32_t                    hme_level0_total_search_area_height;
    uint32_t                    hme_level0_column_index;
    uint32_t                    hme_level0_row_index;
    uint32_t                    hme_level1_column_index;
    uint32_t                    hme_level1_row_index;
    uint32_t                    hme_level2_column_index;
    uint32_t                    hme_level2_row_index;
    uint32_t                    hme_level0_search_area_in_width_array[EB_HME_SEARCH_AREA_COLUMN_MAX_COUNT];
    uint32_t                    hme_level0_search_area_in_height_array[EB_HME_SEARCH_AREA_ROW_MAX_COUNT];
    uint32_t                    hme_level1_search_area_in_width_array[EB_HME_SEARCH_AREA_COLUMN_MAX_COUNT];
    uint32_t                    hme_level1_search_area_in_height_array[EB_HME_SEARCH_AREA_ROW_MAX_COUNT];
    uint32_t                    hme_level2_search_area_in_width_array[EB_HME_SEARCH_AREA_COLUMN_MAX_COUNT];
    uint32_t                    hme_level2_search_area_in_height_array[EB_HME_SEARCH_AREA_ROW_MAX_COUNT];

    /****************************************
     * MD Parameters
     ****************************************/
    uint8_t                     constrained_intra;

    /****************************************
     * Rate Control
     ****************************************/
    uint32_t                    scene_change_detection;
    uint32_t                    rate_control_mode;
    uint32_t                    look_ahead_distance;
    uint32_t                    target_bit_rate;
    uint32_t                    max_qp_allowed;
    uint32_t                    min_qp_allowed;

    /****************************************
    * TUNE
    ****************************************/
    uint8_t                     tune;

    /****************************************
     * Optional Features
     ****************************************/

    uint8_t                     bit_rate_reduction;
    uint8_t                     improve_sharpness;
    uint32_t                    video_usability_info;
    uint32_t                    high_dynamic_range_input;

    /****************************************
     * Annex A Parameters
     ****************************************/
    uint32_t                    profile;
    uint32_t                    tier;
    uint32_t                    level;

    /****************************************
     * On-the-fly Testing
     ****************************************/
    uint8_t                     eos_flag;

    /****************************************
    * Optimization Type
    ****************************************/
    uint32_t                    asm_type;

    // Channel info
    uint32_t                    channel_id;
    uint32_t                    active_channel_count;
    uint32_t                    logical_processors;
    int32_t                     target_socket;
    uint8_t                     stop_encoder;         // to signal CTRL+C Event, need to stop encoding.
} EbConfig;

/***************************************
 * App Callback data struct
 ***************************************/
typedef struct EbAppContext {
    EbSvtAv1EncConfiguration              eb_enc_parameters;

    // Component Handle
    EbComponentType*                   svt_encoder_handle;

    // Buffer Pools
    EbBufferHeaderType                 *input_picture_buffer;
    EbBufferHeaderType                 *output_stream_buffer;
    EbBufferHeaderType                 *recon_buffer;

    uint32_t instance_idx;

} EbAppContext;


/********************************
 * External Function
 ********************************/
extern EbErrorType eb_app_context_ctor(EbAppContext *contextPtr, EbConfig *config);
extern void eb_app_context_dtor(EbAppContext *contextPtr);
extern EbErrorType init_encoder(EbConfig *config, EbAppContext *callback_data, uint32_t instance_idx);
extern EbErrorType de_init_encoder(EbAppContext *callback_data_ptr, uint32_t instance_index);

#endif // EbAppContext_h
