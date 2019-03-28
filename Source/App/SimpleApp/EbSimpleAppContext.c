/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/
#include <stdlib.h>
#include "EbSimpleAppContext.h"
#define INPUT_SIZE_576p_TH                0x90000        // 0.58 Million
#define INPUT_SIZE_1080i_TH                0xB71B0        // 0.75 Million
#define INPUT_SIZE_1080p_TH                0x1AB3F0    // 1.75 Million
#define INPUT_SIZE_4K_TH                0x29F630    // 2.75 Million
#define EB_OUTPUTSTREAMBUFFERSIZE_MACRO(ResolutionSize)                ((ResolutionSize) < (INPUT_SIZE_1080i_TH) ? 0x1E8480 : (ResolutionSize) < (INPUT_SIZE_1080p_TH) ? 0x2DC6C0 : (ResolutionSize) < (INPUT_SIZE_4K_TH) ? 0x2DC6C0 : 0x2DC6C0  )
EbErrorType AllocateFrameBuffer(
    EbConfig        *config,
    uint8_t           *p_buffer)
{
    EbErrorType   return_error = EB_ErrorNone;
    const int32_t tenBitPackedMode = (config->encoder_bit_depth > 8) && (config->compressed_ten_bit_format == 0) ? 1 : 0;

    // Determine size of each plane
    const size_t luma8bitSize =
        config->input_padded_width    *
        config->input_padded_height   *
        (1 << tenBitPackedMode);

    const size_t chroma8bitSize = luma8bitSize >> 2;
    const size_t luma10bitSize = (config->encoder_bit_depth > 8 && tenBitPackedMode == 0) ? luma8bitSize : 0;
    const size_t chroma10bitSize = (config->encoder_bit_depth > 8 && tenBitPackedMode == 0) ? chroma8bitSize : 0;

    // Determine
    EbSvtIOFormat* inputPtr = (EbSvtIOFormat*)p_buffer;

    if (luma8bitSize) {
        inputPtr->luma = (uint8_t*)malloc(luma8bitSize);
        if (!inputPtr->luma) return EB_ErrorInsufficientResources;
    }
    else {
        inputPtr->luma = 0;
    }
    if (chroma8bitSize) {
        inputPtr->cb = (uint8_t*)malloc(chroma8bitSize);
        if (!inputPtr->cb) return EB_ErrorInsufficientResources;
    }
    else {
        inputPtr->cb = 0;
    }
    if (chroma8bitSize) {
        inputPtr->cr = (uint8_t*)malloc(chroma8bitSize);
        if (!inputPtr->cr) return EB_ErrorInsufficientResources;
    }
    else {
        inputPtr->cr = 0;
    }
    if (luma10bitSize) {
        inputPtr->luma_ext = (uint8_t*)malloc(luma10bitSize);
        if (!inputPtr->luma_ext) return EB_ErrorInsufficientResources;
    }
    else {
        inputPtr->luma_ext = 0;
    }
    if (chroma10bitSize) {
        inputPtr->cb_ext = (uint8_t*)malloc(chroma10bitSize);
        if (!inputPtr->cb_ext) return EB_ErrorInsufficientResources;
    }
    else {
        inputPtr->cb_ext = 0;
    }
    if (chroma10bitSize) {
        inputPtr->cr_ext = (uint8_t*)malloc(chroma10bitSize);
        if (!inputPtr->cr_ext) return EB_ErrorInsufficientResources;
    }
    else {
        inputPtr->cr_ext = 0;
    }
    return return_error;
}
/***********************************
 * AppContext Constructor
 ***********************************/
EbErrorType EbAppContextCtor(
    EbAppContext *contextPtr,
    EbConfig     *config)
{
    EbErrorType   return_error = EB_ErrorInsufficientResources;

    // Input Buffer
    contextPtr->inputPictureBuffer = (EbBufferHeaderType*)malloc(sizeof(EbBufferHeaderType));
    if (!contextPtr->inputPictureBuffer) return return_error;

    contextPtr->inputPictureBuffer->p_buffer = (uint8_t*)malloc(sizeof(EbSvtIOFormat));
    if (!contextPtr->inputPictureBuffer->p_buffer) return return_error;

    contextPtr->inputPictureBuffer->size = sizeof(EbBufferHeaderType);
    contextPtr->inputPictureBuffer->p_app_private = NULL;
    contextPtr->inputPictureBuffer->pic_type = EB_AV1_INVALID_PICTURE;
    // Allocate frame buffer for the p_buffer
    AllocateFrameBuffer(
        config,
        contextPtr->inputPictureBuffer->p_buffer);

    // output buffer
    contextPtr->outputStreamBuffer = (EbBufferHeaderType*)malloc(sizeof(EbBufferHeaderType));
    if (!contextPtr->outputStreamBuffer) return return_error;

    contextPtr->outputStreamBuffer->p_buffer = (uint8_t*)malloc(EB_OUTPUTSTREAMBUFFERSIZE_MACRO(config->source_width*config->source_height));
    if (!contextPtr->outputStreamBuffer->p_buffer) return return_error;

    contextPtr->outputStreamBuffer->size = sizeof(EbBufferHeaderType);
    contextPtr->outputStreamBuffer->n_alloc_len = EB_OUTPUTSTREAMBUFFERSIZE_MACRO(config->source_width*config->source_height);
    contextPtr->outputStreamBuffer->p_app_private = NULL;
    contextPtr->outputStreamBuffer->pic_type = EB_AV1_INVALID_PICTURE;

    // recon buffer
    if (config->recon_file) {
        contextPtr->recon_buffer = (EbBufferHeaderType*)malloc(sizeof(EbBufferHeaderType));
        if (!contextPtr->recon_buffer) return return_error;
        const size_t lumaSize =
            config->input_padded_width    *
            config->input_padded_height;
        // both u and v
        const size_t chromaSize = lumaSize >> 1;
        const size_t tenBit = (config->encoder_bit_depth > 8);
        const size_t frameSize = (lumaSize + chromaSize) << tenBit;

        // Initialize Header
        contextPtr->recon_buffer->size = sizeof(EbBufferHeaderType);

        contextPtr->recon_buffer->p_buffer = (uint8_t*)malloc(frameSize);
        if (!contextPtr->recon_buffer->p_buffer) return return_error;

        contextPtr->recon_buffer->n_alloc_len = (uint32_t)frameSize;
        contextPtr->recon_buffer->p_app_private = NULL;
    }
    else
        contextPtr->recon_buffer = NULL;
    return EB_ErrorNone;
}

/***********************************
 * AppContext Destructor
 ***********************************/
void EbAppContextDtor(
    EbAppContext *contextPtr)
{
    EbSvtIOFormat *inputPtr = (EbSvtIOFormat*)contextPtr->inputPictureBuffer->p_buffer;
    free(inputPtr->luma);
    free(inputPtr->cb);
    free(inputPtr->cr);
    free(inputPtr->luma_ext);
    free(inputPtr->cb_ext);
    free(inputPtr->cr_ext);
    free(contextPtr->inputPictureBuffer->p_buffer);
    free(contextPtr->outputStreamBuffer->p_buffer);
    free(contextPtr->inputPictureBuffer);
    free(contextPtr->outputStreamBuffer);
    if(contextPtr->recon_buffer)
        free(contextPtr->recon_buffer);
}

/***********************************************
* Copy configuration parameters from
*  The config structure, to the
*  callback structure to send to the library
***********************************************/
EbErrorType CopyConfigurationParameters(
    EbConfig                *config,
    EbAppContext            *callback_data,
    uint32_t                 instance_idx)
{
    EbErrorType   return_error = EB_ErrorNone;

    // Assign Instance index to the library
    callback_data->instance_idx = (uint8_t)instance_idx;

    // Initialize Port Activity Flags
    callback_data->eb_enc_parameters.source_width       = config->source_width;
    callback_data->eb_enc_parameters.source_height      = config->source_height;
    callback_data->eb_enc_parameters.encoder_bit_depth   = config->encoder_bit_depth;
    //callback_data->eb_enc_parameters.code_vps_sps_pps     = 1;
    //callback_data->eb_enc_parameters.code_eos_nal        = 1;
    callback_data->eb_enc_parameters.recon_enabled      = config->recon_file ? 1 : 0;

    return return_error;

}

/***********************************
 * Initialize Core & Component
 ***********************************/
EbErrorType init_encoder(
    EbConfig                *config,
    EbAppContext            *callback_data,
    uint32_t                 instance_idx)
{
    EbErrorType        return_error = EB_ErrorNone;

    ///************************* LIBRARY INIT [START] *********************///
    // STEP 1: Call the library to construct a Component Handle
    return_error = eb_init_handle(&callback_data->svt_encoder_handle, callback_data, &callback_data->eb_enc_parameters);
    if (return_error != EB_ErrorNone) {return return_error;}

    // STEP 3: Copy all configuration parameters into the callback structure
    return_error = CopyConfigurationParameters(config,callback_data,instance_idx);
    if (return_error != EB_ErrorNone) { return return_error; }

    // STEP 4: Send over all configuration parameters
    return_error = eb_svt_enc_set_parameter(callback_data->svt_encoder_handle,&callback_data->eb_enc_parameters);
    if (return_error != EB_ErrorNone) { return return_error; }

    // STEP 5: Init Encoder
    return_error = eb_init_encoder(callback_data->svt_encoder_handle);
    // Get ivf header
    if (config->bitstream_file) {
        EbBufferHeaderType *outputStreamBuffer;
        return_error = eb_svt_enc_stream_header(callback_data->svt_encoder_handle, &outputStreamBuffer);
        if (return_error != EB_ErrorNone) {
            return return_error;
        }
        fwrite(outputStreamBuffer->p_buffer, 1, outputStreamBuffer->n_filled_len, config->bitstream_file);
    }
    ///************************* LIBRARY INIT [END] *********************///
    return return_error;
}

/***********************************
 * Deinit Components
 ***********************************/
EbErrorType de_init_encoder(
    EbAppContext *callback_data_ptr,
    uint32_t        instance_index)
{
    (void)instance_index;
    EbErrorType return_error = EB_ErrorNone;

    if (((EbComponentType*)(callback_data_ptr->svt_encoder_handle)) != NULL) {
            return_error = eb_deinit_encoder(callback_data_ptr->svt_encoder_handle);
    }

    // Destruct the buffer memory pool
    if (return_error != EB_ErrorNone) { return return_error; }

    // Destruct the component
    eb_deinit_handle(callback_data_ptr->svt_encoder_handle);

    return return_error;
}
