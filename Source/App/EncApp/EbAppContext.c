/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

/***************************************
 * Includes
 ***************************************/

#include <stdlib.h>

#include "EbAppContext.h"
#include "EbAppConfig.h"


#define INPUT_SIZE_576p_TH                0x90000        // 0.58 Million
#define INPUT_SIZE_1080i_TH                0xB71B0        // 0.75 Million
#define INPUT_SIZE_1080p_TH                0x1AB3F0    // 1.75 Million
#define INPUT_SIZE_4K_TH                0x29F630    // 2.75 Million

#define SIZE_OF_ONE_FRAME_IN_BYTES(width, height,is16bit) ( ( ((width)*(height)*3)>>1 )<<is16bit)
#define IS_16_BIT(bit_depth) (bit_depth==10?1:0)
#define EB_OUTPUTSTREAMBUFFERSIZE_MACRO(ResolutionSize)                ((ResolutionSize) < (INPUT_SIZE_1080i_TH) ? 0x1E8480 : (ResolutionSize) < (INPUT_SIZE_1080p_TH) ? 0x2DC6C0 : (ResolutionSize) < (INPUT_SIZE_4K_TH) ? 0x2DC6C0 : 0x2DC6C0  )

 /***************************************
 * Variables Defining a memory table
 *  hosting all allocated pointers
 ***************************************/
EbMemoryMapEntry                 *appMemoryMap;
uint32_t                         *appMemoryMapIndex;
uint64_t                         *totalAppMemory;
uint32_t                          appMallocCount = 0;
static EbMemoryMapEntry          *appMemoryMapAllChannels[MAX_CHANNEL_NUMBER];
static uint32_t                   appMemoryMapIndexAllChannels[MAX_CHANNEL_NUMBER];
static uint64_t                   appMemoryMallocdAllChannels[MAX_CHANNEL_NUMBER];

/***************************************
* Allocation and initializing a memory table
*  hosting all allocated pointers
***************************************/
void AllocateMemoryTable(
    uint32_t    instance_idx)
{
    // Malloc Memory Table for the instance @ instance_idx
    appMemoryMapAllChannels[instance_idx]        = (EbMemoryMapEntry*)malloc(sizeof(EbMemoryMapEntry) * MAX_APP_NUM_PTR);

    // Init the table index
    appMemoryMapIndexAllChannels[instance_idx]   = 0;

    // Size of the table
    appMemoryMallocdAllChannels[instance_idx]    = sizeof(EbMemoryMapEntry) * MAX_APP_NUM_PTR;
    totalAppMemory = &appMemoryMallocdAllChannels[instance_idx];

    // Set pointer to the first entry
    appMemoryMap                                = appMemoryMapAllChannels[instance_idx];

    // Set index to the first entry
    appMemoryMapIndex                           = &appMemoryMapIndexAllChannels[instance_idx];

    // Init Number of pointers
    appMallocCount = 0;

    return;
}


/*************************************
**************************************
*** Helper functions Input / Output **
**************************************
**************************************/
/******************************************************
* Copy fields from the stream to the input buffer
    Input   : stream
    Output  : valid input buffer
******************************************************/
void ProcessInputFieldBufferingMode(
    uint64_t                      processedFrameCount,
    int32_t                      *filledLen,
    FILE                         *inputFile,
    uint8_t                        *lumaInputPtr,
    uint8_t                        *cbInputPtr,
    uint8_t                        *crInputPtr,
    uint32_t                      inputPaddedWidth,
    uint32_t                      inputPaddedHeight,
    uint8_t                       is16bit) {


    uint64_t  sourceLumaRowSize   = (uint64_t)(inputPaddedWidth << is16bit);
    uint64_t  sourceChromaRowSize = sourceLumaRowSize >> 1;

    uint8_t  *ebInputPtr;
    uint32_t  inputRowIndex;

    // Y
    ebInputPtr = lumaInputPtr;
    // Skip 1 luma row if bottom field (point to the bottom field)
    if (processedFrameCount % 2 != 0)
        fseeko64(inputFile, (long)sourceLumaRowSize, SEEK_CUR);

    for (inputRowIndex = 0; inputRowIndex < inputPaddedHeight; inputRowIndex++) {

        *filledLen += (uint32_t)fread(ebInputPtr, 1, sourceLumaRowSize, inputFile);
        // Skip 1 luma row (only fields)
        fseeko64(inputFile, (long)sourceLumaRowSize, SEEK_CUR);
        ebInputPtr += sourceLumaRowSize;
    }

    // U
    ebInputPtr = cbInputPtr;
    // Step back 1 luma row if bottom field (undo the previous jump), and skip 1 chroma row if bottom field (point to the bottom field)
    if (processedFrameCount % 2 != 0) {
        fseeko64(inputFile, -(long)sourceLumaRowSize, SEEK_CUR);
        fseeko64(inputFile, (long)sourceChromaRowSize, SEEK_CUR);
    }

    for (inputRowIndex = 0; inputRowIndex < inputPaddedHeight >> 1; inputRowIndex++) {

        *filledLen += (uint32_t)fread(ebInputPtr, 1, sourceChromaRowSize, inputFile);
        // Skip 1 chroma row (only fields)
        fseeko64(inputFile, (long)sourceChromaRowSize, SEEK_CUR);
        ebInputPtr += sourceChromaRowSize;
    }

    // V
    ebInputPtr = crInputPtr;
    // Step back 1 chroma row if bottom field (undo the previous jump), and skip 1 chroma row if bottom field (point to the bottom field)
    // => no action


    for (inputRowIndex = 0; inputRowIndex < inputPaddedHeight >> 1; inputRowIndex++) {

        *filledLen += (uint32_t)fread(ebInputPtr, 1, sourceChromaRowSize, inputFile);
        // Skip 1 chroma row (only fields)
        fseeko64(inputFile, (long)sourceChromaRowSize, SEEK_CUR);
        ebInputPtr += sourceChromaRowSize;
    }

    // Step back 1 chroma row if bottom field (undo the previous jump)
    if (processedFrameCount % 2 != 0) {
        fseeko64(inputFile, -(long)sourceChromaRowSize, SEEK_CUR);
    }
}


/***********************************************
* Copy configuration parameters from
*  The config structure, to the
*  callback structure to send to the library
***********************************************/
EbErrorType CopyConfigurationParameters(
    EbConfig_t                *config,
    EbAppContext            *callback_data,
    uint32_t                 instance_idx)
{
    EbErrorType   return_error = EB_ErrorNone;
    uint32_t         hmeRegionIndex;

    // Assign Instance index to the library
    callback_data->instance_idx = (uint8_t)instance_idx;

    // Initialize Port Activity Flags
    callback_data->output_stream_port_active = APP_PortActive;
    callback_data->eb_enc_parameters.source_width = config->sourceWidth;
    callback_data->eb_enc_parameters.source_height = config->sourceHeight;
    callback_data->eb_enc_parameters.intra_period_length = config->intraPeriod;
    callback_data->eb_enc_parameters.intra_refresh_type = config->intraRefreshType;
    callback_data->eb_enc_parameters.base_layer_switch_mode = config->base_layer_switch_mode;
    callback_data->eb_enc_parameters.enc_mode = (EbBool)config->encMode;
    callback_data->eb_enc_parameters.frame_rate = config->frameRate;
    callback_data->eb_enc_parameters.frame_rate_denominator = config->frameRateDenominator;
    callback_data->eb_enc_parameters.frame_rate_numerator = config->frameRateNumerator;
    callback_data->eb_enc_parameters.hierarchical_levels = config->hierarchicalLevels;
    callback_data->eb_enc_parameters.pred_structure = (uint8_t)config->predStructure;
    callback_data->eb_enc_parameters.in_loop_me_flag = config->in_loop_me_flag;
    callback_data->eb_enc_parameters.ext_block_flag = config->ext_block_flag;
    callback_data->eb_enc_parameters.tile_rows = config->tile_rows;
    callback_data->eb_enc_parameters.tile_columns = config->tile_columns;

    callback_data->eb_enc_parameters.scene_change_detection = config->scene_change_detection;
    callback_data->eb_enc_parameters.look_ahead_distance = config->look_ahead_distance;
    callback_data->eb_enc_parameters.frames_to_be_encoded = config->frames_to_be_encoded;
    callback_data->eb_enc_parameters.rate_control_mode = config->rateControlMode;
    callback_data->eb_enc_parameters.target_bit_rate = config->targetBitRate;
    callback_data->eb_enc_parameters.max_qp_allowed = config->max_qp_allowed;
    callback_data->eb_enc_parameters.min_qp_allowed = config->min_qp_allowed;
    callback_data->eb_enc_parameters.qp = config->qp;
    callback_data->eb_enc_parameters.use_qp_file = (EbBool)config->use_qp_file;
    callback_data->eb_enc_parameters.disable_dlf_flag = (EbBool)config->disable_dlf_flag;
    callback_data->eb_enc_parameters.enable_warped_motion = (EbBool)config->enable_warped_motion;
    callback_data->eb_enc_parameters.use_default_me_hme = (EbBool)config->use_default_me_hme;
    callback_data->eb_enc_parameters.enable_hme_flag = (EbBool)config->enableHmeFlag;
    callback_data->eb_enc_parameters.enable_hme_level0_flag = (EbBool)config->enableHmeLevel0Flag;
    callback_data->eb_enc_parameters.enable_hme_level1_flag = (EbBool)config->enableHmeLevel1Flag;
    callback_data->eb_enc_parameters.enable_hme_level2_flag = (EbBool)config->enableHmeLevel2Flag;
    callback_data->eb_enc_parameters.search_area_width = config->searchAreaWidth;
    callback_data->eb_enc_parameters.search_area_height = config->searchAreaHeight;
    callback_data->eb_enc_parameters.number_hme_search_region_in_width = config->numberHmeSearchRegionInWidth;
    callback_data->eb_enc_parameters.number_hme_search_region_in_height = config->numberHmeSearchRegionInHeight;
    callback_data->eb_enc_parameters.hme_level0_total_search_area_width = config->hmeLevel0TotalSearchAreaWidth;
    callback_data->eb_enc_parameters.hme_level0_total_search_area_height = config->hmeLevel0TotalSearchAreaHeight;
    callback_data->eb_enc_parameters.constrained_intra = (EbBool)config->constrained_intra;
    callback_data->eb_enc_parameters.channel_id = config->channel_id;
    callback_data->eb_enc_parameters.active_channel_count = config->active_channel_count;
    callback_data->eb_enc_parameters.improve_sharpness = (uint8_t)config->improve_sharpness;
    callback_data->eb_enc_parameters.high_dynamic_range_input = config->high_dynamic_range_input;
    callback_data->eb_enc_parameters.encoder_bit_depth = config->encoderBitDepth;
    callback_data->eb_enc_parameters.compressed_ten_bit_format = config->compressedTenBitFormat;
    callback_data->eb_enc_parameters.profile = config->profile;
    callback_data->eb_enc_parameters.tier = config->tier;
    callback_data->eb_enc_parameters.level = config->level;
    callback_data->eb_enc_parameters.injector_frame_rate = config->injector_frame_rate;
    callback_data->eb_enc_parameters.speed_control_flag = config->speed_control_flag;
    callback_data->eb_enc_parameters.asm_type = config->asmType;
    callback_data->eb_enc_parameters.logical_processors = config->logicalProcessors;
    callback_data->eb_enc_parameters.target_socket = config->targetSocket;
    callback_data->eb_enc_parameters.recon_enabled = config->reconFile ? EB_TRUE : EB_FALSE;

    for (hmeRegionIndex = 0; hmeRegionIndex < callback_data->eb_enc_parameters.number_hme_search_region_in_width; ++hmeRegionIndex) {
        callback_data->eb_enc_parameters.hme_level0_search_area_in_width_array[hmeRegionIndex] = config->hmeLevel0SearchAreaInWidthArray[hmeRegionIndex];
        callback_data->eb_enc_parameters.hme_level1_search_area_in_width_array[hmeRegionIndex] = config->hmeLevel1SearchAreaInWidthArray[hmeRegionIndex];
        callback_data->eb_enc_parameters.hme_level2_search_area_in_width_array[hmeRegionIndex] = config->hmeLevel2SearchAreaInWidthArray[hmeRegionIndex];
    }

    for (hmeRegionIndex = 0; hmeRegionIndex < callback_data->eb_enc_parameters.number_hme_search_region_in_height; ++hmeRegionIndex) {
        callback_data->eb_enc_parameters.hme_level0_search_area_in_height_array[hmeRegionIndex] = config->hmeLevel0SearchAreaInHeightArray[hmeRegionIndex];
        callback_data->eb_enc_parameters.hme_level1_search_area_in_height_array[hmeRegionIndex] = config->hmeLevel1SearchAreaInHeightArray[hmeRegionIndex];
        callback_data->eb_enc_parameters.hme_level2_search_area_in_height_array[hmeRegionIndex] = config->hmeLevel2SearchAreaInHeightArray[hmeRegionIndex];
    }

    return return_error;

}

static EbErrorType AllocateFrameBuffer(
    EbConfig_t          *config,
    uint8_t               *p_buffer){

    EbErrorType   return_error = EB_ErrorNone;
    const int32_t tenBitPackedMode = (config->encoderBitDepth > 8) && (config->compressedTenBitFormat == 0) ? 1 : 0;

    // Determine size of each plane
    const size_t luma8bitSize =

        config->inputPaddedWidth    *
        config->inputPaddedHeight   *

        (1 << tenBitPackedMode);

    const size_t chroma8bitSize = luma8bitSize >> 2;
    const size_t luma10bitSize = (config->encoderBitDepth > 8 && tenBitPackedMode == 0) ? luma8bitSize : 0;
    const size_t chroma10bitSize = (config->encoderBitDepth > 8 && tenBitPackedMode == 0) ? chroma8bitSize : 0;

    // Determine
    EbSvtIOFormat* inputPtr = (EbSvtIOFormat*)p_buffer;
    inputPtr->y_stride = config->inputPaddedWidth;
    inputPtr->cr_stride = config->inputPaddedWidth >> 1;
    inputPtr->cb_stride = config->inputPaddedWidth >> 1;
    if (luma8bitSize) {
        EB_APP_MALLOC(uint8_t*, inputPtr->luma, luma8bitSize, EB_N_PTR, EB_ErrorInsufficientResources);
    }
    else {
        inputPtr->luma = 0;
    }
    if (chroma8bitSize) {
        EB_APP_MALLOC(uint8_t*, inputPtr->cb, chroma8bitSize, EB_N_PTR, EB_ErrorInsufficientResources);
    }
    else {
        inputPtr->cb = 0;
    }

    if (chroma8bitSize) {
        EB_APP_MALLOC(uint8_t*, inputPtr->cr, chroma8bitSize, EB_N_PTR, EB_ErrorInsufficientResources);
    }
    else {
        inputPtr->cr = 0;
    }

    if (luma10bitSize) {
        EB_APP_MALLOC(uint8_t*, inputPtr->luma_ext, luma10bitSize, EB_N_PTR, EB_ErrorInsufficientResources);
    }
    else {
        inputPtr->luma_ext = 0;
    }

    if (chroma10bitSize) {
        EB_APP_MALLOC(uint8_t*, inputPtr->cb_ext, chroma10bitSize, EB_N_PTR, EB_ErrorInsufficientResources);
    }
    else {
        inputPtr->cb_ext = 0;
    }

    if (chroma10bitSize) {
        EB_APP_MALLOC(uint8_t*, inputPtr->cr_ext, chroma10bitSize, EB_N_PTR, EB_ErrorInsufficientResources);

    }
    else {
        inputPtr->cr_ext = 0;
    }

    return return_error;
}


EbErrorType AllocateInputBuffers(
    EbConfig_t                *config,
    EbAppContext            *callback_data)
{
    EbErrorType   return_error = EB_ErrorNone;
    {
        EB_APP_MALLOC(EbBufferHeaderType*, callback_data->input_buffer_pool, sizeof(EbBufferHeaderType), EB_N_PTR, EB_ErrorInsufficientResources);

        // Initialize Header
        callback_data->input_buffer_pool->size                       = sizeof(EbBufferHeaderType);

        EB_APP_MALLOC(uint8_t*, callback_data->input_buffer_pool->p_buffer, sizeof(EbSvtIOFormat), EB_N_PTR, EB_ErrorInsufficientResources);

        if (config->bufferedInput == -1) {

            // Allocate frame buffer for the p_buffer
            AllocateFrameBuffer(
                    config,
                    callback_data->input_buffer_pool->p_buffer);
        }

        // Assign the variables
        callback_data->input_buffer_pool->p_app_private = NULL;
        callback_data->input_buffer_pool->pic_type   = EB_AV1_INVALID_PICTURE;
    }

    return return_error;
}
EbErrorType AllocateOutputReconBuffers(
    EbConfig_t                *config,
    EbAppContext            *callback_data)
{

    EbErrorType   return_error = EB_ErrorNone;
    const size_t lumaSize =
        config->inputPaddedWidth    *
        config->inputPaddedHeight;
    // both u and v
    const size_t chromaSize = lumaSize >> 1;
    const size_t tenBit = (config->encoderBitDepth > 8);
    const size_t frameSize = (lumaSize + chromaSize) << tenBit;

// ... Recon Port
    EB_APP_MALLOC(EbBufferHeaderType*, callback_data->recon_buffer, sizeof(EbBufferHeaderType), EB_N_PTR, EB_ErrorInsufficientResources);

    // Initialize Header
    callback_data->recon_buffer->size = sizeof(EbBufferHeaderType);

    EB_APP_MALLOC(uint8_t*, callback_data->recon_buffer->p_buffer, frameSize, EB_N_PTR, EB_ErrorInsufficientResources);

    callback_data->recon_buffer->n_alloc_len = (uint32_t)frameSize;
    callback_data->recon_buffer->p_app_private = NULL;
    return return_error;
}

EbErrorType AllocateOutputBuffers(
    EbConfig_t                *config,
    EbAppContext            *callback_data)
{

    EbErrorType   return_error = EB_ErrorNone;
    uint32_t           outputStreamBufferSize = (uint32_t)(EB_OUTPUTSTREAMBUFFERSIZE_MACRO(config->inputPaddedHeight * config->inputPaddedWidth));;
    {
        EB_APP_MALLOC(EbBufferHeaderType*, callback_data->stream_buffer_pool, sizeof(EbBufferHeaderType), EB_N_PTR, EB_ErrorInsufficientResources);

        // Initialize Header
        callback_data->stream_buffer_pool->size = sizeof(EbBufferHeaderType);

        EB_APP_MALLOC(uint8_t*, callback_data->stream_buffer_pool->p_buffer, outputStreamBufferSize, EB_N_PTR, EB_ErrorInsufficientResources);

        callback_data->stream_buffer_pool->n_alloc_len = outputStreamBufferSize;
        callback_data->stream_buffer_pool->p_app_private = NULL;
        callback_data->stream_buffer_pool->pic_type = EB_AV1_INVALID_PICTURE;
    }
    return return_error;
}

EbErrorType PreloadFramesIntoRam(
    EbConfig_t                *config)
{
    EbErrorType    return_error = EB_ErrorNone;
    int32_t             processedFrameCount;
    int32_t             filledLen;
    int32_t             inputPaddedWidth = config->inputPaddedWidth;
    int32_t             inputPaddedHeight = config->inputPaddedHeight;
    int32_t             readSize;
    uint8_t  *ebInputPtr;

    FILE *inputFile = config->inputFile;

    if (config->encoderBitDepth == 10 && config->compressedTenBitFormat == 1)
    {

        readSize = (inputPaddedWidth*inputPaddedHeight * 3) / 2 + (inputPaddedWidth / 4 * inputPaddedHeight * 3) / 2;

    }
    else
    {

        readSize = inputPaddedWidth * inputPaddedHeight * 3 * (config->encoderBitDepth > 8 ? 2 : 1) / 2;

    }
    EB_APP_MALLOC(uint8_t **, config->sequenceBuffer, sizeof(uint8_t*) * config->bufferedInput, EB_N_PTR, EB_ErrorInsufficientResources);


    for (processedFrameCount = 0; processedFrameCount < config->bufferedInput; ++processedFrameCount) {
        EB_APP_MALLOC(uint8_t*, config->sequenceBuffer[processedFrameCount], readSize, EB_N_PTR, EB_ErrorInsufficientResources);
        // Interlaced Video
        if (config->separateFields) {
            EbBool is16bit = config->encoderBitDepth > 8;
            if (is16bit == 0 || (is16bit == 1 && config->compressedTenBitFormat == 0)) {

                const int32_t tenBitPackedMode = (config->encoderBitDepth > 8) && (config->compressedTenBitFormat == 0) ? 1 : 0;

                const size_t luma8bitSize =

                    (config->inputPaddedWidth) *
                    (config->inputPaddedHeight) *

                    (1 << tenBitPackedMode);

                const size_t chroma8bitSize = luma8bitSize >> 2;

                filledLen = 0;

                ProcessInputFieldBufferingMode(
                    processedFrameCount,
                    &filledLen,
                    inputFile,
                    config->sequenceBuffer[processedFrameCount],
                    config->sequenceBuffer[processedFrameCount] + luma8bitSize,
                    config->sequenceBuffer[processedFrameCount] + luma8bitSize + chroma8bitSize,
                    (uint32_t)inputPaddedWidth,
                    (uint32_t)inputPaddedHeight,

                    is16bit);

                if (readSize != filledLen) {

                    fseek(inputFile, 0, SEEK_SET);
                    filledLen = 0;

                    ProcessInputFieldBufferingMode(
                        processedFrameCount,
                        &filledLen,
                        inputFile,
                        config->sequenceBuffer[processedFrameCount],
                        config->sequenceBuffer[processedFrameCount] + luma8bitSize,
                        config->sequenceBuffer[processedFrameCount] + luma8bitSize + chroma8bitSize,
                        (uint32_t)inputPaddedWidth,
                        (uint32_t)inputPaddedHeight,

                        is16bit);
                }

                // Reset the pointer position after a top field
                if (processedFrameCount % 2 == 0) {
                    fseek(inputFile, -(long)(readSize << 1), SEEK_CUR);
                }
            }
            // Unpacked 10 bit
            else {

                const int32_t tenBitPackedMode = (config->encoderBitDepth > 8) && (config->compressedTenBitFormat == 0) ? 1 : 0;

                const size_t luma8bitSize =
                    (config->inputPaddedWidth) *
                    (config->inputPaddedHeight) *
                    (1 << tenBitPackedMode);

                const size_t chroma8bitSize = luma8bitSize >> 2;

                const size_t luma10bitSize = (config->encoderBitDepth > 8 && tenBitPackedMode == 0) ? luma8bitSize : 0;
                const size_t chroma10bitSize = (config->encoderBitDepth > 8 && tenBitPackedMode == 0) ? chroma8bitSize : 0;

                filledLen = 0;

                ProcessInputFieldBufferingMode(
                    processedFrameCount,
                    &filledLen,
                    inputFile,
                    config->sequenceBuffer[processedFrameCount],
                    config->sequenceBuffer[processedFrameCount] + luma8bitSize,
                    config->sequenceBuffer[processedFrameCount] + luma8bitSize + chroma8bitSize,
                    (uint32_t)inputPaddedWidth,
                    (uint32_t)inputPaddedHeight,
                    0);

                ProcessInputFieldBufferingMode(
                    processedFrameCount,
                    &filledLen,
                    inputFile,
                    config->sequenceBuffer[processedFrameCount] + luma8bitSize + (chroma8bitSize << 1),
                    config->sequenceBuffer[processedFrameCount] + luma8bitSize + (chroma8bitSize << 1) + luma10bitSize,
                    config->sequenceBuffer[processedFrameCount] + luma8bitSize + (chroma8bitSize << 1) + luma10bitSize + chroma10bitSize,
                    (uint32_t)inputPaddedWidth,
                    (uint32_t)inputPaddedHeight,
                    0);

                if (readSize != filledLen) {

                    fseek(inputFile, 0, SEEK_SET);
                    filledLen = 0;

                    ProcessInputFieldBufferingMode(
                        processedFrameCount,
                        &filledLen,
                        inputFile,
                        config->sequenceBuffer[processedFrameCount],
                        config->sequenceBuffer[processedFrameCount] + luma8bitSize,
                        config->sequenceBuffer[processedFrameCount] + luma8bitSize + chroma8bitSize,
                        (uint32_t)inputPaddedWidth,
                        (uint32_t)inputPaddedHeight,
                        0);

                    ProcessInputFieldBufferingMode(
                        processedFrameCount,
                        &filledLen,
                        inputFile,
                        config->sequenceBuffer[processedFrameCount] + luma8bitSize + (chroma8bitSize << 1),
                        config->sequenceBuffer[processedFrameCount] + luma8bitSize + (chroma8bitSize << 1) + luma10bitSize,
                        config->sequenceBuffer[processedFrameCount] + luma8bitSize + (chroma8bitSize << 1) + luma10bitSize + chroma10bitSize,
                        (uint32_t)inputPaddedWidth,
                        (uint32_t)inputPaddedHeight,
                        0);

                }

                // Reset the pointer position after a top field
                if (processedFrameCount % 2 == 0) {
                    fseek(inputFile, -(long)(readSize << 1), SEEK_CUR);
                }
            }
        }
        else {

            // Fill the buffer with a complete frame
            filledLen = 0;
            ebInputPtr = config->sequenceBuffer[processedFrameCount];
            filledLen += (uint32_t)fread(ebInputPtr, 1, readSize, inputFile);

            if (readSize != filledLen) {

                fseek(config->inputFile, 0, SEEK_SET);

                // Fill the buffer with a complete frame
                filledLen = 0;
                ebInputPtr = config->sequenceBuffer[processedFrameCount];
                filledLen += (uint32_t)fread(ebInputPtr, 1, readSize, inputFile);
            }
        }
    }

    return return_error;
}

/***************************************
* Functions Implementation
***************************************/

/***********************************
 * Initialize Core & Component
 ***********************************/
EbErrorType init_encoder(
    EbConfig_t              *config,
    EbAppContext          *callback_data,
    uint32_t                 instance_idx)
{
    EbErrorType        return_error = EB_ErrorNone;

    // Allocate a memory table hosting all allocated pointers
    AllocateMemoryTable(instance_idx);

    ///************************* LIBRARY INIT [START] *********************///
    // STEP 1: Call the library to construct a Component Handle
    return_error = eb_init_handle(&callback_data->svt_encoder_handle, callback_data, &callback_data->eb_enc_parameters);

    if (return_error != EB_ErrorNone) {
        return return_error;
    }

    // STEP 3: Copy all configuration parameters into the callback structure
    return_error = CopyConfigurationParameters(
                    config,
                    callback_data,
                    instance_idx);

    if (return_error != EB_ErrorNone) {
        return return_error;
    }

    // STEP 4: Send over all configuration parameters
    // Set the Parameters
    return_error = eb_svt_enc_set_parameter(
                       callback_data->svt_encoder_handle,
                       &callback_data->eb_enc_parameters);

    if (return_error != EB_ErrorNone) {
        return return_error;
    }

    // STEP 5: Init Encoder
    return_error = eb_init_encoder(callback_data->svt_encoder_handle);
    if (return_error != EB_ErrorNone) { return return_error; }

    ///************************* LIBRARY INIT [END] *********************///

    ///********************** APPLICATION INIT [START] ******************///

    // STEP 6: Allocate input buffers carrying the yuv frames in
    return_error = AllocateInputBuffers(
        config,
        callback_data);

    if (return_error != EB_ErrorNone) {
        return return_error;
    }

    // STEP 7: Allocate output buffers carrying the bitstream out
    return_error = AllocateOutputBuffers(
        config,
        callback_data);

    if (return_error != EB_ErrorNone) {
        return return_error;
    }

    // STEP 8: Allocate output Recon Buffer
    return_error = AllocateOutputReconBuffers(
        config,
        callback_data);

    if (return_error != EB_ErrorNone) {
        return return_error;
    }

    // Allocate the Sequence Buffer
    if (config->bufferedInput != -1) {

        // Preload frames into the ram for a faster yuv access time
        PreloadFramesIntoRam(
            config);
    }
    else {
        config->sequenceBuffer = 0;
    }

    if (return_error != EB_ErrorNone) {
        return return_error;
    }


    ///********************** APPLICATION INIT [END] ******************////////

    return return_error;
}

/***********************************
 * Deinit Components
 ***********************************/
EbErrorType de_init_encoder(
    EbAppContext *callback_data_ptr,
    uint32_t        instance_index)
{
    EbErrorType return_error = EB_ErrorNone;
    int32_t              ptrIndex        = 0;
    EbMemoryMapEntry*   memoryEntry     = (EbMemoryMapEntry*)0;

    if (((EbComponentType*)(callback_data_ptr->svt_encoder_handle)) != NULL) {
            return_error = eb_deinit_encoder(callback_data_ptr->svt_encoder_handle);
    }

    // Destruct the buffer memory pool
    if (return_error != EB_ErrorNone) {
        return return_error;
    }

    // Loop through the ptr table and free all malloc'd pointers per channel
    for (ptrIndex = appMemoryMapIndexAllChannels[instance_index] - 1; ptrIndex >= 0; --ptrIndex) {
        memoryEntry = &appMemoryMapAllChannels[instance_index][ptrIndex];
        switch (memoryEntry->ptrType) {
        case EB_N_PTR:
            free(memoryEntry->ptr);
            break;
        default:
            return_error = EB_ErrorMax;
            break;
        }
    }
    free(appMemoryMapAllChannels[instance_index]);

    // Destruct the component
    eb_deinit_handle(callback_data_ptr->svt_encoder_handle);

    return return_error;
}
