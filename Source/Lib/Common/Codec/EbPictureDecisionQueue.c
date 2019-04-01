/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

#include <stdlib.h>
#include "EbPictureDecisionQueue.h"


EbErrorType pa_reference_queue_entry_ctor(
    PaReferenceQueueEntry_t   **entry_dbl_ptr)
{
    PaReferenceQueueEntry_t *entryPtr;
    EB_MALLOC(PaReferenceQueueEntry_t*, entryPtr, sizeof(PaReferenceQueueEntry_t), EB_N_PTR);
    *entry_dbl_ptr = entryPtr;

    entryPtr->inputObjectPtr = (EbObjectWrapper*)EB_NULL;
    entryPtr->picture_number = 0;
    entryPtr->referenceEntryIndex = 0;
    entryPtr->dependentCount = 0;
#if BASE_LAYER_REF
    EB_MALLOC(ReferenceList*, entryPtr->list0Ptr, sizeof(ReferenceList), EB_N_PTR);
    EB_MALLOC(ReferenceList*, entryPtr->list1Ptr, sizeof(ReferenceList), EB_N_PTR);
    entryPtr->list0Ptr->reference_list = 0;
    entryPtr->list0Ptr->reference_list_count = 0;
    entryPtr->list1Ptr->reference_list = 0;
    entryPtr->list1Ptr->reference_list_count = 0;
#else
    entryPtr->list0Ptr = (ReferenceList*)EB_NULL;
    entryPtr->list1Ptr = (ReferenceList*)EB_NULL;
#endif
    EB_MALLOC(int32_t*, entryPtr->list0.list, sizeof(int32_t) * (1 << MAX_TEMPORAL_LAYERS), EB_N_PTR);

    EB_MALLOC(int32_t*, entryPtr->list1.list, sizeof(int32_t) * (1 << MAX_TEMPORAL_LAYERS), EB_N_PTR);

    return EB_ErrorNone;
}


