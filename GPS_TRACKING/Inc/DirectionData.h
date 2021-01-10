#ifndef __DIRECTION_DATA_H__
#define __DIRECTION_DATA_H__

#include <stdlib.h>

#define DIRECTION_DATA_LIST_NODE_LIMIT 255.

typedef enum AddResultCodeEnum {
    STATUS_OK = 0,
    STATUS_EXCEED_LIMIT = 1,
    STATUS_NOT_ENOUGH_MEMORY = 2
} AddResultCode;

typedef struct direction_data{
    double lat;
    double lon;
} DirectionData;

typedef struct DirectionDataNode_Tag
{
    DirectionData *data;
    struct DirectionDataNode_Tag *pNext;
} DirectionDataNode;

typedef struct
{
    DirectionDataNode *pHead;
    DirectionDataNode *pTail;
    int count;
} DirectionDataList;


DirectionData* newData(char* str);
void DirectionDataList_Init(DirectionDataList *);
AddResultCode DirectionDataList_Put(DirectionDataList *, DirectionData *);
DirectionData *DirectionDataList_Get(DirectionDataList *);
void DirectionDataList_Clear(DirectionDataList *);

#endif /* __DIRECTION_DATA_H__ */
