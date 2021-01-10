#include "DirectionData.h"

void DirectionDataList_Init(DirectionDataList *list)
{
    list->pHead = NULL;
    list->pTail = NULL;
    list->count = 0;
}

AddResultCode DirectionDataList_Put(DirectionDataList *list, DirectionData* directionData)
{
    if (list->count < DIRECTION_DATA_LIST_NODE_LIMIT)
    {
        DirectionDataNode *node = (DirectionDataNode *)malloc(sizeof(DirectionDataNode));
        if (node == NULL)
        {
            return STATUS_NOT_ENOUGH_MEMORY;
        }
        node->data = directionData;
        node->pNext = NULL;

        if (list->count == 0)
        {
            list->pHead = node;
            list->pTail = node;
        }
        else
        {
            list->pTail->pNext = node;
            list->pTail = node;
        }
        list->count++;

        return STATUS_OK;
    }
    else
    {
        return STATUS_EXCEED_LIMIT;
    }
}

DirectionData *DirectionDataList_Get(DirectionDataList *list)
{
    if (list->count > 0)
    {
        DirectionDataNode *node = list->pHead;
        DirectionData *data = node->data;

        if (--(list->count) == 0)
        {
            list->pHead = NULL;
            list->pTail = NULL;
        }
        else
        {
            list->pHead = node->pNext;
        }

        free(node);
        // *data will be free by user.

        return data;
    }
    else
    {
        return NULL;
    }
}

void DirectionDataList_Clear(DirectionDataList *list)
{
    DirectionDataNode *node;

    while (list->pHead != NULL)
    {
        node = list->pHead;
        list->pHead = node->pNext;

        free(node->data);
        free(node);
    }
    DirectionDataList_Init(list);
}

DirectionData* newData(char* str){
	DirectionData *newPoint = (DirectionData *)malloc(sizeof(DirectionData));
	char lat[10]="";
	char lon[10]="";
	int i = 0;
	int iLatLon = 0;
	for(i;i<strlen(str);i++){
		if(str[i]==','){
			iLatLon = 0;
			i++;
			break;
		}
		lat[iLatLon++] = str[i];
	}
	for(i;i<strlen(str);i++){
		if(str[i]==','){
			iLatLon = 0;
			break;
		}
		lon[iLatLon++] = str[i];
	}
	newPoint->lat = atof(lat);
	newPoint->lon = atof(lon);
	return newPoint;
}
