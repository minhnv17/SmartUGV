#ifndef U_ORB_H
#define U_ORB_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define MAX_TOPICS 10
#define MAX_SUBSCRIBERS 10

typedef void (*uORB_SubscribeCallback_t)(const void *data, uint32_t size);

typedef struct
{
    char topic[30]; // Assuming a maximum topic name length of 29 characters + null terminator
    uint8_t *data;
    uint32_t size;
    bool in_use;
} uORB_Topic;

uORB_Topic *uORB_RegisterTopic(const char *topic, uint8_t *data, uint32_t size);
void uORB_UnregisterTopic(const char *topic);
int8_t uORB_Publish(const uORB_Topic *topic_handle, const void *data);
uint16_t uORB_Subscribe(const char *topic);
int16_t uORB_CopyData(const uint16_t fd, void *data);

#endif /* U_ORB_H */
