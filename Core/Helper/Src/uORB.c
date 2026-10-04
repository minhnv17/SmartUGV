#include <string.h>
#include "uORB.h"

static uORB_Topic topics[MAX_TOPICS]; // Assuming a maximum of 10 topics
static int16_t used_topics = 0;

uORB_Topic *uORB_RegisterTopic(const char *topic, uint8_t *data, uint32_t size)
{
    if (used_topics >= MAX_TOPICS)
    {
        return NULL; // Error: Maximum number of topics reached
    }

    if (topic == NULL || data == NULL || size == 0)
    {
        return NULL; // Error: Invalid topic or data
    }

    for (int i = 0; i < MAX_TOPICS; i++)
    {
        if (topics[i].in_use && strcmp(topics[i].topic, topic) == 0)
        {
            return NULL; // Error: Topic already registered
        }
    }

    // Find an available topic slot
    for (int i = 0; i < MAX_TOPICS; i++)
    {
        if (!topics[i].in_use)
        {
            topics[i].in_use = true;
            strncpy(topics[i].topic, topic, sizeof(topics[i].topic) - 1);
            topics[i].topic[sizeof(topics[i].topic) - 1] = '\0'; // Ensure null termination
            topics[i].data = data;
            topics[i].size = size;
            return &topics[i]; // Return the registered topic handle
        }
    }

    return NULL; // Error: No available topic slot
}

void uORB_UnregisterTopic(const char *topic)
{
    for (int i = 0; i < MAX_TOPICS; i++)
    {
        if (topics[i].in_use && strcmp(topics[i].topic, topic) == 0)
        {
            topics[i].in_use = false;
            return; // Successfully unregistered the topic
        }
    }
}

int8_t uORB_Publish(const uORB_Topic *topic_handle, const void *data)
{
    if (topic_handle == NULL || data == NULL)
    {
        return -1; // Error: Invalid topic handle or data
    }

    if (topic_handle->in_use == false)
    {
        return -1; // Error: Topic not in use or size mismatch
    }
    memcpy(topic_handle->data, data, topic_handle->size);

    return 0; // Success
}

uint16_t uORB_Subscribe(const char *topic)
{
    for (int i = 0; i < MAX_TOPICS; i++)
    {
        if (topics[i].in_use && strcmp(topics[i].topic, topic) == 0)
        {
            return i; // Return the index of the topic
        }
    }
    return -1; // Error: Topic not found
}

int16_t uORB_CopyData(const uint16_t fd, void *data)
{
    if (fd >= MAX_TOPICS || !topics[fd].in_use)
    {
        return -1; // Error: Invalid file descriptor
    }

    memcpy(data, topics[fd].data, topics[fd].size);
    return topics[fd].size; // Success
}
