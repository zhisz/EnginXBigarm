//
// Created by Ukua on 2023/8/2.
//

#ifndef JUSTFW_TINYBUS_H
#define JUSTFW_TINYBUS_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "interface.h"


typedef struct Bus_TopicHandle Bus_TopicHandleTypeDef;
typedef struct Bus_Subscriber Bus_SubscriberTypeDef;
typedef struct Bus_Ptr Bus_PtrTypeDef;


struct Bus_Subscriber {
    uint8_t enable;//是否启用订阅接收，默认开启
    void *_next_subscriber;
    struct Bus_TopicHandle *topic;//所属话题

    /**
     * @brief 订阅回调函数
     * @param message:消息数据指针
     * @param topic:收到消息的话题句柄
     */
    void (*callback)(void *message, Bus_TopicHandleTypeDef *topic);

};

struct Bus_TopicHandle {
    char *name;
    Bus_SubscriberTypeDef *first_subscriber;
    Bus_TopicHandleTypeDef *_next_topic;
};

struct Bus_Ptr {
    char *name;
    void *ptr;
    Bus_PtrTypeDef *_next_ptr;
};


/**
 * @brief 搜索话题
 * @param topicName:话题名称
 * @return 话题句柄(当话题不存在时返回NULL)
 */
Bus_TopicHandleTypeDef *Bus_TopicSearch(char *topicName);

/**
 * @brief 话题注册
 * @param topic_name:话题名称
 * @return 话题句柄
 * @note 如果先前存在同名话题，则会直接返回该话题句柄
 */
Bus_TopicHandleTypeDef *Bus_TopicRegister(char *topicName);

/**
 * @brief 订阅话题
 * @param topic:话题句柄
 * @param *callback:订阅回调函数
 * @return 订阅句柄
 * @note 订阅默认情况下自动开启接收
 */
Bus_SubscriberTypeDef *Bus_Subscribe(Bus_TopicHandleTypeDef *topic,
                                     void (*callback)(void *message, Bus_TopicHandleTypeDef *topic));

/**
 * @brief 从话题名字订阅话题
 * @param topic_name:话题名称
 * @param *callback:订阅回调函数
 * @return 订阅句柄
 * @note 订阅默认情况下自动开启接收
 * @note 如果不存在该名字的话题，则会自动创建一个新的话题
 */
Bus_SubscriberTypeDef *Bus_SubscribeFromName(char *topic_name,
                                             void (*callback)(void *message, Bus_TopicHandleTypeDef *topic));

/**
 * @brief 发布消息
 * @param topic:话题句柄
 * @param *message:消息数据指针
 */
void Bus_Publish(Bus_TopicHandleTypeDef *topic, void *message);

/**
 * @brief 从话题名字发布消息
 * @param topic_name:话题名称
 * @param *message:消息数据指针
 * @note 如果不存在该名字的话题，则会自动创建一个新的话题
 */
void Bus_PublishFromName(char *topic_name, void *message);

/**
 * @brief 共享指针
 * @param ptrName:指针名称
 * @param len:内存大小
 * @note 如果不存在该名字的指针，则会自动创申请一块新的空间
 */
void *Bus_SharePtr(char *ptrName, size_t len);

/**
 * @brief 话题初始化
 * @return 0:成功 1:失败
 */
uint8_t Bus_Init();


#endif //JUSTFW_TINYBUS_H
