//
// Created by Ukua on 2023/8/2.
//

#include "tinybus.h"


Bus_TopicHandleTypeDef *g_topic_root;
Bus_PtrTypeDef *g_ptr_root;
osMessageQId g_bus_callback_queue;

/**
 * @brief 搜索话题
 * @param topicName:话题名称
 * @return 话题句柄(当话题不存在时返回NULL)
 */
Bus_TopicHandleTypeDef *Bus_TopicSearch(char *topicName) {
    //遍历所有话题，如果找到了同名话题，返回该话题句柄
    Bus_TopicHandleTypeDef *it;
    for (it = g_topic_root; it != NULL; it = it->_next_topic) {
        if (strcmp(it->name, topicName) == 0)
            return it;
    }
    return NULL;
}


/**
 * @brief 话题注册
 * @param topic_name:话题名称
 * @return 话题句柄
 * @note 如果先前存在同名话题，则会直接返回该话题句柄
 */
Bus_TopicHandleTypeDef *Bus_TopicRegister(char *topicName) {
    //遍历所有话题，如果找到了同名话题，返回该话题句柄
    Bus_TopicHandleTypeDef *it;
    for (it = g_topic_root; it != NULL; it = it->_next_topic) {
        if (strcmp(it->name, topicName) == 0)
            return it;
    }

    //如果没有找到同名话题，创建一个新的话题句柄并插入到话题链表末尾
    Bus_TopicHandleTypeDef *topic = (Bus_TopicHandleTypeDef *) JUST_MALLOC(sizeof(Bus_TopicHandleTypeDef));
    topic->name = topicName;
    topic->first_subscriber = NULL;
    topic->_next_topic = NULL;

    for (it = g_topic_root; it->_next_topic != NULL; it = it->_next_topic);
    it->_next_topic = topic;

    return topic;
}


/**
 * @brief 订阅话题
 * @param topic:话题句柄
 * @param *callback:订阅回调函数
 * @return 订阅句柄
 * @note 订阅默认情况下自动开启接收
 */
Bus_SubscriberTypeDef *
Bus_Subscribe(Bus_TopicHandleTypeDef *topic, void (*callback)(void *message, Bus_TopicHandleTypeDef *topic)) {
    Bus_SubscriberTypeDef *subscriber = (Bus_SubscriberTypeDef *) JUST_MALLOC(sizeof(Bus_SubscriberTypeDef));
    subscriber->callback = callback;
    subscriber->enable = 1;
    subscriber->_next_subscriber = NULL;
    subscriber->topic = topic;

    //把订阅插入到订阅链表的末尾
    if (topic->first_subscriber != NULL) {
        Bus_SubscriberTypeDef *it;
        for (it = topic->first_subscriber; it->_next_subscriber != NULL; it = it->_next_subscriber);
        it->_next_subscriber = subscriber;
    } else
        topic->first_subscriber = subscriber;

    return subscriber;
}


/**
 * @brief 从话题名字订阅话题
 * @param topic_name:话题名称
 * @param *callback:订阅回调函数
 * @return 订阅句柄
 * @note 订阅默认情况下自动开启接收
 * @note 如果不存在该名字的话题，则会自动创建一个新的话题
 */
Bus_SubscriberTypeDef *
Bus_SubscribeFromName(char *topic_name, void (*callback)(void *message, Bus_TopicHandleTypeDef *topic)) {
    Bus_TopicHandleTypeDef *topic = Bus_TopicSearch(topic_name);
    if (topic == NULL)
        topic = Bus_TopicRegister(topic_name);
    return Bus_Subscribe(topic, callback);
}


void Bus_MainLoop() {
    //遍历所有话题，如果有订阅者，则调用其回调函数

}

/**
 * @brief 发布消息
 * @param topic:话题句柄
 * @param message:消息数据指针
 */
void Bus_Publish(Bus_TopicHandleTypeDef *topic, void *message) {
    Bus_SubscriberTypeDef *it;
    for (it = topic->first_subscriber; it != NULL; it = it->_next_subscriber) {
        if (it->enable)
            it->callback(message, topic);
    }
}


/**
 * @brief 从话题名字发布消息
 * @param topic_name:话题名称
 * @param message:消息数据指针
 * @note 如果不存在该名字的话题，则会自动创建一个新的话题
 */
void Bus_PublishFromName(char *topic_name, void *message) {
    Bus_TopicHandleTypeDef *topic = Bus_TopicSearch(topic_name);
    if (topic == NULL)
        topic = Bus_TopicRegister(topic_name);
    Bus_Publish(topic, message);
}


/**
 * @brief 共享指针
 * @param ptrName:指针名称
 * @param len:内存大小
 * @return 所需共享指针
 * @note 如果不存在该名字的指针，则会自动创申请一块新的空间
 * @note 该函数会自动给内存区域置零
 */
void *Bus_SharePtr(char *ptrName, size_t len) {
    Bus_PtrTypeDef *it;
    for (it = g_ptr_root; it != NULL; it = it->_next_ptr) {
        if (strcmp(it->name, ptrName) == 0)
            return it->ptr;
    }
    //如果不存在，则创建指针

    for (it = g_ptr_root; it->_next_ptr != NULL; it = it->_next_ptr);
    Bus_PtrTypeDef *bus_ptr = JUST_MALLOC(sizeof(Bus_PtrTypeDef));
    bus_ptr->name = ptrName;
    bus_ptr->ptr = JUST_MALLOC(len);
    bus_ptr->_next_ptr = NULL;
    it->_next_ptr = bus_ptr;

    //以防万一给内存个清零
    memset(bus_ptr->ptr, 0, len);

    return bus_ptr->ptr;
}

/**
 * @brief 发布共享指针
 * @note 不考虑重命名错误
 */
void Publish_SharePtr(char *ptrName,size_t len,void*ptr)
{
    Bus_PtrTypeDef *it;
    for (it = g_ptr_root; it->_next_ptr != NULL; it = it->_next_ptr)
    ;
    Bus_PtrTypeDef *bus_ptr = JUST_MALLOC(sizeof(Bus_PtrTypeDef));
    bus_ptr->ptr = JUST_MALLOC(len);
    bus_ptr->name = ptrName;
    bus_ptr->ptr=ptr;
    bus_ptr->_next_ptr = NULL;
    it->_next_ptr = bus_ptr;
}

/**
 * @brief 话题初始化
 * @return 0:成功 1:失败
 */
uint8_t Bus_Init() {
    g_topic_root = JUST_MALLOC(sizeof(Bus_TopicHandleTypeDef));
    g_topic_root->name = "root";
    g_topic_root->first_subscriber = NULL;
    g_topic_root->_next_topic = NULL;
    if (g_topic_root == NULL)
        return 1;

    g_ptr_root = JUST_MALLOC(sizeof(Bus_PtrTypeDef));
    g_ptr_root->name = "root";
    g_ptr_root->_next_ptr = NULL;
    if (g_ptr_root == NULL)
        return 1;


    return 0;
}