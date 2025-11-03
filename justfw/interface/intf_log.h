//
// Created by Ukua on 2023/9/24.
//

#ifndef JUSTFW_INTF_LOG_H
#define JUSTFW_INTF_LOG_H

#include "stdio.h"

#define LOG(str) Bus_PublishFromName("LOG", str);

#endif //JUSTFW_INTF_LOG_H
