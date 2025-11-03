//
// Created by Konodoki on 2024-03-10.
//

#ifndef JUSTFW_FREERTOSDEBUG_H
#define JUSTFW_FREERTOSDEBUG_H
//#define English
#define RunTimeInfoBuffSize 1024
#define Freertos_Debug_Pulse_Timer TIM14
void Freeretos_Debug_Start();
void Time_IR_Callback();
void PrintFreeRTOSLog();
#endif //JUSTFW_FREERTOSDEBUG_H
