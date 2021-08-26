#ifndef __HTTPSERVER_NETCONN_H__
#define __HTTPSERVER_NETCONN_H__

#include "lwip/api.h"
 #include "lwip/sockets.h"                                     // CMSIS RTOS header file

#include <cmsis_os.h> 
extern void MotorTask(void const * args);             // function prototype
extern osThreadId MotorTaskID;
//osThreadDef (Motor,MotorTask, osPriorityBelowNormal, 1, 100);
//extern  char RecvDataToQ;
extern  char MotorRecvQ;
extern struct MotorData myMotorData;
typedef struct {
  char RcvPulses[20];
} MotorData;


// Pool definition
//extern osPoolDef(MyPool, 10, long);
//extern osMailQDef(Motor_q, 5, int);
////extern osMailQId (Motor_q_id);
extern osMailQId Motor_q_id;
extern osSemaphoreDef_t my_semaphore;    // Declare semaphore
extern osSemaphoreId  my_semaphore_id;
void http_server_netconn_init(void);
void DynWebPage(struct netconn *conn);

#endif /* __HTTPSERVER_NETCONN_H__ */
