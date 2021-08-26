/**
  ******************************************************************************
  * @file    LwIP/LwIP_HTTP_Server_Netconn_RTOS/Src/httpser-netconn.c 
  * @author  MCD Application Team
  * @brief   Basic http server implementation using LwIP netconn API  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/apps/fs.h"
#include "string.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "httpserver-netconn.h"

#include "stm32f7xx.h"                  // Device header

#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define WEBSERVER_THREAD_PRIO    ( osPriorityNormal )





MotorData RecvDataToQ;
struct netconn  *conn;
struct netconn  *newconn;





/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief serve tcp connection  
  * @param conn: pointer on connection structure 
  * @retval None
  */


/**
  * @brief  http server thread 
  * @param arg: pointer on argument(not used here) 
  * @retval None
  */
static void http_server_netconn_thread_recv(void *args){
	struct netbuf *buf;
	char * temp;
	err_t RecvError;
	 void *RecvData;
	
u16_t RecvLen;
	osMailQDef(Motor_q, 10,MotorData);
	osThreadDef(Motor,MotorTask,osPriorityNormal,0,configMINIMAL_STACK_SIZE*1);
	MotorTaskID=osThreadCreate(osThread(Motor), "TASK");
	Motor_q_id = osMailCreate(osMailQ(Motor_q), NULL);	
	buf=netbuf_new();
	while(1){
	RecvError =netconn_recv(newconn,&buf);
		if(RecvError!=ERR_OK){
			while(1);
		} 
	//netconn_write(newconn, SendData, len, NETCONN_COPY);
	netbuf_data(buf,&RecvData,&RecvLen); //recieves data
	//netconn_write(newconn, RecvData, len, NETCONN_COPY);
	RecvDataToQ = *(MotorData *) osMailAlloc(Motor_q_id, osWaitForever);	
	
	//temp=*( char **)RecvData;
	strcpy(RecvDataToQ.RcvPulses,RecvData);
	//RecvDataToQ->RcvPulses=&temp[0];
			//sends data
	osMailPut(Motor_q_id, &RecvDataToQ);	
	netbuf_delete(buf);
	//buf=netbuf_new();
		//struct netbuf *buf;
	//memset(RecvData,0,sizeof(RecvData));	
	
	//osThreadSetPriority(MotorTaskID,osPriorityRealtime);	
	}

//while(1);

}
static void http_server_netconn_thread(void *args)
{ 


err_t recv_err,err,accept_err;
conn=netconn_new(NETCONN_TCP);



u16_t len=17;
char SendData[13];	

	
osThreadDef(Motor,MotorTask,osPriorityNormal,0,configMINIMAL_STACK_SIZE*5);

	if(conn!=NULL){
		   err = netconn_bind(conn, NULL, 2000);
			 if (err == ERR_OK){
					netconn_listen(conn);
					
					
					while(1){
						//struct netbuf *buf;
					 
						accept_err =netconn_accept(conn, &newconn);
						
						//newconn->mbox_threads_waiting=2;
						//newconn->recv_timeout=20000;
						
						if (accept_err == ERR_OK)	{
							//sys_thread_new("Recv", http_server_netconn_thread_recv, NULL, DEFAULT_THREAD_STACKSIZE*2, WEBSERVER_THREAD_PRIO);
							while(1){
							
							sprintf(SendData,"%d,%d\r\n" ,TIM3->CNT,TIM4->CNT);
							netconn_write(newconn, SendData, len, NETCONN_COPY);
//						
								memset(SendData,0,len);
															
								//osThreadYield();  
									
							//osThreadSetPriority(MotorTaskID,osPriorityRealtime);
							
						
							//}while (netbuf_next(buf) >= 0);
							//netbuf_free(buf);
							//memset(buf,0,sizeof(buf));
							//netbuf_delete(buf);
								}
							
							//netbuf_delete(buf);
								//osThreadSetPriority(MotorTaskID,osPriorityRealtime);
							}
						
					//	memset(RecvData,0,sizeof(RecvData));
						}
						netconn_close(newconn);
						netconn_delete(newconn);
					}
			 }

//////////////////////////------------------------------------------NEW FUNCTION
//		int buflen = 1500;
//		int ret;
//		u16_t len=6;
//u16_t RecvLen;
//unsigned char recv_buffer[1500];
//char SendData[10];
//void *RecvData;
//void *RecvDataToQ;
//const char *testdata="a";
//void *tempSendData;	
//int sock, newconn, size;
// struct sockaddr_in address, remotehost;


////RecvDataToQ= &testdata;
////RecvDataToQ = (void *) osMailAlloc(Motor_q_id, osWaitForever);
////osMailPut(Motor_q_id, RecvDataToQ);
////osThreadDef(Motor,MotorTask,osPriorityNormal,1,configMINIMAL_STACK_SIZE*5);
////MotorTaskID=osThreadCreate(osThread(Motor), "TASK");


// /* create a TCP socket */
//  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
//  {
//    return;
//  }
//  
//  /* bind to port 80 at any interface */
//  address.sin_family = AF_INET;
//  address.sin_port = htons(2000);
//  address.sin_addr.s_addr = INADDR_ANY;

//  if (bind(sock, (struct sockaddr *)&address, sizeof (address)) < 0)
//  {
//    return;
//  }
//  
//  /* listen for incoming connections (TCP listen backlog = 5) */
//  listen(sock, 100);
//  
//  size = sizeof(remotehost);
//  
//  while (1) 
//  {
//		//osThreadYield();
//    newconn = accept(sock, (struct sockaddr *)&remotehost, (socklen_t *)&size);
//   
//		//http_server_serve(newconn);

//	//while(newconn){
//	//	osThreadYield();	
//		//sprintf(SendData,"%d\r\n" ,TIM3->CNT);
//		ret = read(newconn, recv_buffer, buflen);
//		write(newconn,recv_buffer,ret);
//		close(newconn);
//		//sprintf(SendData,"%d\r\n" ,TIM3->CNT);
//		//RecvDataToQ = 	(void *) osMailAlloc(Motor_q_id, osWaitForever);	
//			
//		//RecvDataToQ=RecvData;			
//		//osMailPut(Motor_q_id, RecvDataToQ);
//		//MotorTaskID=osThreadCreate(osThread(Motor), "TASK");
//		//osThreadSetPriority(MotorTaskID,osPriorityRealtime);
//		//} 
//	osThreadYield();	
//	}	
	}



/**
  * @brief  Initialize the HTTP server (start its thread) 
  * @param  none
  * @retval None
  */
void http_server_netconn_init()
{
//  osThreadDef(HTTP,http_server_netconn_thread,osPriorityNormal,0,configMINIMAL_STACK_SIZE*5);
//	osThreadCreate(osThread(HTTP), "TASK");
	sys_thread_new("HTTP", http_server_netconn_thread, NULL, DEFAULT_THREAD_STACKSIZE, WEBSERVER_THREAD_PRIO);
	//sys_thread_new("HTTP", http_server_netconn_thread, NULL, DEFAULT_THREAD_STACKSIZE, WEBSERVER_THREAD_PRIO);
}

/**
  * @brief  Create and send a dynamic Web Page. This page contains the list of 
  *         running tasks and the number of page hits. 
  * @param  conn pointer on connection structure 
  * @retval None
  */
//void DynWebPage(struct netconn *conn)
//{
//  portCHAR PAGE_BODY[512];
//  portCHAR pagehits[10] = {0};

//  memset(PAGE_BODY, 0,512);

//  /* Update the hit count */
//  nPageHits++;
//  sprintf(pagehits, "%d", (int)nPageHits);
//  strcat(PAGE_BODY, pagehits);
//  strcat((char *)PAGE_BODY, "<pre><br>Name          State  Priority  Stack   Num" );
//  strcat((char *)PAGE_BODY, "<br>---------------------------------------------<br>");
//    
//  /* The list of tasks and their status */
//  osThreadList((unsigned char *)(PAGE_BODY + strlen(PAGE_BODY)));
//  strcat((char *)PAGE_BODY, "<br><br>---------------------------------------------");
//  strcat((char *)PAGE_BODY, "<br>B : Blocked, R : Ready, D : Deleted, S : Suspended<br>");

//  /* Send the dynamically generated page */
//  netconn_write(conn, PAGE_START, strlen((char*)PAGE_START), NETCONN_COPY);
//  netconn_write(conn, PAGE_BODY, strlen(PAGE_BODY), NETCONN_COPY);
//}
