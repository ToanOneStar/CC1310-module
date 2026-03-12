#include <stdint.h>
#include <stdio.h>
#include <pthread.h>
#include <ti/sysbios/BIOS.h>
#include <ti/drivers/Board.h>

extern void *mainThread(void *arg0);

#define THREADSTACKSIZE 2048

/* Debug: check thread creation */
volatile int g_attr_status = 0;
volatile int g_thread_create_status = 0;

int main(void) {
  printf("hello main");

  pthread_t thread;
  pthread_attr_t attrs;
  struct sched_param priParam;
  int retc;

  /* Call driver init functions */
  Board_init();

  /* Initialize the attributes structure with default values */
  pthread_attr_init(&attrs);

  /* Set priority, detach state, and stack size attributes */
  priParam.sched_priority = 1;
  retc = pthread_attr_setschedparam(&attrs, &priParam);
  retc |= pthread_attr_setdetachstate(&attrs, PTHREAD_CREATE_DETACHED);
  retc |= pthread_attr_setstacksize(&attrs, THREADSTACKSIZE);
  g_attr_status = retc;

  /* If attrs fail, fall back to default attributes */
  if (retc != 0) {
    retc = pthread_create(&thread, NULL, mainThread, NULL);
  } else {
    retc = pthread_create(&thread, &attrs, mainThread, NULL);
  }
  g_thread_create_status = retc;
  if (retc != 0) {
    /* pthread_create() failed */
    while (1) {
    }
  }

  BIOS_start();

  return (0);
}
