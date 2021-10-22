#include <stdio.h>
#include <stdlib.h>
#include "raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis.h"
#include "raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_private.h"
#include "rtwtypes.h"
#include "limits.h"
#include "rt_nonfinite.h"
#include "MW_raspi_init.h"
#include "MW_Pyserver_control.h"
#include "linuxinitialize.h"
#define UNUSED(x)                      x = x
#define NAMELEN                        16

/* Function prototype declaration*/
void exitFcn(int sig);
void *terminateTask(void *arg);
void *baseRateTask(void *arg);
void *subrateTask(void *arg);
volatile boolean_T stopRequested = false;
volatile boolean_T runModel = true;
sem_t stopSem;
sem_t baserateTaskSem;
sem_t subrateTaskSem[1];
int taskId[1];
pthread_t schedulerThread;
pthread_t baseRateThread;
void *threadJoinStatus;
int terminatingmodel = 0;
pthread_t subRateThread[1];
pthread_mutex_t rateTaskFcnRunningMutex[2];
int subratePriority[1];
int testForRateOverrun(int rateID);
void *subrateTask(void *arg)
{
  int tid = *((int *) arg);
  int subRateId;
  subRateId = tid + 1;
  while (runModel) {
    sem_wait(&subrateTaskSem[tid]);
    if (terminatingmodel)
      break;

#ifdef MW_RTOS_DEBUG

    printf(" -subrate task %d semaphore received\n", subRateId);

#endif

    pthread_mutex_lock(&rateTaskFcnRunningMutex[tid+1]);
    raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_step(subRateId);

    /* Get model outputs here */
    pthread_mutex_unlock(&rateTaskFcnRunningMutex[tid+1]);
  }

  pthread_exit((void *)0);
  return NULL;
}

void *baseRateTask(void *arg)
{
  runModel = (rtmGetErrorStatus(raspberrypi_multicore_MPCtes_M) == (NULL)) &&
    !rtmGetStopRequested(raspberrypi_multicore_MPCtes_M);
  while (runModel) {
    sem_wait(&baserateTaskSem);
    pthread_mutex_lock(&rateTaskFcnRunningMutex[0]);

#ifdef MW_RTOS_DEBUG

    printf("*base rate task semaphore received\n");
    fflush(stdout);

#endif

    if (rtmStepTask(raspberrypi_multicore_MPCtes_M, 1)
        ) {
      testForRateOverrun(1);
      sem_post(&subrateTaskSem[0]);
    }

    /* External mode */
    {
      boolean_T rtmStopReq = false;
      rtExtModePauseIfNeeded(raspberrypi_multicore_MPCtes_M->extModeInfo, 2,
        &rtmStopReq);
      if (rtmStopReq) {
        rtmSetStopRequested(raspberrypi_multicore_MPCtes_M, true);
      }

      if (rtmGetStopRequested(raspberrypi_multicore_MPCtes_M) == true) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M, "Simulation finished");
        break;
      }
    }

    /* External mode */
    {
      boolean_T rtmStopReq = false;
      rtExtModeOneStep(raspberrypi_multicore_MPCtes_M->extModeInfo, 2,
                       &rtmStopReq);
      if (rtmStopReq) {
        rtmSetStopRequested(raspberrypi_multicore_MPCtes_M, true);
      }
    }

    raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_step(0);

    /* Get model outputs here */
    rtExtModeCheckEndTrigger();
    pthread_mutex_unlock(&rateTaskFcnRunningMutex[0]);
    stopRequested = !((rtmGetErrorStatus(raspberrypi_multicore_MPCtes_M) ==
                       (NULL)) && !rtmGetStopRequested
                      (raspberrypi_multicore_MPCtes_M));
    runModel = !stopRequested;
  }

  terminateTask(arg);
  pthread_exit((void *)0);
  return NULL;
}

void exitFcn(int sig)
{
  UNUSED(sig);
  rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M, "stopping the model");
  runModel = 0;
}

void *terminateTask(void *arg)
{
  UNUSED(arg);
  terminatingmodel = 1;

  {
    int i;

    /* Signal all periodic tasks to complete */
    for (i=0; i<1; i++) {
      CHECK_STATUS(sem_post(&subrateTaskSem[i]), 0, "sem_post");
      CHECK_STATUS(sem_destroy(&subrateTaskSem[i]), 0, "sem_destroy");
    }

    /* Wait for all periodic tasks to complete */
    for (i=0; i<1; i++) {
      CHECK_STATUS(pthread_join(subRateThread[i], &threadJoinStatus), 0,
                   "pthread_join");
    }

    runModel = 0;
  }

  MW_killPyserver();
  mwRaspiTerminate();

  /* Disable rt_OneStep() here */

  /* Terminate model */
  raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_terminate();
  rtExtModeShutdown(2);
  sem_post(&stopSem);
  return NULL;
}

int testForRateOverrun(int rateID)
{
  if (pthread_mutex_trylock(&rateTaskFcnRunningMutex[rateID]) == 0) {
    pthread_mutex_unlock(&rateTaskFcnRunningMutex[rateID]);
    return 0;                          /* Not overruning this rate */
  } else {
    reportOverrun(rateID);
    return 1;                          /* Overruning this rate */
  }
}

int main(int argc, char **argv)
{
  UNUSED(argc);
  UNUSED(argv);
  subratePriority[0] = 39;
  mwRaspiInit();
  MW_launchPyserver();
  rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M, 0);
  rtExtModeParseArgs(argc, (const char_T **)argv, NULL);

  /* Initialize model */
  raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_initialize();

  /* External mode */
  rtSetTFinalForExtMode(&rtmGetTFinal(raspberrypi_multicore_MPCtes_M));
  rtExtModeCheckInit(2);

  {
    boolean_T rtmStopReq = false;
    rtExtModeWaitForStartPkt(raspberrypi_multicore_MPCtes_M->extModeInfo, 2,
      &rtmStopReq);
    if (rtmStopReq) {
      rtmSetStopRequested(raspberrypi_multicore_MPCtes_M, true);
    }
  }

  rtERTExtModeStartMsg();

  /* Call RTOS Initialization function */
  myRTOSInit(0.005, 1);

  /* Wait for stop semaphore */
  sem_wait(&stopSem);

#if (MW_NUMBER_TIMER_DRIVEN_TASKS > 0)

  {
    int i;
    for (i=0; i < MW_NUMBER_TIMER_DRIVEN_TASKS; i++) {
      CHECK_STATUS(sem_destroy(&timerTaskSem[i]), 0, "sem_destroy");
    }
  }

#endif

  {
    int idxMutex;
    for (idxMutex=0; idxMutex<2; idxMutex++)
      pthread_mutex_destroy(&rateTaskFcnRunningMutex[idxMutex]);
  }

  return 0;
}
