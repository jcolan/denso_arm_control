#ifndef _STATUS_CODE_H_
#define _STATUS_CODE_H_

// Function return status

#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1
#define EXIT_PREEMPTED 2
#define EXIT_ABORTED 3

// Object number
#define ROBOT0 0
#define ROBOT1 1

enum STATUS
{
  STOPPED = 0,
  MOVING
};

// Robot Status
enum
{
  R_UNINITIALIZED = 0,
  R_CONNECTED,
  R_READY,
  R_SLAVE,
};

// ROBOT Actions
enum
{
  R_CONNECT = 0,
  R_DISCONNECT,
  R_MOTOR_ON,
  R_MOTOR_OFF,
  R_SLAVE_ON,
  R_SLAVE_OFF,
};

#endif
