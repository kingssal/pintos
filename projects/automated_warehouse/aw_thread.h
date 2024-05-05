#ifndef _PROJECTS_PROJECT1_AW_THREAD_H__
#define _PROJECTS_PROJECT1_AW_THREAD_H__

//스레딩 유틸리티 또는 작업을 위한 구현

#include "threads/interrupt.h"
#include "threads/synch.h"
#include "threads/thread.h"
#include "list.h"

extern struct list blocked_threads;

void block_thread(void);

void unblock_threads(void);

void list_clear(struct list* list);



#endif