#include "projects/automated_warehouse/aw_thread.h"
#include "threads/thread.h"
#include "threads/interrupt.h"
#include "threads/synch.h"
#include "kernel/list.h"

//
// You need to read carefully thread/synch.h and thread/synch.c
//
// In the code, a fucntion named "sema_down" implements blocking thread and
// makes list of blocking thread
//
// And a function named "sema_up" implements unblocing thread using blocking list
//
// You must implement blocking list using "blocking_threads" in this code.
// Then you can also implement unblocking thread.
//

struct list blocked_threads;

void init_blocked_threads(void)
{
    list_init(&blocked_threads);
}

/**
 * A function unblocking all blocked threads in "blocked_threads"
 * It must be called by robot threads
 */
void block_thread()
{
    enum intr_level old_level = intr_disable();

    // Add the current thread to the blocked_threads list
    printf("\n블럭 리스트에 추가합니다\n");
    list_push_back(&blocked_threads, &thread_current()->elem);

    // Block the current thread
    printf("\n블럭 시킵니다\n");
    thread_block();
    intr_set_level(old_level);
}

/**
 * A function unblocking all blocked threads in "blocked_threads"
 * It must be called by central control thread
 */
void unblock_threads()
{
    enum intr_level old_level = intr_disable();

    // Iterate through all blocked threads and unblock them
    while (!list_empty(&blocked_threads))
    {
        struct thread *t = list_entry(list_pop_front(&blocked_threads), struct thread, elem);
        printf("언 블럭 시킵니다\n");
        thread_unblock(t);
    }

    intr_set_level(old_level);
}