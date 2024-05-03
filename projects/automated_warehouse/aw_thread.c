#include "projects/automated_warehouse/aw_thread.h"

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


struct list blocked_threads; //멀티쓰레딩 환경에서 특정 스레드들이 블럭될 때 관리하기 위한 구조체

/**
 * 아래 함수들은 로봇 쓰레드와 중앙 관제 노드 쓰레드간의 동기화 및 상태 관리를 위한 구조 제공
 * 로봇 쓰레드는 작업을 완료 후 block_thread를 호출하여 자신을 블럭, 
 * 중앙 관제 노드는 모든 로봇의 데이터를 처리한 후 unblock_threads 호출하여 모든 로봇 쓰레드를 다시 활성화
*/

/**
 * A function unblocking all blocked threads in "blocked_threads" 
 * It must be called by robot threads
 * "blocked_threads"안에 안에 있는 모든 블럭된 쓰레드를 unblock 시킴
 * robot 쓰레드에 의해 불려야 함
 */
void block_thread(){
    // You must implement this

    // Code below is example
    // enum intr_level old_level;
    // old_level = intr_disable ();
    // thread_block ();
    // intr_set_level (old_level);

    //내가 적은 코드
    enum intr_level old_level = intr_disable(); // 인터럽트를 비활성화하여 선점을 방지
    struct thread *current = thread_current(); //현재 쓰레드 객체를 얻음
    ASSERT(current != NULL);
    list_push_back(&blocked_threads, &current->elem); // 현재 쓰레드를 차단 쓰레드 목록에 추가
    thread_block(); //쓰레드를 차단 상태로 전환
    intr_set_level(old_level); //이전 인터럽트 레벨로 복구시킴
}

/**
 * A function unblocking all blocked threads in "blocked_threads" 
 * It must be called by central control thread
 * 컨트롤 쓰레드에 의해 불림
 */
// struct list blocked_threads에 저장되어 있는 Block된 스레드들을 전부 Unblock
void unblock_threads(){
    enum intr_level old_level = intr_disable(); //인터럽트 비활성화
    struct list_elem *e;

    for (e = list_begin(&blocked_threads); e != list_end(&blocked_threads); e = list_next(e)){
        struct thread *t = list_entry(e, struct thread, elem);
        thread_unblock(t);
    }

    list_clear(&blocked_threads); // 차단된 쓰레드 목록을 비움
    intr_set_level(old_level); // 인터럽트 레벨 복귀
}

void list_clear(struct list* list) {
    while (!list_empty(list)) {
        struct list_elem *e = list_pop_front(list);
        // 필요한 경우, list_elem을 포함하는 구조체를 해제
    }
}

