#include <stdio.h>
#include <string.h>

#include "threads/init.h"
#include "threads/malloc.h"
#include "threads/synch.h"
#include "threads/thread.h"

#include "devices/timer.h"

#include "projects/automated_warehouse/aw_manager.h"

struct robot* robots;

// test code for central control node thread
// 중앙 관제 노드 쓰레드
void test_cnt(){
        while(1){
                print_map(robots, 1); //로봇들의 현재 위치와 상태를 출력
                thread_sleep(1000);     // 1초 간격으로 업데이트
                block_thread();         // 로봇 장업 완료 후 블럭 상태로 전환
                //process_robots_data();        //로봇 데이터 처리 함수, 구현 필요
                unblock_threads();      // 모든 로봇 쓰레드를 다시 활성화
        }
}

// test code for robot thread
// 로봇 쓰레드 
void test_thread(void* aux){ 
        int idx = *((int *)aux);
        int test = 0;
        while(1){
                printf("로봇 thread %d : %d\n", idx, test++);   // 로봇의 작업 번호 출력
                thread_sleep(idx * 1000);       // 인덱스에 따라 다른 시간 간격으로 슬립
                do_robot_task(idx);  //로봇 작업 수행 함수, 구현 필요
                block_thread();         //작업 완료 후 쓰레드를 블럭 상태로 전환
        }
}

//로봇이 어떻게 움직이는지 구현
void do_robot_task(int idx){
        

}

// entry point of simulator
void run_automated_warehouse(char **argv)
{
        init_automated_warehouse(argv); // do not remove this

        printf("implement automated warehouse!\n");

        // test case robots
        // 로봇 객체 생성 및 초기화
        robots = malloc(sizeof(struct robot) * 4);
        setRobot(&robots[0], "R1", 5, 5, 0, 0); //처음은 한 개로 시작하자!
        // setRobot(&robots[1], "R2", 0, 2, 0, 0);
        // setRobot(&robots[2], "R3", 1, 1, 1, 1);
        // setRobot(&robots[3], "R4", 5, 5, 0, 0);

        // example of create thread
        // 스레드 생성
        tid_t* threads = malloc(sizeof(tid_t) * 4);
        int idxs[4] = {1, 2, 3, 4};
        threads[0] = thread_create("CNT", 0, &test_cnt, NULL); //중앙 관제 노드 용
        threads[1] = thread_create("R1", 0, &test_thread, &idxs[1]);
        // threads[2] = thread_create("R2", 0, &test_thread, &idxs[2]);
        // threads[3] = thread_create("R3", 0, &test_thread, &idxs[3]);

        // if you want, you can use main thread as a central control node
        
}