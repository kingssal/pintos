#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "threads/init.h"
#include "threads/malloc.h"
#include "threads/synch.h"
#include "threads/thread.h"

#include "devices/timer.h"

#include "projects/automated_warehouse/aw_manager.h"
#include "projects/automated_warehouse/automated_warehouse.h"
#include "projects/automated_warehouse/aw_message.h"

#define _POSIX_C_SOURCE 200809L

struct robot *robots;
char *destination[5];
int num_robots = 0;

// test code for central control node thread
// 중앙 관제 노드
void test_cnt()
{
    while (1)
    {
        print_map(robots, num_robots); // 로봇들의 현재 위치와 상태 출력
        thread_sleep(1000);            // 현재 실행 중인 스레드를 1000ms(1초) 동안 정지, 다른 스레드에 할당용
        block_thread();                // 로봇의 쓰레드를 블럭
        process_robots_data();         // 로봇 데이터 처리, 모든 처리가 끝나면 시뮬 종료 해야함
        unblock_threads();             // 로봇 쓰레드를 다시 활성화
    }
}

// 로봇 데이터 처리
void process_robots_data()
{
    // 모든 로봇 현재 위치와 상태 정보를 수집
    for (int i = 0; i < num_robots; i++)
    {
        if (boxes_from_robots[i].dirtyBit == 1)
        {
            struct message msg = boxes_from_robots[i].msg;
            printf("Robot R%d at (%d, %d) current_payload : %d, required_payload : %d", i, msg.row, msg.col, msg.current_payload, msg.required_payload);
            boxes_from_robots[i].dirtyBit = 0; // 메시지 확인 후 dirtyBit 초기화

            // 로봇에 대한 명령 설정
            determine_robot_cmd(i, msg);
        }
    }
}

// 로봇의 명령 설정
void determine_robot_cmd(int robot_id, struct message robot_msg)
{
    /*
            {'X', 'X', 'A', 'X', 'X', 'X', 'X' },
            {'X', '1', ' ', '2', '3', '4', 'X' },
            {'B', ' ', ' ', ' ', ' ', ' ', 'X' },
            {'X', ' ', ' ', ' ', ' ', ' ', 'X' },
            {'X', '5', ' ', '6', '7', 'S', 'X' },
            {'X', 'X', 'C', 'X', 'X', 'W', 'X' }
        1. current payload = 0,  (A,B,C,W,S,빈칸)에 있을 때
        2. current payload != required payload 일때
        3. 대기
        4. 로봇이 장애물을 만나서 블럭 됐을 때
        5. 로봇이 임무를 완성했을 때
    */

    // 목표 위치 계산 로직, 아래 if문들 아래로 들어감
    int target_row = 0, target_col = 0;

    int target_char =  (int)destination[robot_id][1];         // 목적지 문자 A
    int target_number = destination[robot_id][0] - '0'; // 목적지 숫자 2

    if (robot_msg.current_payload == 0)
    {
        // 목적지의 위치를 파악하고 장애물을 파악하고 현재 위치로부터 상하좌우 명령
        // 목적지는 required_payload 위치가 됨
        find_target_position(robot_msg.required_payload, &target_row, &target_col);
        send_cmd_to_robot(robot_id, target_row, target_col);
    }
    else if (robot_msg.current_payload != robot_msg.required_payload)
    {
        // 1과 동일하게 한다
        find_target_position(robot_msg.required_payload, &target_row, &target_col);
        send_cmd_to_robot(robot_id, target_row, target_col);
    }
    else if (robot_msg.current_payload == robot_msg.required_payload)
    {
        printf("화물 적재 완료!");
        find_target_position(target_char, &target_row, &target_col);
        send_cmd_to_robot(robot_id, target_row, target_col);
    }
    else if (robots[robot_id].row == target_row && robots[robot_id].col == target_col)
    {
        find_target_position(target_char, &target_row, &target_col);
        // 로봇이 목적지에 도달하고 화물을 적재했다면
        printf("Robot R%d has completed its task at (%d, %d).\n", robot_id, target_row, target_col);
    }
    else
    {
        // 대기 위치로 이동하도록 명령
        send_cmd_to_robot(robot_id, 5, 5);
    }
}

// 지도 상의 위치를 찾는 함수
void find_target_position(int target, int *row, int *col)
{
    switch (target)
    {
    case 1:
        *row = 1;
        *col = 1; // 위치 1은 (1,1)
        break;
    case 2:
        *row = 3;
        *col = 1; // 위치 2는 (3,1)
        break;
    case 3:
        *row = 4;
        *col = 1; // 위치 3는 (4,1)
        break;
    case 4:
        *row = 5;
        *col = 1; // 위치 4는 (5,1)
        break;
    case 5:
        *row = 1;
        *col = 4; // 위치 5는 (1,4)
        break;
    case 6:
        *row = 3;
        *col = 4; // 위치 6는 (3,4)
        break;
    case 7:
        *row = 4;
        *col = 4; // 위치 7는 (4,4)
        break;
    case 65: // 'A'
        *row = 0;
        *col = 2;
        break;
    case 66:
        *row = 2;
        *col = 0;
        break;
    case 67:
        *row = 5;
        *col = 2;
        break;
    default:
        break;
    }
}

// 로봇 명령을 실제로 로봇에게 보냄
void send_cmd_to_robot(int robot_id, int target_row, int target_col)
{
    int direction = calculate_direction(robots[robot_id].row, robots[robot_id].col, target_row, target_col);
    boxes_from_central_control_node[robot_id].msg.cmd = direction;
    boxes_from_central_control_node[robot_id].dirtyBit = 1;
}

// 방향 계산 함수
int calculate_direction(int current_row, int current_col, int target_row, int target_col)
{
    // 간단한 방향 계산 로직: 단순 예시로 위/아래/왼쪽/오른쪽 이동 결정
    if (current_row < target_row)
        return 2; // 아래로 이동
    if (current_row > target_row)
        return 1; // 위로 이동
    if (current_col < target_col)
        return 4; // 오른쪽으로 이동
    if (current_col > target_col)
        return 3; // 왼쪽으로 이동
    return 5;     // 현위치 대기
}

// test code for robot thread
// 로봇 쓰레드
void test_thread(void *aux)
{
    int idx = *((int *)aux);
    int test = 0;
    while (1)
    {
        printf("thread %d : %d\n", idx, test++); // 로봇 작업 번호 출력
        thread_sleep(idx * 1000);                // 인덱스에 따라 다른 시간 간격(1초씩) 슬립
        do_robot_task(idx);                      // 로봇 작업 수행
        block_thread();                          // 작업 수행 후 블럭으로 전환
    }
}

// 로봇 작업
void do_robot_task(int idx)
{
    // 로봇 별 메시지 박스 접근
    struct message msg = boxes_from_central_control_node[idx].msg;

    if (boxes_from_central_control_node[idx].dirtyBit == 1) // 메시지가 새로운지 확인
    {
        // 메시지 해석 및 동작 결정
        switch (msg.cmd)
        {
        case 1: // 위
            if (can_move(robots[idx].row - 1, robots[idx].col))
            {
                robots[idx].row -= 1;
            }
            break;
        case 2: // 아래
            if (can_move(robots[idx].row + 1, robots[idx].col))
            { // 아래로 이동 가능한지 확인
                robots[idx].row += 1;
            }
            break;
        case 3: // 왼쪽
            if (can_move(robots[idx].row, robots[idx].col - 1))
            { // 왼쪽으로 이동 가능한지 확인
                robots[idx].col -= 1;
            }
            break;
        case 4: // 오른쪽
            if (can_move(robots[idx].row, robots[idx].col + 1))
            { // 오른쪽으로 이동 가능한지 확인
                robots[idx].col += 1;
            }
            break;
        case 5: // 동작없음
            // 아무 동작도 하지 않음
            break;
        default:
            printf("Unknown command %d\n", msg.cmd);
            break;
        }

        // 메시지 처리 후 dirtyBit 초기화
        boxes_from_central_control_node[idx].dirtyBit = 0;

        // 로봇 상태 업데이트 후 중앙 관제 노드에 보고
        update_robot_status(idx);
    }
}

bool can_move(int new_row, int new_col)
{
    // 이동하려는 위치에 다른 로봇이 있는지 확인
    for (int i = 0; i < num_robots; i++)
    {
        if (robots[i].row == new_row && robots[i].col == new_col)
        {
            return false; // 다른 로봇이 위치하고 있으면 이동 불가
        }
    }

    // 이동하려는 위치에 벽이 있는지 확인
    if ((new_row == 0 && new_col == 0) ||
        (new_row == 0 && new_col == 1) ||
        (new_row == 0 && new_col == 3) ||
        (new_row == 0 && new_col == 4) ||
        (new_row == 0 && new_col == 5) ||
        (new_row == 1 && new_col == 0) ||
        (new_row == 3 && new_col == 0) ||
        (new_row == 4 && new_col == 0) ||
        (new_row == 5 && new_col == 0) ||
        (new_row == 5 && new_col == 1) ||
        (new_row == 5 && new_col == 3) ||
        (new_row == 5 && new_col == 4))
    {
        return false;
    }

    return true;
}

// 로봇의 상태를 중앙 관제 노드에 업데이트
void update_robot_status(int idx)
{
    boxes_from_robots[idx].msg.row = robots[idx].row;
    boxes_from_robots[idx].msg.col = robots[idx].col;
    // payload 상태 업데이트 필요한 경우 처리
    boxes_from_robots[idx].dirtyBit = 1; // 변경 사항을 알림
}

// entry point of simulator
void run_automated_warehouse(char **argv)
{
    init_automated_warehouse(argv); // do not remove this

    // 목적지
    char *input_str = strdup(argv[2]);
    char *token;
    char *rest = input_str;

    while ((token = strtok_r(rest, ":", &rest)) != NULL && num_robots < 5)
    {
        destination[num_robots] = strdup(token); // 각 목적지를 저장
        num_robots++;                            // 로봇의 수가 결정됨
    }

    printf("implement automated warehouse!\n");

    // test case robots
    // 로봇 객체 배열 동적 할당
    robots = malloc(sizeof(struct robot) * 4);
    setRobot(&robots[0], "R1", 5, 5, 0, 0);
    // setRobot(&robots[1], "R2", 0, 2, 0, 0);
    // setRobot(&robots[2], "R3", 1, 1, 1, 1);
    // setRobot(&robots[3], "R4", 5, 5, 0, 0);

    // 로봇의 required_payload 설정
    for (int i = 0; i < num_robots; i++)
    {
        robots[i].required_payload = (int)destination[i][0];
    }

    // example of create thread
    tid_t *threads = malloc(sizeof(tid_t) * 4);
    int idxs[4] = {1, 2, 3, 4};
    threads[0] = thread_create("CNT", 0, &test_cnt, NULL);       // 중앙 관제 노드
    threads[1] = thread_create("R1", 0, &test_thread, &idxs[1]); // R1 쓰레드
    // threads[2] = thread_create("R2", 0, &test_thread, &idxs[2]);
    // threads[3] = thread_create("R3", 0, &test_thread, &idxs[3]);

    // if you want, you can use main thread as a central control node
}
