#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "threads/init.h"
#include "threads/malloc.h"
#include "threads/synch.h"
#include "threads/thread.h"

#include "devices/timer.h"

#include "projects/automated_warehouse/aw_manager.h"
#include "projects/automated_warehouse/aw_message.h"
#include "aw_message.h"

struct robot *robots;
char *destination[15];
int robot1_required_payload; // 2
int robot1_unload;           // A
int robot_requied_payload[5];
int robot_unload[5];

// 지도에 대한 전역 상수
#define MAP_ROWS 6
#define MAP_COLS 6
#define WALL 'X'
int num_robots = 5; // 그냥 전역 변수로 선언, 나중에 바꿀거임.

// 지도 데이터 정의
char map[MAP_ROWS][MAP_COLS] = {
    {'X', 'X', 'A', 'X', 'X', 'X'},
    {'X', '1', ' ', '2', '3', '4'},
    {'B', ' ', ' ', ' ', ' ', ' '},
    {' ', ' ', ' ', ' ', ' ', ' '},
    {'X', '5', ' ', '6', '7', 'S'},
    {'X', 'X', 'C', 'X', 'X', 'W'}};
// 벽인지 맵 범위를 벗어났는지 판단하는 함수
int isBlocked(int row, int col)
{
    return (row < 0 || row >= MAP_ROWS || col < 0 || col >= MAP_COLS || map[row][col] == WALL);
}

// 주어진 방향으로 이동을 시도하여 이동 가능한지 확인하는 함수
int canMoveTo(int next_row, int next_col)
{
    return !isBlocked(next_row, next_col);
}

// 함수 선언
void process_robots_data();
void determine_robot_command(int robot_id, struct message robot_msg);
void find_target_position(int target, int *row, int *col);
void send_command_to_robot(int robot_id, int target_row, int target_col);
int calculate_direction(int current_row, int current_col, int target_row, int target_col);
bool can_move(int new_row, int new_col);
void update_robot_status(int idx);
void do_robot_task(int idx);

// test code for central control node thread
void test_cnt()
{
    while (1)
    {
        print_map(robots, 1);
        thread_sleep(1000);

        // 중앙 관제 노드가 먼저 블로킹될 필요가 없으므로, 로봇의 상태를 먼저 처리하고 언블록합니다.
        printf("Central node processes robot data\n");
        process_robots_data(); // 로봇 데이터 처리

        printf("Central node unblocks all robots\n");
        unblock_threads(); // 모든 로봇을 언블록

        // 이후에 중앙 관제 노드가 대기할 필요가 있다면 이 부분에서 블로킹합니다.
        printf("central node 잠깐 쉼\n");
        thread_sleep(100);
    }
}

void process_robots_data()
{
    // printf("process_robots_process()호출 완료");
    // 모든 로봇 현재 위치와 상태 정보를 수집
    for (int i = 0; i < num_robots; i++)
    {
        if (i >= num_robots || boxes_from_robots == NULL)
        {
            printf("Invalid robot index or uninitialized boxes_from_robots array\n");
            break;
        }

        if (boxes_from_robots[i].dirtyBit == 1) // 로봇의 데이터가 최신이면
        {
            struct message msg = boxes_from_robots[i].msg;
            printf("process_robots_data 정상적으로 호출됨");
            printf("Robot R%d at (%d, %d) current_payload: %d, required_payload: %d\n",
                   (i+1), msg.row, msg.col, msg.current_payload, msg.required_payload);
            boxes_from_robots[i].dirtyBit = 0; // 메시지 확인 후 dirtyBit 초기화

            determine_robot_command(i, msg);
        }
    }
}

// 로봇의 명령 설정
void determine_robot_command(int robot_id, struct message robot_msg)
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

    if ((robot_msg.row == 5 && robot_msg.col == 5) || (robot_msg.row == 4 && robot_msg.col == 5) ||
        (robot_msg.row == 3 && robot_msg.col == 5) || (robot_msg.row == 2 && robot_msg.col == 5) ||
        (robot_msg.row == 2 && robot_msg.col == 5) || (robot_msg.row == 2 && robot_msg.col == 4) ||
        (robot_msg.row == 2 && robot_msg.col == 3) || (robot_msg.row == 2 && robot_msg.col == 2) ||
        (robot_msg.row == 3 && robot_msg.col == 5) || (robot_msg.row == 3 && robot_msg.col == 4) ||
        (robot_msg.row == 3 && robot_msg.col == 3) || (robot_msg.row == 3 && robot_msg.col == 2) ||
        (robot_msg.row == 1 && robot_msg.col == 2) || (robot_msg.row == 4 && robot_msg.col == 2))
    {
        printf("1번 호출됨");
        find_target_position(robot_msg.required_payload, &target_row, &target_col);
        send_command_to_robot(robot_id, target_row, target_col);
    }
    else if (robot_msg.row == 1 && robot_msg.col == 1)
    {
        printf("2번 호출됨");
        robot_msg.current_payload = 1;
        if (robot_msg.current_payload == robot_msg.required_payload)
        {
            printf("1번 화물 적재 완료!");
            boxes_from_robots[robot_id].msg.required_payload = robot1_unload;
            // Now, move the robot to the unload destination defined by `robot1_unload`
            find_target_position((int)robot1_unload, &target_row, &target_col);
            send_command_to_robot(robot_id, target_row, target_col);
        }
        find_target_position(robot_msg.required_payload, &target_row, &target_col);
        send_command_to_robot(robot_id, target_row, target_col);
    }
    else if (robot_msg.row == 1 && robot_msg.col == 3)
    {
        printf("2번 호출됨");
        robot_msg.current_payload = 2;
        if (robot_msg.current_payload == robot_msg.required_payload)
        {
            printf("2번 화물 적재 완료!");
            boxes_from_robots[robot_id].msg.required_payload = robot1_unload;
            // Now, move the robot to the unload destination defined by `robot1_unload`
            find_target_position((int)robot1_unload, &target_row, &target_col);
            send_command_to_robot(robot_id, target_row, target_col);
        }
        find_target_position(robot_msg.required_payload, &target_row, &target_col);
        send_command_to_robot(robot_id, target_row, target_col);
    }
    else if (robot_msg.row == 1 && robot_msg.col == 4)
    {
        printf("2번 호출됨");
        robot_msg.current_payload = 3;
        if (robot_msg.current_payload == robot_msg.required_payload)
        {
            printf("3번 화물 적재 완료!");
            boxes_from_robots[robot_id].msg.required_payload = robot1_unload;
            // Now, move the robot to the unload destination defined by `robot1_unload`
            find_target_position((int)robot1_unload, &target_row, &target_col);
            send_command_to_robot(robot_id, target_row, target_col);
        }
        find_target_position(robot_msg.required_payload, &target_row, &target_col);
        send_command_to_robot(robot_id, target_row, target_col);
    }
    else if (robot_msg.row == 1 && robot_msg.col == 5)
    {
        printf("2번 호출됨");
        robot_msg.current_payload = 4;
        if (robot_msg.current_payload == robot_msg.required_payload)
        {
            printf("4번 화물 적재 완료!");
            boxes_from_robots[robot_id].msg.required_payload = robot1_unload;
            // Now, move the robot to the unload destination defined by `robot1_unload`
            find_target_position((int)robot1_unload, &target_row, &target_col);
            send_command_to_robot(robot_id, target_row, target_col);
        }
        find_target_position(robot_msg.required_payload, &target_row, &target_col);
        send_command_to_robot(robot_id, target_row, target_col);
    }
    else if (robot_msg.row == 4 && robot_msg.col == 1)
    {
        printf("2번 호출됨");
        robot_msg.current_payload = 5;
        if (robot_msg.current_payload == robot_msg.required_payload)
        {
            printf("5번 화물 적재 완료!");
            boxes_from_robots[robot_id].msg.required_payload = robot1_unload;
            // Now, move the robot to the unload destination defined by `robot1_unload`
            find_target_position((int)robot1_unload, &target_row, &target_col);
            send_command_to_robot(robot_id, target_row, target_col);
        }
        find_target_position(robot_msg.required_payload, &target_row, &target_col);
        send_command_to_robot(robot_id, target_row, target_col);
    }
    else if (robot_msg.row == 4 && robot_msg.col == 3)
    {
        printf("2번 호출됨");
        robot_msg.current_payload = 6;
        if (robot_msg.current_payload == robot_msg.required_payload)
        {
            printf("6번 화물 적재 완료!");
            boxes_from_robots[robot_id].msg.required_payload = robot1_unload;
            // Now, move the robot to the unload destination defined by `robot1_unload`
            find_target_position((int)robot1_unload, &target_row, &target_col);
            send_command_to_robot(robot_id, target_row, target_col);
        }
        find_target_position(robot_msg.required_payload, &target_row, &target_col);
        send_command_to_robot(robot_id, target_row, target_col);
    }
    else if (robot_msg.row == 4 && robot_msg.col == 4)
    {
        printf("2번 호출됨");
        robot_msg.current_payload = 7;
        if (robot_msg.current_payload == robot_msg.required_payload)
        {
            printf("7번 화물 적재 완료!");
            boxes_from_robots[robot_id].msg.required_payload = robot1_unload;
            // Now, move the robot to the unload destination defined by `robot1_unload`
            find_target_position((int)robot1_unload, &target_row, &target_col);
            send_command_to_robot(robot_id, target_row, target_col);
        }
        find_target_position(robot_msg.required_payload, &target_row, &target_col);
        send_command_to_robot(robot_id, target_row, target_col);
    }
    else if (robot_msg.row == 0 && robot_msg.col == 2)
    {
        printf("하역칸 A\n");
        printf("%d", robot_msg.required_payload);
        robot_msg.current_payload = 17;
        if (robot_msg.current_payload == robot_msg.required_payload)
        {
            printf("화물 하역 완료!");
            thread_exit();
        }
    }
    else if (robot_msg.row == 2 && robot_msg.col == 0)
    {
        printf("B 하역장\n");
        robot_msg.current_payload = 18;
        if (robot_msg.current_payload == robot_msg.required_payload)
        {
            printf("화물 하역 완료!");
            thread_exit();
        }
    }
    else if (robot_msg.row == 5 && robot_msg.col == 2)
    {
        printf("C 하역장\n");
        robot_msg.current_payload = 19;
        if (robot_msg.current_payload == robot_msg.required_payload)
        {
            printf("화물 하역 완료!");
            thread_exit();
        }
    }
    else
    {
        printf("3번 호출됨");

        find_target_position(robot_msg.required_payload, &target_row, &target_col);
        send_command_to_robot(robot_id, target_row, target_col);
    }

    // if ((robot_msg.current_payload == 0) || (robot_msg.current_payload != robot_msg.required_payload))
    // {
    //     // 목적지는 required_payload
    //     find_target_position(robot_msg.required_payload, &target_row, &target_col);
    //     send_command_to_robot(robot_id, target_row, target_col);
    // }
    // else if (robot_msg.current_payload == robot_msg.required_payload)
    // {
    //     printf("화물 적재 완료!");
    //     boxes_from_robots[robot_id].msg.required_payload = (int)robot1_unload;
    //     // Now, move the robot to the unload destination defined by `robot1_unload`
    //     find_target_position((int)robot1_unload, &target_row, &target_col);
    //     send_command_to_robot(robot_id, target_row, target_col);
    // }
    // else if (robots[robot_id].row == target_row && robots[robot_id].col == target_col)
    // {
    //     // 로봇이 목적지에 도달하고 화물을 적재했다면
    //     printf("Robot R%d has completed its task at (%d, %d).\n", robot_id, target_row, target_col);
    //     exit(0);
    // }
    // else
    // {
    //     // 대기 위치로 이동하도록 명령
    //     printf("이 시발이 호출됨");
    //     send_command_to_robot(robot_id, 5, 5);
    // }
}

void find_target_position(int target, int *row, int *col)
{
    // printf("find_target_position\n");
    // printf("%d\n", target);
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
    case 65: // 'A'  requied_payload가 변경되는 로직 필요
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

void send_command_to_robot(int robot_id, int target_row, int target_col)
{
    printf("send_command_to_robot, %d, %d\n\n", robot_id, target_row);
    int direction = calculate_direction(robots[robot_id].row, robots[robot_id].col, target_row, target_col);
    printf("%d", direction);
    boxes_from_central_control_node[robot_id].msg.cmd = direction;
    boxes_from_central_control_node[robot_id].dirtyBit = 1;
}

int isWall(int row, int col)
{
    // Example matrix data could be defined here or accessed from elsewhere
    char grid[6][6] = {
        {'X', 'X', 'A', 'X', 'X', 'X'},
        {'X', '1', ' ', '2', '3', '4'},
        {'B', ' ', ' ', ' ', ' ', ' '},
        {'X', ' ', ' ', ' ', ' ', ' '},
        {'X', '5', ' ', '6', '7', 'S'},
        {'X', 'X', 'C', 'X', 'X', 'W'}};

    return grid[row][col] == 'X'; // Returns true if there is a wall
}

// 방향 계산 함수
int calculate_direction(int current_row, int current_col, int target_row, int target_col)
{
    printf("calculate_direction\n");
    //
    int vertical_diff = target_row - current_row;
    int horizontal_diff = target_col - current_col;

    // 방향별로 차이를 비교하여 이동 우선순위 설정
    int directions[4][2] = {
        {-1, 0}, // 위로 이동
        {0, -1}, // 왼쪽으로 이동
        {1, 0},  // 아래로 이동
        {0, 1}   // 오른쪽으로 이동
    };

    // 순차적으로 모든 방향을 시도
    for (int i = 0; i < 4; ++i)
    {
        int next_row = current_row + directions[i][0];
        int next_col = current_col + directions[i][1];

        if (canMoveTo(next_row, next_col))
        {
            // 해당 방향을 숫자로 반환
            if (directions[i][0] == -1 && directions[i][1] == 0)
                return 1; // 위로 이동
            else if (directions[i][0] == 0 && directions[i][1] == -1)
                return 3; // 왼쪽으로 이동
            else if (directions[i][0] == 1 && directions[i][1] == 0)
                return 2; // 아래로 이동
            else if (directions[i][0] == 0 && directions[i][1] == 1)
                return 4; // 오른쪽으로 이동
        }
    }

    // 모든 방향이 막힌 경우에는 움직이지 않음
    return 5; // 그대로 유지
}

// test code for robot thread
void test_thread(void *aux)
{
    int idx = *((int *)aux) - 2; // 스레드 2번
    int test = 0;
    while (1)
    {
        printf("Thread R%d starts work %d\n", (idx + 1), test++); // 작업 시작을 알리는 메시지
        thread_sleep(idx * 1000);                                 // 인덱스에 따른 지연 시간
        do_robot_task(idx);                                       // 로봇 작업 수행
        printf("Thread R%d blocks\n", (idx + 1));                 // 블로킹 전 메시지 출력
        block_thread();                                           // 작업 완료 후 블록 상태로 전환
        printf("Thread R%d unblocked\n", (idx + 1));              // 언블록 후 메시지 출력
    }
}

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
                printf("Robot R%d moved up to (%d, %d)\n", idx, robots[idx].row, robots[idx].col);
            }
            break;
        case 2: // 아래
            if (can_move(robots[idx].row + 1, robots[idx].col))
            { // 아래로 이동 가능한지 확인
                robots[idx].row += 1;
                printf("Robot R%d moved down to (%d, %d)\n", idx, robots[idx].row, robots[idx].col);
            }
            break;
        case 3: // 왼쪽
            if (can_move(robots[idx].row, robots[idx].col - 1))
            { // 왼쪽으로 이동 가능한지 확인
                robots[idx].col -= 1;
                printf("Robot R%d moved left to (%d, %d)\n", idx, robots[idx].row, robots[idx].col);
            }
            break;
        case 4: // 오른쪽
            if (can_move(robots[idx].row, robots[idx].col + 1))
            { // 오른쪽으로 이동 가능한지 확인
                robots[idx].col += 1;
                printf("Robot R%d moved right to (%d, %d)\n", idx, robots[idx].row, robots[idx].col);
            }
            break;
        case 5: // 동작없음
            printf("Robot R%d is waiting\n", idx);

            // 아무 동작도 하지 않음
            break;
        default:
            printf("Unknown command %d\n", msg.cmd);
            break;
        }

        // 메시지 처리 후 dirtyBit 초기화
        boxes_from_central_control_node[idx].dirtyBit = 0;
    }
    // 로봇 상태 업데이트 후 중앙 관제 노드에 보고
    update_robot_status(idx);
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
    printf("로봇1의 정보를 업데이트합니다.\n");
    boxes_from_robots[idx].msg.row = robots[idx].row;
    boxes_from_robots[idx].msg.col = robots[idx].col;
    // payload 상태 업데이트 필요한 경우 처리
    printf("확인 차     %d\n", boxes_from_robots[idx].msg.row);
    boxes_from_robots[idx].dirtyBit = 1; // 변경 사항을 알림
}

// entry point of simulator
void run_automated_warehouse(char **argv)
{
    init_automated_warehouse(argv); // do not remove this

    destination[0] = argv[2]; //2A:4C:2B:2C:3A

    robot1_required_payload = destination[0][0] - '0'; // 2
    robot1_unload = destination[0][1] - '0';           // A

    robot_requied_payload[0] = destination[0][0] - '0'; //2
    robot_requied_payload[1] = destination[0][3] - '0'; //4
    robot_requied_payload[2] = destination[0][6] - '0';
    robot_requied_payload[3] = destination[0][9] - '0';
    robot_requied_payload[4] = destination[0][12] - '0';
    robot_unload[0] = destination[0][1] - 0 ; // 65
    robot_unload[1] = destination[0][4] - 0 ; // 
    robot_unload[2] = destination[0][7] - 0 ; // 
    robot_unload[3] = destination[0][10] - 0 ; //
    robot_unload[4] = destination[0][13] - 0 ; // 

    printf("%d", robot_unload[0]);
    
    // Inside run_automated_warehouse()
    boxes_from_robots = malloc(sizeof(struct message_box) * num_robots);
    if (boxes_from_robots == NULL)
    {
        printf("Memory allocation failed for boxes_from_robots\n");
    }

    // Initialize all elements of boxes_from_robots
    for (int i = 0; i < num_robots; i++)
    {
        boxes_from_robots[i].dirtyBit = 0; // or any default value
        // Further initialization if needed
        boxes_from_robots[i].msg.required_payload = robot1_required_payload; // 일단 이렇게
    }

    boxes_from_central_control_node = malloc(sizeof(struct message_box) * num_robots);
    if (boxes_from_central_control_node == NULL)
    {
        // 메모리 할당 실패 처리
        printf("Memory allocation failed for boxes_from_central_control_node\n");
    }

    // 각 메시지 박스 초기화
    for (int i = 0; i < num_robots; i++)
    {
        boxes_from_central_control_node[i].dirtyBit = 0;
        // 필요 시 추가 초기화
    }

    printf("implement automated warehouse!\n");

    // test case robots
    robots = malloc(sizeof(struct robot) * 5);
    setRobot(&robots[0], "R1", 5, 5, 4, 0);
    setRobot(&robots[1], "R2", 0, 2, 0, 0);
    setRobot(&robots[2], "R3", 1, 1, 1, 1);
    setRobot(&robots[3], "R4", 5, 5, 0, 0);
    setRobot(&robots[3], "R5", 5, 5, 0, 0);

    // example of create thread
    tid_t *threads = malloc(sizeof(tid_t) * 6);
    int idxs[5] = {1, 2, 3, 4, 5};
    threads[0] = thread_create("CNT", 0, &test_cnt, NULL);
    threads[1] = thread_create("R1", 0, &test_thread, &idxs[1]);
    threads[2] = thread_create("R2", 0, &test_thread, &idxs[2]);
    threads[3] = thread_create("R3", 0, &test_thread, &idxs[3]);
    threads[4] = thread_create("R4", 0, &test_thread, &idxs[4]);
    threads[5] = thread_create("R5", 0, &test_thread, &idxs[5]);

}

/*
docker run -it --name pintos --mount type=bind,source=/Users/sj.kang/Desktop/pintos,target=/root/pintos --platform linux/amd64 ubuntu:bionic
컨테이너 시작하기
docker start /pintos
컨테이너 내부로 진입하기
docker exec -it /pintos bash

cd root/pintos/threads
make
cd build
chmod +x ../../utils/pintos
../../utils/pintos automated_warehouse 1 2A:3B:4C

간단 버전!!
docker exec -it /pintos bash
cd root/pintos/threads
make
../utils/pintos automated_warehouse 1 2A:3B:4C


컨테이너 종료하기
docker stop /pintos
*/