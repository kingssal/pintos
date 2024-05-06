#ifndef _PROJECTS_PROJECT1_AW_MESSAGE_H__
#define _PROJECTS_PROJECT1_AW_MESSAGE_H__

/**
 * For easy to implement, combine robot and central control node message
 * If you want to modify message structure, don't split it
 */
struct message {
    //
    // 중앙 관제 노드에게 보내는 메시지
    //
    /** 현재 로봇의 row */
    int row;
    /** 현재 로봇의 column */
    int col;
    /** 현재 로봇의 current_payload */
    int current_payload;
    /** required paylod of robot */
    int required_payload;

    //
    // 로봇에게 보내는 메시지
    //
    /** next command for robot */
    int cmd;
};

/** 
 * Simple message box which can receive only one message from sender
 * 센더로부터 오직 한개의 메시지를 받을 수 있는 메시지 박스
*/
struct message_box {
    /** check if the message was written by others
     *  메시지가 다른이로부터 오는 걸 방지
     *  데이터가 최근에 업데이트 되었는지를 나타냄
     *  플레그가 설정되어 있으면 데이터가 최신
     */
    int dirtyBit;
    /** stored message 안에 들은 메시지 */
    struct message msg;
};

/** message boxes from central control node to each robot 
 *  중앙 관제 노드가 각 로봇에게 할당하는 메시지 박스
*/
extern struct message_box* boxes_from_central_control_node;
/** message boxes from robots to central control node 
 * 로봇이 중앙 관제 노드로 보내는 메시지 박스
*/
extern struct message_box* boxes_from_robots;

#endif