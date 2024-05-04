#include "projects/automated_warehouse/robot.h"

/**
 * A function setting up robot structure
 * 창고 내에서 로봇 작동을 위한 주요 논리 포함하는 
 */
void setRobot(struct robot* _robot, const char* name, int row, int col, int required_payload, int current_payload){
    _robot->name = name;
    _robot->row = row;
    _robot->col = col;
    _robot->required_payload = required_payload; // 적재 해야할 화물
    _robot->current_payload = current_payload;  // 현재 위치하고 있는 화물 위치, required랑 다르면 이동을 해야함
}