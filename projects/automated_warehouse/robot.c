#include "projects/automated_warehouse/robot.h"

/**
 * A function setting up robot structure
 * 창고 내에서 로봇 작동을 위한 주요 논리 포함하는 
 */
void setRobot(struct robot* _robot, const char* name, int row, int col, int required_payload, int current_payload){
    _robot->name = name;
    _robot->row = row;
    _robot->col = col;
    _robot->required_payload = required_payload; // 하역해야할 장소
    _robot->current_payload = current_payload;  // 현재 갖고 있는 화물?
}