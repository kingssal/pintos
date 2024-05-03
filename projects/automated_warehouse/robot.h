#ifndef _PROJECTS_PROJECT1_ROBOT_H__
#define _PROJECTS_PROJECT1_ROBOT_H__

/**
 * A Structure representing robot
 * 로봇 기능에 대한 선언
 */
struct robot {
    const char* name;
    int row;
    int col;
    int required_payload;
    int current_payload; 
};

void setRobot(struct robot* _robot, const char* name, int row, int col, int required_payload, int current_payload);

#endif