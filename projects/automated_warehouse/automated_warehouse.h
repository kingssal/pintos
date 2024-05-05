#ifndef __PROJECTS_AUTOMATED_WAREHOUSE_H__
#define __PROJECTS_AUTOMATED_WAREHOUSE_H__

#include "projects/automated_warehouse/aw_manager.h"
#include "aw_message.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


void run_automated_warehouse(char **argv);
void test_thread(void* aux);
void test_cnt();
void do_robot_task(int idx);
void process_robots_data(void);
void determine_robot_cmd(int robot_id, struct message robot_msg);
void find_target_position(int target, int *row, int *col);
void send_cmd_to_robot(int robot_id, int target_row, int target_col);
int calculate_direction(int current_row, int current_col, int target_row, int target_col);
bool can_move(int new_row, int new_col);
void update_robot_status(int idx);


#endif 
