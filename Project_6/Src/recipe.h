#ifndef __STM32L476G_DISCOVERY_RECIPE_H
#define __STM32L476G_DISCOVERY_RECIPE_H

#include <stdlib.h>
#include <stdbool.h>
#include "stm32l476xx.h"

enum status{
	status_running,
	status_paused,
	status_command_error,
	status_nested_error
};

enum states{
	recipe_running,
	recipe_paused,
	recipe_cmd_err,
	recipe_nested_err,
	recipe_done
};

enum user_events{
	pause_recipe,
	continue_recipe,
	move_right,
	move_left,
	begin_recipe,
	noop
};

struct commands{
	enum user_events event[2];
	bool updated[2];
};

// Define all of the commands that are valid
#define MOV (0x20)
#define WAIT (0x40)
#define LOOP (0x80)
#define END_LOOP (0xA0)
#define RECIPE_END (0)

#endif
