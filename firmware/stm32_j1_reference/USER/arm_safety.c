#include "arm_safety.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "safe_console.h"
#include "servo_bus.h"
#include "z_gpio.h"
#include "z_timer.h"

#define ARM_JOINT_COUNT 5
#define ARM_PLAN_SIZE 5
#define ARM_TEST_STEP_COUNT 4

#define BOOT_SETTLE_MS 250
#define POSITION_QUERY_TIMEOUT_MS 150
#define POSITION_QUERY_RETRIES 2
#define POSITION_POLL_INTERVAL_MS 80
#define STEP_MOVE_TIME_MS 1500
#define STEP_MOVE_TIMEOUT_MS 3000
#define STEP_BEEP_MS 50
#define TORQUE_RESTORE_SETTLE_MS 150
#define MANUAL_BEEP_ON_MS 100
#define MANUAL_BEEP_OFF_MS 100
#define MANUAL_BEEP_MAX_COUNT 5

typedef struct {
	const char *name;
	u16 minimum;
	u16 maximum;
	u16 test_delta;
	u16 readback_tolerance;
	u16 home;
	u8 include_in_home;
} joint_config_t;

typedef enum {
	STATE_BOOT_WAIT = 0,
	STATE_BOOT_QUERY_SEND,
	STATE_BOOT_QUERY_WAIT,
	STATE_IDLE,
	STATE_TEST_TORQUE_RESTORE,
	STATE_TEST_TORQUE_WAIT,
	STATE_TEST_START_SEND,
	STATE_TEST_START_WAIT,
	STATE_DIAG_SEND,
	STATE_DIAG_WAIT,
	STATE_READ_POSITION_SEND,
	STATE_READ_POSITION_WAIT,
	STATE_HOME_PRECHECK_SEND,
	STATE_HOME_PRECHECK_WAIT,
	STATE_RECOVERY_TORQUE_RESTORE,
	STATE_RECOVERY_TORQUE_WAIT,
	STATE_STEP_MOVE_SEND,
	STATE_STEP_POLL_DELAY,
	STATE_STEP_POLL_WAIT,
	STATE_STEP_BEEP_WAIT,
	STATE_FAULT
} safety_state_t;

typedef enum {
	ACTION_NONE = 0,
	ACTION_TEST,
	ACTION_HOME,
	ACTION_RECOVERY
} action_kind_t;

/*
 * Stage-1 commissioning limits are intentionally narrow. They are not the
 * final mechanical limits. Expand one joint at a time only after real tests.
 */
static const joint_config_t joint_config[ARM_JOINT_COUNT] = {
	{"BASE",       1480, 1550, 20, 8, 1514, 1},
	{"SHOULDER",   1500, 1570, 20, 8, 1539, 1},
	{"ELBOW",      1510, 1570,  8, 3, 1547, 1},
	{"UPPER_ARM",  1510, 1580, 20, 8, 1548, 1},
	{"TOOL_ROLL",  1470, 1540, 20, 8, 1505, 1}
};

/* ID 005 is deliberately read-only during stage-1 commissioning. */
/* Required commissioning order: 003, 002, 001, 004, 000. */
static const u8 test_order[ARM_JOINT_COUNT] = {3, 2, 1, 4, 0};
static const u8 home_order[ARM_PLAN_SIZE] = {3, 2, 1, 0, 4};

static safety_state_t state;
static action_kind_t action_kind;
static u32 state_started_ms;
static u32 query_started_ms;
static u32 step_started_ms;
static u32 last_poll_ms;
static u32 beep_started_ms;
static u8 query_retries;
static u8 boot_joint;
static u8 boot_ready;
static u8 boot_limit_violation_mask;
static u8 fault_latched;
static u8 verified_mask;
static u8 test_joint;
static u8 diagnostic_joint;
static u8 position_read_joint;
static u8 plan_joint[ARM_PLAN_SIZE];
static u16 plan_target[ARM_PLAN_SIZE];
static u8 plan_count;
static u8 plan_index;
static u16 joint_position[ARM_JOINT_COUNT];
static u8 joint_position_valid[ARM_JOINT_COUNT];
static u8 manual_beep_remaining;
static u8 manual_beep_on;
static u32 manual_beep_changed_ms;

static u8 elapsed(u32 now, u32 start, u32 duration) {
	return ((u32)(now - start) >= duration);
}

static int position_error(u16 current, u16 target) {
	int difference;
	difference = (int)current - (int)target;
	return difference < 0 ? -difference : difference;
}

static const char *state_name(void) {
	switch(state) {
		case STATE_BOOT_WAIT: return "BOOT_WAIT";
		case STATE_BOOT_QUERY_SEND: return "BOOT_QUERY_SEND";
		case STATE_BOOT_QUERY_WAIT: return "BOOT_QUERY_WAIT";
		case STATE_IDLE: return "IDLE";
		case STATE_TEST_TORQUE_RESTORE: return "TEST_TORQUE_RESTORE";
		case STATE_TEST_TORQUE_WAIT: return "TEST_TORQUE_WAIT";
		case STATE_TEST_START_SEND: return "TEST_START_SEND";
		case STATE_TEST_START_WAIT: return "TEST_START_WAIT";
		case STATE_DIAG_SEND: return "DIAG_SEND";
		case STATE_DIAG_WAIT: return "DIAG_WAIT";
		case STATE_READ_POSITION_SEND: return "READ_POSITION_SEND";
		case STATE_READ_POSITION_WAIT: return "READ_POSITION_WAIT";
		case STATE_HOME_PRECHECK_SEND: return "HOME_PRECHECK_SEND";
		case STATE_HOME_PRECHECK_WAIT: return "HOME_PRECHECK_WAIT";
		case STATE_STEP_MOVE_SEND: return "STEP_MOVE_SEND";
		case STATE_STEP_POLL_DELAY: return "STEP_POLL_DELAY";
		case STATE_STEP_POLL_WAIT: return "STEP_POLL_WAIT";
		case STATE_STEP_BEEP_WAIT: return "STEP_BEEP_WAIT";
		case STATE_FAULT: return "FAULT";
		default: return "UNKNOWN";
	}
}

static void print_line(const char *text) {
	safe_console_write(text);
}

static void print_position(const char *prefix, u8 id, u16 position) {
	char line[96];
	sprintf(line, "%s id=%03u position=%u\r\n", prefix,
		(unsigned int)id, (unsigned int)position);
	print_line(line);
}

static u8 position_within_limit(u8 id, u16 position) {
	if(id >= ARM_JOINT_COUNT) {
		return 0;
	}
	return position >= joint_config[id].minimum && position <= joint_config[id].maximum;
}

static void reject_joint_limit(u8 id, int target) {
	char line[128];
	sprintf(line, "ERR JOINT_LIMIT id=%03u target=%d min=%u max=%u\r\n",
		(unsigned int)id, target,
		(unsigned int)joint_config[id].minimum,
		(unsigned int)joint_config[id].maximum);
	print_line(line);
	action_kind = ACTION_NONE;
	state = STATE_IDLE;
}

static void enter_fault(const char *code, u8 id) {
	char line[96];

	servo_bus_stop_all();
	beep_off();
	action_kind = ACTION_NONE;
	fault_latched = 1;
	state = STATE_FAULT;
	sprintf(line, "ERR %s id=%03u\r\n", code, (unsigned int)id);
	print_line(line);
}

static void reset_scan(u32 now) {
	u8 i;
	for(i = 0; i < ARM_JOINT_COUNT; i++) {
		joint_position[i] = 0;
		joint_position_valid[i] = 0;
	}
	boot_joint = 0;
	query_retries = 0;
	boot_ready = 0;
	boot_limit_violation_mask = 0;
	fault_latched = 0;
	verified_mask = 0;
	action_kind = ACTION_NONE;
	state_started_ms = now;
	state = STATE_BOOT_WAIT;
	print_line("INFO RESCAN_STARTED\r\n");
}

static u8 test_order_allowed(u8 id, u8 *required_id) {
	u8 i;
	u8 earlier;

	if((verified_mask & (1U << id)) != 0) {
		return 1;
	}

	for(i = 0; i < ARM_JOINT_COUNT; i++) {
		if(test_order[i] == id) {
			for(earlier = 0; earlier < i; earlier++) {
				if((verified_mask & (1U << test_order[earlier])) == 0) {
					*required_id = test_order[earlier];
					return 0;
				}
			}
			return 1;
		}
	}
	return 0;
}

static u8 home_is_verified(void) {
	u8 i;
	for(i = 0; i < ARM_PLAN_SIZE; i++) {
		if((verified_mask & (1U << home_order[i])) == 0) {
			return 0;
		}
	}
	return 1;
}

static void begin_test(u8 id) {
	u8 required_id;
	char line[96];

	if(id >= ARM_JOINT_COUNT) {
		print_line("ERR INVALID_JOINT\r\n");
		return;
	}
	if(!test_order_allowed(id, &required_id)) {
		sprintf(line, "ERR TEST_ORDER required=%03u\r\n", (unsigned int)required_id);
		print_line(line);
		return;
	}

	test_joint = id;
	action_kind = ACTION_TEST;
	plan_count = 0;
	plan_index = 0;
	query_retries = 0;
	state = STATE_TEST_TORQUE_RESTORE;
	sprintf(line, "OK TEST_ACCEPTED id=%03u name=%s\r\n",
		(unsigned int)id, joint_config[id].name);
	print_line(line);
}

static void begin_home(void) {
	u8 i;
	char line[80];

	if(!home_is_verified()) {
		sprintf(line, "ERR HOME_NOT_VERIFIED mask=0x%02X\r\n", (unsigned int)verified_mask);
		print_line(line);
		return;
	}

	for(i = 0; i < ARM_PLAN_SIZE; i++) {
		plan_joint[i] = home_order[i];
		plan_target[i] = joint_config[home_order[i]].home;
		if(!position_within_limit(plan_joint[i], plan_target[i])) {
			reject_joint_limit(plan_joint[i], plan_target[i]);
			return;
		}
	}

	plan_count = ARM_PLAN_SIZE;
	plan_index = 0;
	action_kind = ACTION_HOME;
	query_retries = 0;
	state = STATE_HOME_PRECHECK_SEND;
	print_line("OK HOME_ACCEPTED sequence=003,002,001,000,004\r\n");
}

static void begin_recovery(u8 id, u16 target) {
	char line[96];
	u16 current;
	u16 distance;
	u16 maximum_delta;

	if(id >= ARM_JOINT_COUNT || !joint_position_valid[id]) {
		print_line("ERR RECOVERY_POSITION_UNKNOWN\r\n");
		return;
	}
	if(!position_within_limit(id, target)) {
		reject_joint_limit(id, target);
		return;
	}
	current = joint_position[id];
	distance = current > target ? (u16)(current - target) : (u16)(target - current);
	maximum_delta = (u16)(joint_config[id].test_delta + joint_config[id].readback_tolerance);
	if(distance > maximum_delta) {
		sprintf(line, "ERR RECOVERY_DELTA id=%03u current=%u target=%u max=%u\r\n",
			(unsigned int)id, (unsigned int)current, (unsigned int)target,
			(unsigned int)maximum_delta);
		print_line(line);
		return;
	}

	plan_joint[0] = id;
	plan_target[0] = target;
	plan_count = 1;
	plan_index = 0;
	action_kind = ACTION_RECOVERY;
	query_retries = 0;
	state = STATE_RECOVERY_TORQUE_RESTORE;
	sprintf(line, "OK RECOVERY_ACCEPTED id=%03u current=%u target=%u\r\n",
		(unsigned int)id, (unsigned int)current, (unsigned int)target);
	print_line(line);
}

static void prepare_test_plan(u16 start_position) {
	int targets[ARM_TEST_STEP_COUNT];
	u8 i;
	int delta;

	delta = (int)joint_config[test_joint].test_delta;
	targets[0] = (int)start_position + delta;
	targets[1] = (int)start_position;
	targets[2] = (int)start_position - delta;
	targets[3] = (int)start_position;

	if(!position_within_limit(test_joint, start_position)) {
		reject_joint_limit(test_joint, start_position);
		return;
	}

	for(i = 0; i < ARM_TEST_STEP_COUNT; i++) {
		if(targets[i] < joint_config[test_joint].minimum ||
			targets[i] > joint_config[test_joint].maximum) {
			reject_joint_limit(test_joint, targets[i]);
			return;
		}

		plan_joint[i] = test_joint;
		plan_target[i] = (u16)targets[i];
	}

	plan_count = ARM_TEST_STEP_COUNT;
	plan_index = 0;
	state = STATE_STEP_MOVE_SEND;
}

static const char *test_step_name(u8 index) {
	switch(index) {
		case 0: return "POS";
		case 1: return "RETURN_1";
		case 2: return "NEG";
		case 3: return "RETURN_2";
		default: return "UNKNOWN";
	}
}

static void step_complete(u32 now, u8 id, u16 current) {
	char line[160];
	const char *step;

	joint_position[id] = current;
	joint_position_valid[id] = 1;
	step = action_kind == ACTION_TEST ? test_step_name(plan_index) :
		(action_kind == ACTION_RECOVERY ? "RECOVERY" : "HOME");
	sprintf(line,
		"STEP action=%s step=%s id=%03u target=%u current=%u elapsed=%lu result=OK\r\n",
		action_kind == ACTION_TEST ? "TEST" :
			(action_kind == ACTION_RECOVERY ? "RECOVERY" : "HOME"), step,
		(unsigned int)id, (unsigned int)plan_target[plan_index],
		(unsigned int)current, (unsigned long)(now - step_started_ms));
	print_line(line);

	beep_on();
	beep_started_ms = now;
	state = STATE_STEP_BEEP_WAIT;
}

static void action_complete(void) {
	char line[80];
	if(action_kind == ACTION_TEST) {
		verified_mask |= (u8)(1U << test_joint);
		sprintf(line, "OK TEST_DONE id=%03u verified_mask=0x%02X\r\n",
			(unsigned int)test_joint, (unsigned int)verified_mask);
		print_line(line);
	} else if(action_kind == ACTION_HOME) {
		print_line("OK HOME_DONE\r\n");
	} else if(action_kind == ACTION_RECOVERY) {
		print_line("OK RECOVERY_DONE\r\n");
	}
	action_kind = ACTION_NONE;
	state = STATE_IDLE;
}

static void print_status(void) {
	char line[128];
	u8 i;
	sprintf(line, "OK STATUS state=%s ready=%u fault=%u limit_mask=0x%02X verified_mask=0x%02X\r\n",
		state_name(), (unsigned int)boot_ready, (unsigned int)fault_latched,
		(unsigned int)boot_limit_violation_mask,
		(unsigned int)verified_mask);
	print_line(line);
	for(i = 0; i < ARM_JOINT_COUNT; i++) {
		if(joint_position_valid[i]) {
			sprintf(line, "JOINT id=%03u name=%s current=%u min=%u max=%u\r\n",
				(unsigned int)i, joint_config[i].name,
				(unsigned int)joint_position[i],
				(unsigned int)joint_config[i].minimum,
				(unsigned int)joint_config[i].maximum);
		} else {
			sprintf(line, "JOINT id=%03u name=%s current=UNKNOWN min=%u max=%u\r\n",
				(unsigned int)i, joint_config[i].name,
				(unsigned int)joint_config[i].minimum,
				(unsigned int)joint_config[i].maximum);
		}
		print_line(line);
	}
	print_line("JOINT id=005 name=END_EFFECTOR current=UNKNOWN movement=READ_ONLY home=EXCLUDED\r\n");
}

static void begin_manual_beep(u8 count, u32 now) {
	char line[64];

	manual_beep_remaining = count;
	manual_beep_on = 1;
	manual_beep_changed_ms = now;
	beep_on();
	sprintf(line, "OK BEEP_ACCEPTED count=%u\r\n", (unsigned int)count);
	print_line(line);
}

static void manual_beep_process(u32 now) {
	if(manual_beep_remaining == 0) {
		return;
	}

	if(manual_beep_on) {
		if(!elapsed(now, manual_beep_changed_ms, MANUAL_BEEP_ON_MS)) {
			return;
		}
		beep_off();
		manual_beep_on = 0;
		manual_beep_changed_ms = now;
		manual_beep_remaining--;
		if(manual_beep_remaining == 0) {
			print_line("OK BEEP_DONE\r\n");
		}
	} else if(elapsed(now, manual_beep_changed_ms, MANUAL_BEEP_OFF_MS)) {
		beep_on();
		manual_beep_on = 1;
		manual_beep_changed_ms = now;
	}
}

void arm_safety_init(void) {
	u8 i;
	for(i = 0; i < ARM_JOINT_COUNT; i++) {
		joint_position[i] = 0;
		joint_position_valid[i] = 0;
	}
	state = STATE_BOOT_WAIT;
	action_kind = ACTION_NONE;
	state_started_ms = millis();
	query_started_ms = 0;
	step_started_ms = 0;
	last_poll_ms = 0;
	beep_started_ms = 0;
	query_retries = 0;
	boot_joint = 0;
	boot_ready = 0;
	boot_limit_violation_mask = 0;
	fault_latched = 0;
	verified_mask = 0;
	plan_count = 0;
	plan_index = 0;
	manual_beep_remaining = 0;
	manual_beep_on = 0;
	manual_beep_changed_ms = 0;
	beep_off();
	print_line("BOOT JIBOT1_STAGE1_SAFE no_startup_motion=1\r\n");
}

void arm_safety_emergency_stop(void) {
	u8 requires_rescan;
	requires_rescan = (action_kind != ACTION_NONE || !boot_ready || state != STATE_IDLE);
	servo_bus_stop_all();
	beep_off();
	manual_beep_remaining = 0;
	manual_beep_on = 0;
	action_kind = ACTION_NONE;
	plan_count = 0;
	plan_index = 0;
	if(requires_rescan) {
		fault_latched = 1;
		state = STATE_FAULT;
		print_line("OK STOPPED fault_latched=1 rescan_required=1\r\n");
	} else {
		state = boot_ready && !fault_latched ? STATE_IDLE : STATE_FAULT;
		print_line("OK STOPPED\r\n");
	}
}

void arm_safety_handle_command(const char *command) {
	char normalized[SAFE_CONSOLE_LINE_SIZE];
	u16 i;
	u16 start;
	u16 end;
	unsigned int id;
	unsigned int beep_count;
	unsigned int recover_target;
	char trailing;
	u32 now;

	if(command == 0) {
		return;
	}

	start = 0;
	while(command[start] == ' ' || command[start] == '\t') {
		start++;
	}
	end = (u16)strlen(command);
	while(end > start && (command[end - 1] == ' ' || command[end - 1] == '\t' ||
		command[end - 1] == '\r' || command[end - 1] == '\n')) {
		end--;
	}
	if((end - start) >= SAFE_CONSOLE_LINE_SIZE) {
		print_line("ERR COMMAND_TOO_LONG\r\n");
		return;
	}
	for(i = 0; i < (end - start); i++) {
		normalized[i] = (char)tolower((unsigned char)command[start + i]);
	}
	normalized[end - start] = '\0';

	if(strcmp(normalized, "stop") == 0 || strcmp(normalized, "$dst!") == 0) {
		arm_safety_emergency_stop();
		return;
	}
	if(strcmp(normalized, "status") == 0) {
		print_status();
		return;
	}
	if(strcmp(normalized, "rescan") == 0) {
		if(action_kind != ACTION_NONE) {
			print_line("ERR ARM_BUSY\r\n");
			return;
		}
		now = millis();
		reset_scan(now);
		return;
	}
	if(sscanf(normalized, "beep,%u%c", &beep_count, &trailing) == 1) {
		if(beep_count == 0 || beep_count > MANUAL_BEEP_MAX_COUNT) {
			print_line("ERR INVALID_BEEP_COUNT range=1..5\r\n");
			return;
		}
		if(action_kind != ACTION_NONE || manual_beep_remaining != 0) {
			print_line("ERR ARM_BUSY\r\n");
			return;
		}
		begin_manual_beep((u8)beep_count, millis());
		return;
	}
	if(sscanf(normalized, "diag_joint,%u%c", &id, &trailing) == 1) {
		if(id > SERVO_BUS_MAX_ID) {
			print_line("ERR INVALID_JOINT\r\n");
			return;
		}
		if(action_kind != ACTION_NONE || (state != STATE_IDLE && state != STATE_FAULT)) {
			print_line("ERR ARM_BUSY\r\n");
			return;
		}
		diagnostic_joint = (u8)id;
		query_retries = 0;
		state = STATE_DIAG_SEND;
		print_line("OK DIAG_ACCEPTED read_only=1\r\n");
		return;
	}
	if(sscanf(normalized, "read_joint,%u%c", &id, &trailing) == 1) {
		if(id > SERVO_BUS_MAX_ID) {
			print_line("ERR INVALID_JOINT\r\n");
			return;
		}
		if(action_kind != ACTION_NONE || (state != STATE_IDLE && state != STATE_FAULT)) {
			print_line("ERR ARM_BUSY\r\n");
			return;
		}
		position_read_joint = (u8)id;
		query_retries = 0;
		state = STATE_READ_POSITION_SEND;
		print_line("OK READ_ACCEPTED read_only=1\r\n");
		return;
	}
	if(strcmp(normalized, "observe") == 0 ||
		strcmp(normalized, "pick_ready") == 0 ||
		strcmp(normalized, "park") == 0) {
		print_line("ERR POSE_NOT_IMPLEMENTED\r\n");
		return;
	}

	if(!boot_ready || fault_latched || state == STATE_FAULT) {
		print_line("ERR ARM_NOT_READY use=rescan\r\n");
		return;
	}
	if(action_kind != ACTION_NONE || state != STATE_IDLE) {
		print_line("ERR ARM_BUSY\r\n");
		return;
	}

	if(sscanf(normalized, "test_joint,%u%c", &id, &trailing) == 1) {
		if(id >= ARM_JOINT_COUNT) {
			if(id == 5) {
				print_line("ERR READ_ONLY_SERVO id=005\r\n");
			} else {
				print_line("ERR INVALID_JOINT\r\n");
			}
			return;
		}
		begin_test((u8)id);
		return;
	}
	if(sscanf(normalized, "recover_joint,%u,%u%c", &id, &recover_target, &trailing) == 2) {
		if(id == 5) {
			print_line("ERR READ_ONLY_SERVO id=005\r\n");
			return;
		}
		if(id >= ARM_JOINT_COUNT || recover_target > 65535U) {
			print_line("ERR INVALID_RECOVERY\r\n");
			return;
		}
		begin_recovery((u8)id, (u16)recover_target);
		return;
	}
	if(sscanf(normalized, "safe_move,%u,%u%c", &id, &recover_target, &trailing) == 2) {
		if(id == 5) {
			print_line("ERR READ_ONLY_SERVO id=005\r\n");
			return;
		}
		if(id >= ARM_JOINT_COUNT || recover_target > 65535U) {
			print_line("ERR INVALID_SAFE_MOVE\r\n");
			return;
		}
		begin_recovery((u8)id, (u16)recover_target);
		return;
	}
	if(strcmp(normalized, "home") == 0) {
		begin_home();
		return;
	}

	print_line("ERR UNKNOWN_COMMAND\r\n");
}

void arm_safety_process(void) {
	u32 now;
	u8 response_id;
	u16 response_position;
	u8 id;
	u16 target;
	char line[128];
	char response[32];

	now = millis();
	manual_beep_process(now);

	switch(state) {
		case STATE_BOOT_WAIT:
			if(elapsed(now, state_started_ms, BOOT_SETTLE_MS)) {
				state = STATE_BOOT_QUERY_SEND;
			}
			break;

		case STATE_BOOT_QUERY_SEND:
			servo_bus_request_position(boot_joint);
			query_started_ms = now;
			state = STATE_BOOT_QUERY_WAIT;
			break;

		case STATE_BOOT_QUERY_WAIT:
			if(servo_bus_take_position(&response_id, &response_position)) {
				if(response_id == boot_joint) {
					joint_position[boot_joint] = response_position;
					joint_position_valid[boot_joint] = 1;
					print_position("BOOT_POSITION", boot_joint, response_position);
					if(!position_within_limit(boot_joint, response_position)) {
						if(boot_limit_violation_mask == 0) {
							servo_bus_stop_all();
						}
						boot_limit_violation_mask |= (u8)(1U << boot_joint);
						sprintf(line,
							"WARN JOINT_LIMIT id=%03u current=%u min=%u max=%u scan_continues=1\r\n",
							(unsigned int)boot_joint, (unsigned int)response_position,
							(unsigned int)joint_config[boot_joint].minimum,
							(unsigned int)joint_config[boot_joint].maximum);
						print_line(line);
					}
					boot_joint++;
					query_retries = 0;
					if(boot_joint >= ARM_JOINT_COUNT) {
						if(boot_limit_violation_mask != 0) {
							fault_latched = 1;
							state = STATE_FAULT;
							sprintf(line, "ERR BOOT_JOINT_LIMIT mask=0x%02X movement_locked=1\r\n",
								(unsigned int)boot_limit_violation_mask);
							print_line(line);
						} else {
							boot_ready = 1;
							state = STATE_IDLE;
							print_line("OK READY test_order=003,002,001,004,000 suction=005\r\n");
						}
					} else {
						state = STATE_BOOT_QUERY_SEND;
					}
				}
			} else if(elapsed(now, query_started_ms, POSITION_QUERY_TIMEOUT_MS)) {
				if(query_retries < POSITION_QUERY_RETRIES) {
					query_retries++;
					state = STATE_BOOT_QUERY_SEND;
				} else {
					enter_fault("POSITION_READ", boot_joint);
				}
			}
			break;

		case STATE_TEST_START_SEND:
			servo_bus_request_position(test_joint);
			query_started_ms = now;
			state = STATE_TEST_START_WAIT;
			break;

		case STATE_TEST_TORQUE_RESTORE:
			servo_bus_restore_torque(test_joint);
			state_started_ms = now;
			sprintf(line, "HOLD_CURRENT id=%03u settle=%u\r\n",
				(unsigned int)test_joint, (unsigned int)TORQUE_RESTORE_SETTLE_MS);
			print_line(line);
			state = STATE_TEST_TORQUE_WAIT;
			break;

		case STATE_TEST_TORQUE_WAIT:
			if(elapsed(now, state_started_ms, TORQUE_RESTORE_SETTLE_MS)) {
				query_retries = 0;
				state = STATE_TEST_START_SEND;
			}
			break;

		case STATE_TEST_START_WAIT:
			if(servo_bus_take_position(&response_id, &response_position)) {
				if(response_id == test_joint) {
					joint_position[test_joint] = response_position;
					joint_position_valid[test_joint] = 1;
					print_position("TEST_START", test_joint, response_position);
					prepare_test_plan(response_position);
				}
			} else if(elapsed(now, query_started_ms, POSITION_QUERY_TIMEOUT_MS)) {
				if(query_retries < POSITION_QUERY_RETRIES) {
					query_retries++;
					state = STATE_TEST_START_SEND;
				} else {
					enter_fault("POSITION_READ", test_joint);
				}
			}
			break;

		case STATE_DIAG_SEND:
			servo_bus_request_telemetry(diagnostic_joint);
			query_started_ms = now;
			state = STATE_DIAG_WAIT;
			break;

		case STATE_DIAG_WAIT:
			if(servo_bus_take_response(response, sizeof(response))) {
				sprintf(line, "OK DIAG id=%03u elapsed=%lu response=%s\r\n",
					(unsigned int)diagnostic_joint,
					(unsigned long)(now - query_started_ms), response);
				print_line(line);
				state = fault_latched ? STATE_FAULT : (boot_ready ? STATE_IDLE : STATE_FAULT);
			} else if(elapsed(now, query_started_ms, POSITION_QUERY_TIMEOUT_MS)) {
				if(query_retries < POSITION_QUERY_RETRIES) {
					query_retries++;
					state = STATE_DIAG_SEND;
				} else {
					sprintf(line, "ERR DIAG_READ id=%03u\r\n", (unsigned int)diagnostic_joint);
					print_line(line);
					state = fault_latched ? STATE_FAULT : (boot_ready ? STATE_IDLE : STATE_FAULT);
				}
			}
			break;

		case STATE_READ_POSITION_SEND:
			servo_bus_request_position(position_read_joint);
			query_started_ms = now;
			state = STATE_READ_POSITION_WAIT;
			break;

		case STATE_READ_POSITION_WAIT:
			if(servo_bus_take_position(&response_id, &response_position)) {
				if(response_id == position_read_joint) {
					if(position_read_joint < ARM_JOINT_COUNT) {
						joint_position[position_read_joint] = response_position;
						joint_position_valid[position_read_joint] = 1;
					}
					sprintf(line, "OK POSITION id=%03u current=%u elapsed=%lu\r\n",
						(unsigned int)position_read_joint, (unsigned int)response_position,
						(unsigned long)(now - query_started_ms));
					print_line(line);
					state = fault_latched ? STATE_FAULT : (boot_ready ? STATE_IDLE : STATE_FAULT);
				}
			} else if(elapsed(now, query_started_ms, POSITION_QUERY_TIMEOUT_MS)) {
				if(query_retries < POSITION_QUERY_RETRIES) {
					query_retries++;
					state = STATE_READ_POSITION_SEND;
				} else {
					sprintf(line, "ERR POSITION_READ id=%03u\r\n",
						(unsigned int)position_read_joint);
					print_line(line);
					state = fault_latched ? STATE_FAULT : (boot_ready ? STATE_IDLE : STATE_FAULT);
				}
			}
			break;

		case STATE_HOME_PRECHECK_SEND:
			id = plan_joint[plan_index];
			servo_bus_request_position(id);
			query_started_ms = now;
			state = STATE_HOME_PRECHECK_WAIT;
			break;

		case STATE_HOME_PRECHECK_WAIT:
			id = plan_joint[plan_index];
			if(servo_bus_take_position(&response_id, &response_position)) {
				if(response_id == id) {
					joint_position[id] = response_position;
					joint_position_valid[id] = 1;
					if(!position_within_limit(id, response_position)) {
						reject_joint_limit(id, response_position);
					} else if(action_kind == ACTION_RECOVERY &&
						position_error(response_position, plan_target[plan_index]) >
						(int)(joint_config[id].test_delta + joint_config[id].readback_tolerance)) {
						sprintf(line,
							"ERR RECOVERY_DELTA id=%03u current=%u target=%u max=%u\r\n",
							(unsigned int)id, (unsigned int)response_position,
							(unsigned int)plan_target[plan_index],
							(unsigned int)(joint_config[id].test_delta +
								joint_config[id].readback_tolerance));
						print_line(line);
						action_kind = ACTION_NONE;
						state = STATE_IDLE;
					} else {
						query_retries = 0;
						state = STATE_STEP_MOVE_SEND;
					}
				}
			} else if(elapsed(now, query_started_ms, POSITION_QUERY_TIMEOUT_MS)) {
				if(query_retries < POSITION_QUERY_RETRIES) {
					query_retries++;
					state = STATE_HOME_PRECHECK_SEND;
				} else {
					enter_fault("POSITION_READ", id);
				}
			}
			break;

		case STATE_RECOVERY_TORQUE_RESTORE:
			id = plan_joint[0];
			servo_bus_restore_torque(id);
			state_started_ms = now;
			sprintf(line, "HOLD_CURRENT id=%03u settle=%u recovery=1\r\n",
				(unsigned int)id, (unsigned int)TORQUE_RESTORE_SETTLE_MS);
			print_line(line);
			state = STATE_RECOVERY_TORQUE_WAIT;
			break;

		case STATE_RECOVERY_TORQUE_WAIT:
			if(elapsed(now, state_started_ms, TORQUE_RESTORE_SETTLE_MS)) {
				query_retries = 0;
				state = STATE_HOME_PRECHECK_SEND;
			}
			break;

		case STATE_STEP_MOVE_SEND:
			id = plan_joint[plan_index];
			target = plan_target[plan_index];
			if(!position_within_limit(id, target)) {
				reject_joint_limit(id, target);
				break;
			}
			servo_bus_move(id, target, STEP_MOVE_TIME_MS);
			step_started_ms = now;
			last_poll_ms = now;
			sprintf(line, "MOVE id=%03u target=%u time=%u\r\n",
				(unsigned int)id, (unsigned int)target,
				(unsigned int)STEP_MOVE_TIME_MS);
			print_line(line);
			state = STATE_STEP_POLL_DELAY;
			break;

		case STATE_STEP_POLL_DELAY:
			id = plan_joint[plan_index];
			if(elapsed(now, step_started_ms, STEP_MOVE_TIMEOUT_MS)) {
				sprintf(line, "TIMEOUT_STATUS id=%03u target=%u current=%u\r\n",
					(unsigned int)id, (unsigned int)plan_target[plan_index],
					(unsigned int)joint_position[id]);
				print_line(line);
				enter_fault("MOVE_TIMEOUT", id);
			} else if(elapsed(now, last_poll_ms, POSITION_POLL_INTERVAL_MS)) {
				servo_bus_request_position(id);
				query_started_ms = now;
				state = STATE_STEP_POLL_WAIT;
			}
			break;

		case STATE_STEP_POLL_WAIT:
			id = plan_joint[plan_index];
			target = plan_target[plan_index];
			if(elapsed(now, step_started_ms, STEP_MOVE_TIMEOUT_MS)) {
				sprintf(line, "TIMEOUT_STATUS id=%03u target=%u current=%u\r\n",
					(unsigned int)id, (unsigned int)target,
					(unsigned int)joint_position[id]);
				print_line(line);
				enter_fault("MOVE_TIMEOUT", id);
			} else if(servo_bus_take_position(&response_id, &response_position)) {
				if(response_id == id) {
					joint_position[id] = response_position;
					joint_position_valid[id] = 1;
					if(position_error(response_position, target) <=
						(int)joint_config[id].readback_tolerance) {
						step_complete(now, id, response_position);
					} else {
						last_poll_ms = now;
						state = STATE_STEP_POLL_DELAY;
					}
				}
			} else if(elapsed(now, query_started_ms, POSITION_QUERY_TIMEOUT_MS)) {
				last_poll_ms = now;
				state = STATE_STEP_POLL_DELAY;
			}
			break;

		case STATE_STEP_BEEP_WAIT:
			if(elapsed(now, beep_started_ms, STEP_BEEP_MS)) {
				beep_off();
				plan_index++;
				if(plan_index >= plan_count) {
					action_complete();
				} else if(action_kind == ACTION_HOME) {
					query_retries = 0;
					state = STATE_HOME_PRECHECK_SEND;
				} else {
					state = STATE_STEP_MOVE_SEND;
				}
			}
			break;

		case STATE_IDLE:
		case STATE_FAULT:
		default:
			break;
	}
}
