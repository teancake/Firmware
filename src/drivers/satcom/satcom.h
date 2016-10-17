#pragma once

#include <stdlib.h>
#include <stdbool.h>

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>

typedef enum {
	SATCOM_OK = 0,
	SATCOM_NO_MSG = -1,
	SATCOM_ERROR = -255,
} satcom_status;

typedef enum {
	SATCOM_UART_OK = 0,
	SATCOM_UART_OPEN_FAIL = -1,
} satcom_uart_status;

typedef enum {
	SATCOM_READ_OK = 0,
	SATCOM_READ_TIMEOUT = -1,
	SATCOM_READ_PARSING_FAIL = -2,
} satcom_read_status;

typedef enum {
	SATCOM_RESULT_OK,
	SATCOM_RESULT_ERROR,
	SATCOM_RESULT_SBDRING,
	SATCOM_RESULT_READY,
	SATCOM_RESULT_HWFAIL,
	SATCOM_RESULT_NA,
} satcom_result_code;

//typedef struct
//{
//	uint8_t	info;
//	uint8_t	result_code;
//} satcom_at_msg;

typedef enum {
	SATCOM_STATE_STANDBY,
	SATCOM_STATE_CSQ,
	SATCOM_STATE_SBDSESSION,
	SATCOM_STATE_TEST,
} satcom_state;

const char *satcom_state_string[4] = {"STANDBY", "SIGNAL CHECK", "SBD SESSION", "TEST"};

extern "C" __EXPORT int satcom_main(int argc, char *argv[]);

#define SATCOM_TX_BUF_LEN			340		// TX buffer size - maximum for a SBD MO message
#define SATCOM_RX_MSG_BUF_LEN			300		// RX buffer size for MT messages
#define SATCOM_RX_COMMAND_BUF_LEN		50		// RX buffer size for other commands
#define SATCOM_TX_STACKING_TIME			3000000	// time to wait for additional mavlink messages, TODO make this a param
#define SATCOM_SIGNAL_REFRESH_DELAY		5000000 // update signal quality every 5s

class satcom : public device::CDev
{
public:
	static satcom *instance;
	static int task_handle;
	bool task_should_exit = false;
	int uart_fd = -1;

	int param_read_interval_s;

	hrt_abstime last_signal_check = 0;
	uint8_t signal_quality = 0;

	bool test_pending = false;
	char test_command[32];
	hrt_abstime test_timer = 0;

	uint8_t rx_command_buf[SATCOM_RX_COMMAND_BUF_LEN] = {0};
	int rx_command_len = 0;

	uint8_t rx_msg_buf[SATCOM_RX_MSG_BUF_LEN] = {0};
	int rx_msg_end_idx = 0;
	int rx_msg_read_idx = 0;

	uint8_t tx_buf[SATCOM_TX_BUF_LEN] = {0};
	int tx_buf_write_idx = 0;

	bool ring_pending = false;
	bool rx_session_pending = false;
	bool rx_read_pending = false;
	bool tx_session_pending = false;

	hrt_abstime last_write_time = 0;
	hrt_abstime last_read_time = 0;

	satcom_state state = SATCOM_STATE_STANDBY;
	satcom_state new_state = SATCOM_STATE_STANDBY;

	pthread_mutex_t tx_buf_mutex = pthread_mutex_t();
	bool verbose = false;

	/*
	 * Constructor
	 */
	satcom();

	/*
	 * Start the driver
	 */
	static int start(int argc, char *argv[]);

	/*
	 * Stop the driver
	 */
	static int stop();

	/*
	 * Display driver status
	 */
	static void status();

	/*
	 * Run a basic driver test
	 */
	static void test(int argc, char *argv[]);

	/*
	 * Entry point of the task, has to be a static function
	 */
	static void main_loop_helper(int argc, char *argv[]);

	/*
	 * Main driver loop
	 */
	void main_loop(int argc, char *argv[]);

	/*
	 * Use to send mavlink messages directly
	 */
	ssize_t write(struct file *filp, const char *buffer, size_t buflen);

	/*
	 * Use to read received mavlink messages directly
	 */
	ssize_t read(struct file *filp, char *buffer, size_t buflen);

	/*
	 * Passes everything to CDev
	 */
	int ioctl(struct file *filp, int cmd, unsigned long arg);

	/*
	 * Get the poll state
	 */
	pollevent_t poll_state(struct file *filp);

	/*
	 * Open and configure the given UART port
	 */
	satcom_uart_status open_uart(char *uart_name);

	/*
	 *
	 */
	void write_tx_buf();

	/*
	 *
	 */
	void read_rx_buf();

	/*
	 *
	 */
	bool clear_mo_buffer();

	/*
	 * Perform a SBD session, sending the message from the MO buffer (if previously written)
	 * and retreiving a MT message from the Iridium system (if there is one waiting)
	 * This will also update the registration needed for SBD RING
	 */
	int sbd_session(void);

	/*
	 * Get the network signal strength
	 */
	void start_csq(void);

	/*
	 *
	 */
	satcom_result_code read_at_command();

	/*
	 *
	 */
	satcom_result_code read_at_msg();

	/*
	 *
	 */
	satcom_result_code read_at(uint8_t *rx_buf, int *rx_len);

	/*
	 *
	 */
	void schedule_test(void);

	/*
	 *
	 */
	void standby_loop(void);

	/*
	 *
	 */
	void csq_loop(void);

	/*
	 *
	 */
	void sbdsession_loop(void);

	/*
	 *
	 */
	void test_loop(void);

	/*
	 * TEST
	 */
	void start_test(void);

	/*
	 *
	 */
	void start_sbd_session(void);

	/*
	 * Checks if the modem responds to the "AT" command
	 */
	bool is_modem_ready(void);

	/*
	 * Send a AT command to the modem
	 */
	void write_at(const char *command);
};
