#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <pthread.h>

#include <nuttx/config.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <systemlib/scheduling_priorities.h>
#include <systemlib/param/param.h>

#include "drivers/drv_satcom.h"
#include "satcom.h"

satcom *satcom::instance;
int satcom::task_handle;

satcom::satcom():
	CDev("satcom", SATCOM_DEVICE_PATH)
{}

int satcom::start(int argc, char *argv[])
{
	PX4_INFO("starting");

	if (satcom::instance != nullptr) {
		PX4_WARN("already started");
		return PX4_ERROR;
	}

	satcom::instance = new satcom();

	satcom::task_handle = px4_task_spawn_cmd("satcom", SCHED_DEFAULT,
			      SCHED_PRIORITY_SLOW_DRIVER, 2048, (main_t)&satcom::main_loop_helper, argv);

	return OK;
}

int satcom::stop()
{
	if (satcom::instance == nullptr) {
		PX4_WARN("not started");
		return PX4_ERROR;
	}

	PX4_WARN("stopping...");

	satcom::instance->task_should_exit = true;

//	// give it enough time to stop
//	param_timeout_s = 10;

	// TODO
	for (int i = 0; (i < 10 + 1) && (satcom::task_handle != -1); i++) {
		sleep(1);
	}

	// well, kill it anyway, though this may crash
	if (satcom::task_handle != -1) {
		PX4_WARN("killing task forcefully");

		::close(satcom::instance->uart_fd);
		task_delete(satcom::task_handle);
		satcom::task_handle = -1;
		delete satcom::instance;
		satcom::instance = nullptr;
	}

	return OK;
}

void satcom::status()
{
	if (satcom::instance == nullptr) {
		PX4_WARN("not started");
		return;

	}

	PX4_INFO("started");
	PX4_INFO("state: %s", satcom_state_string[instance->state]);

	PX4_INFO("TX buf written: %d", instance->tx_buf_write_idx);
}

void satcom::test(int argc, char *argv[])
{
	if (instance == nullptr) {
		PX4_WARN("not started");
		return;
	}

	if (instance->state != SATCOM_STATE_STANDBY || instance->test_pending) {
		PX4_WARN("MODEM BUSY!");
		return;
	}

	if (argc > 2) {
		strcpy(instance->test_command, argv[2]);

	} else {
		instance->test_command[0] = 0;
	}

	instance->schedule_test();
}

void satcom::main_loop_helper(int argc, char *argv[])
{
	// start the main loop and stay in it
	satcom::instance->main_loop(argc, argv);

	// tear down everything after the main loop exits
	::close(satcom::instance->uart_fd);
	satcom::task_handle = -1;
	delete satcom::instance;
	satcom::instance = nullptr;

	PX4_WARN("stopped");
}

void satcom::main_loop(int argc, char *argv[])
{
	pthread_mutex_init(&tx_buf_mutex, NULL);

	int arg_i = 3;
	int arg_uart_name = 0;

	while (arg_i < argc) {
		if (!strcmp(argv[arg_i], "-d")) {
			arg_i++;
			arg_uart_name = arg_i;

		} else if (!strcmp(argv[arg_i], "-v")) {
			PX4_WARN("verbose mode ON");
			verbose = true;
		}

		arg_i++;
	}

	if (arg_uart_name == 0) {
		PX4_WARN("no satcom modem UART port provided!");
		task_should_exit = true;
		return;
	}

	if (open_uart(argv[arg_uart_name]) != SATCOM_UART_OK) {
		PX4_WARN("failed to open UART port!");
		task_should_exit = true;
		return;
	}

	param_t param_pointer;

	param_pointer = param_find("SATCOM_READINT");
	param_get(param_pointer, &param_read_interval_s);

	if (param_read_interval_s == -1) {
		param_read_interval_s = 10;
	}

	if (verbose) { PX4_INFO("read interval %d", param_read_interval_s); }

	while (!task_should_exit) {
		switch (state) {
		case SATCOM_STATE_STANDBY:
			standby_loop();
			break;

		case SATCOM_STATE_CSQ:
			csq_loop();
			break;

		case SATCOM_STATE_SBDSESSION:
			sbdsession_loop();
			break;

		case SATCOM_STATE_TEST:
			test_loop();
			break;
		}

		if (new_state != state) {
			if (verbose) { PX4_INFO("SWITCHING STATE FROM %s TO %s", satcom_state_string[state], satcom_state_string[new_state]); }

			state = new_state;

		} else {
			usleep(100000);	// 100ms
		}
	}
}

void satcom::standby_loop(void)
{
	// TODO probably remove this later
	if (test_pending) {
		test_pending = false;

		if (!strcmp(test_command, "s")) {
			write(0, "kreczmer", 8);

		} else if (!strcmp(test_command, "ss")) {
			write(0, "kreczmerkreczmerkreczmerkreczmerkreczmerkreczmerkreczmerkreczmerkreczmerkreczmerkreczmerkreczmer!!!!!!", 100);

		} else if (!strcmp(test_command, "read")) {
			rx_session_pending = true;

		} else {
			test_timer = hrt_absolute_time();
			start_test();
			return;
		}
	}

	// check for incoming SBDRING, handled inside read_at_command()
	read_at_command();

	if (param_read_interval_s != 0 && (hrt_absolute_time() - last_read_time) / 1000000 > param_read_interval_s) {
		rx_session_pending = true;
	}

	// write the MO buffer when the message stacking time expires
	if ((tx_buf_write_idx > 0) && (hrt_absolute_time() - last_write_time > SATCOM_TX_STACKING_TIME)) {
		write_tx_buf();
	}

	// do not start a SBD session if there is still data in the MT buffer, or it will be lost
	if ((tx_session_pending || rx_session_pending) && !rx_read_pending) {
		if (hrt_absolute_time() - last_signal_check < SATCOM_SIGNAL_REFRESH_DELAY && signal_quality > 0) {
			// clear the MO buffer if we only want to read a message
			if (rx_session_pending && !tx_session_pending) {
				if (clear_mo_buffer()) {
					start_sbd_session();
				}

			} else {
				start_sbd_session();
			}

		} else {
			start_csq();
		}
	}

	// only read the MT buffer if the higher layer (mavlink app) read the previous message
	if (rx_read_pending && (rx_msg_read_idx == rx_msg_end_idx)) {
		read_rx_buf();
	}
}

void satcom::csq_loop(void)
{
	int res = read_at_command();

	if (res == SATCOM_RESULT_NA) {
		return;
	}

	if (res != SATCOM_RESULT_OK) {
		if (verbose) { PX4_INFO("UPDATE SIGNAL QUALITY: ERROR"); }

		new_state = SATCOM_STATE_STANDBY;
		return;
	}

	if (strncmp((const char *)rx_command_buf, "+CSQ:", 5)) {
		if (verbose) { PX4_INFO("UPDATE SIGNAL QUALITY: WRONG ANSWER:"); }

		if (verbose) { PX4_INFO("%s", rx_command_buf); }

		new_state = SATCOM_STATE_STANDBY;
		return;
	}

	signal_quality = rx_command_buf[5] - 48;
	//signal_check_pending = false;
	last_signal_check = hrt_absolute_time();

	if (verbose) { PX4_INFO("SIGNAL QUALITY: %d", signal_quality); }

	new_state = SATCOM_STATE_STANDBY;
}

void satcom::sbdsession_loop(void)
{
	int res = read_at_command();

	if (res == SATCOM_RESULT_NA) {
		return;
	}

	if (res != SATCOM_RESULT_OK) {
		if (verbose) { PX4_INFO("SBD SESSION: ERROR"); }

		if (verbose) { PX4_INFO("SBD SESSION: RESULT %d", res); }

		new_state = SATCOM_STATE_STANDBY;
		return;
	}

	if (strncmp((const char *)rx_command_buf, "+SBDIX:", 7)) {
		if (verbose) { PX4_INFO("SBD SESSION: WRONG ANSWER:"); }

		if (verbose) { PX4_INFO("%s", rx_command_buf); }

		new_state = SATCOM_STATE_STANDBY;
		return;
	}

	int mo_status, mt_status, mt_len, mt_queued;
	const char *p = (const char *)rx_command_buf + 7;
	char **rx_buf_parse = (char **)&p;

	mo_status = strtol(*rx_buf_parse, rx_buf_parse, 10);
	(*rx_buf_parse)++;
	strtol(*rx_buf_parse, rx_buf_parse, 10); // MOMSN, ignore it
	(*rx_buf_parse)++;
	mt_status = strtol(*rx_buf_parse, rx_buf_parse, 10);
	(*rx_buf_parse)++;
	strtol(*rx_buf_parse, rx_buf_parse, 10); // MTMSN, ignore it
	(*rx_buf_parse)++;
	mt_len = strtol(*rx_buf_parse, rx_buf_parse, 10);
	(*rx_buf_parse)++;
	mt_queued = strtol(*rx_buf_parse, rx_buf_parse, 10);

	if (verbose) { PX4_INFO("MO ST: %d, MT ST: %d, MT LEN: %d, MT QUEUED: %d", mo_status, mt_status, mt_len, mt_queued); }

	switch (mo_status) {
	case 0:
	case 2:
	case 3:
	case 4:
		if (verbose) { PX4_INFO("SBD SESSION: SUCCESS"); }

		ring_pending = false;
		rx_session_pending = false;
		tx_session_pending = false;
		last_read_time = hrt_absolute_time();

		if (mt_len > 0) {
			rx_read_pending = true;
		}

		break;

	case 1:
		if (verbose) { PX4_INFO("SBD SESSION: MO SUCCESS, MT FAIL"); }

		tx_session_pending = false;
		break;

	case 32:
		if (verbose) { PX4_INFO("SBD SESSION: NO NETWORK SIGNAL"); }

		signal_quality = 0;

		break;

	default:
		if (verbose) { PX4_INFO("SBD SESSION: FAILED (%d)", mo_status); }
	}

	new_state = SATCOM_STATE_STANDBY;
}

void satcom::test_loop(void)
{
	int res = read_at_command();

	if (res != SATCOM_RESULT_NA) {
		PX4_INFO("TEST RESULT: %d, LENGTH %d\nDATA:\n%s", res, rx_command_len, rx_command_buf);
		PX4_INFO("TEST DONE, TOOK %lld MS", (hrt_absolute_time() - test_timer) / 1000);
		new_state = SATCOM_STATE_STANDBY;
	}
}

ssize_t satcom::write(struct file *filp, const char *buffer, size_t buflen)
{
	if (verbose) { PX4_INFO("WRITE: LEN %d, TX WRITTEN: %d", buflen, tx_buf_write_idx); }

	if (buflen > SATCOM_TX_BUF_LEN - tx_buf_write_idx) {
		return 0;
	}

	pthread_mutex_lock(&tx_buf_mutex);

	memcpy(tx_buf, buffer, buflen);

	tx_buf_write_idx += buflen;
	last_write_time = hrt_absolute_time();

	pthread_mutex_unlock(&tx_buf_mutex);

	return buflen;
}

ssize_t satcom::read(struct file *filp, char *buffer, size_t buflen)
{
	if (rx_msg_read_idx < rx_msg_end_idx) {
		size_t bytes_to_copy = rx_msg_end_idx - rx_msg_read_idx;

		if (bytes_to_copy > buflen) {
			bytes_to_copy = buflen;
		}

		memcpy(buffer, &rx_msg_buf[rx_msg_read_idx], bytes_to_copy);

		rx_msg_read_idx += bytes_to_copy;

		return bytes_to_copy;

	} else {
		return -EAGAIN;
	}
}

int satcom::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	return CDev::ioctl(filp, cmd, arg);
}

pollevent_t satcom::poll_state(struct file *filp)
{
	if (rx_msg_read_idx < rx_msg_end_idx) {
		return POLLIN;

	} else {
		return 0;
	}
}

void satcom::write_tx_buf()
{
	if (!is_modem_ready()) {
		if (verbose) { PX4_INFO("SEND SBD: MODEM NOT READY!"); }

		return;
	}

	pthread_mutex_lock(&tx_buf_mutex);

	char command[13];
	sprintf(command, "AT+SBDWB=%d", tx_buf_write_idx);
	write_at(command);

	if (read_at_command() != SATCOM_RESULT_READY) {
		if (verbose) { PX4_INFO("SEND SBD: MODEM NOT RESPONDING!"); }

		return;
	}

	int sum = 0;

	int written = 0;

	while (written != tx_buf_write_idx) {
		written += ::write(uart_fd, tx_buf + written, tx_buf_write_idx - written);
	}

	for (int i = 0; i < tx_buf_write_idx; i++) {
		sum += tx_buf[i];
	}

	uint8_t checksum[2] = {(uint8_t)(sum / 256), (uint8_t)(sum & 255)};
	::write(uart_fd, checksum, 2);

	if (read_at_command() != SATCOM_RESULT_OK) {
		if (verbose) { PX4_INFO("SEND SBD: ERROR WHILE WRITING DATA TO MODEM!"); }

		pthread_mutex_unlock(&tx_buf_mutex);
		return;
	}

	if (rx_command_buf[0] != '0') {
		if (verbose) { PX4_INFO("SEND SBD: ERROR WHILE WRITING DATA TO MODEM! (%d)", rx_command_buf[0] - '0'); }

		pthread_mutex_unlock(&tx_buf_mutex);
		return;
	}

	if (verbose) { PX4_INFO("SEND SBD: DATA WRITTEN TO MODEM"); }

	tx_buf_write_idx = 0;

	pthread_mutex_unlock(&tx_buf_mutex);

	tx_session_pending = true;
}

void satcom::read_rx_buf(void)
{
	if (!is_modem_ready()) {
		if (verbose) { PX4_INFO("READ SBD: MODEM NOT READY!"); }

		return;
	}

	write_at("AT+SBDRB");

	if (read_at_msg() != SATCOM_RESULT_OK) {
		if (verbose) { PX4_INFO("READ SBD: MODEM NOT RESPONDING!"); }

		return;
	}

	int data_len = (rx_msg_buf[0] << 8) + rx_msg_buf[1];

	// rx_buf contains 2 byte lenght, data, 2 byte checksum and /r/n delimiter
	if (data_len != rx_msg_end_idx - 6) {
		if (verbose) { PX4_INFO("READ SBD: WRONG DATA LENGTH"); }

		return;
	}

	int checksum = 0;

	for (int i = 2; i < data_len + 2; i++) {
		checksum += rx_msg_buf[i];
	}

	if ((checksum / 256 != rx_msg_buf[rx_msg_end_idx - 4]) || ((checksum & 255) != rx_msg_buf[rx_msg_end_idx - 3])) {
		if (verbose) { PX4_INFO("READ SBD: WRONG DATA CHECKSUM"); }

		return;
	}

	rx_msg_read_idx = 2;	// ignore the length
	rx_msg_end_idx -= 4;	// ignore the checksum and delimiter
	rx_read_pending = false;

	if (verbose) { PX4_INFO("READ SBD: SUCCESS, LEN: %d", data_len); }
}

bool satcom::clear_mo_buffer()
{
	write_at("AT+SBDD0");

	if (read_at_command() != SATCOM_RESULT_OK || rx_command_buf[0] != '0') {
		if (verbose) { PX4_INFO("CLEAR MO BUFFER: ERROR"); }

		return false;
	}

	return true;
}

void satcom::start_csq(void)
{
	if (verbose) { PX4_INFO("UPDATING SIGNAL QUALITY"); }

	if (!is_modem_ready()) {
		if (verbose) { PX4_INFO("UPDATE SIGNAL QUALITY: MODEM NOT READY!"); }

		return;
	}

	write_at("AT+CSQ");
	new_state = SATCOM_STATE_CSQ;
}

void satcom::start_sbd_session(void)
{
	if (verbose) { PX4_INFO("STARTING SBD SESSION"); }

	if (!is_modem_ready()) {
		if (verbose) { PX4_INFO("SBD SESSION: MODEM NOT READY!"); }

		return;
	}

	if (ring_pending) {
		write_at("AT+SBDIXA");

	} else {
		write_at("AT+SBDIX");
	}

	new_state = SATCOM_STATE_SBDSESSION;
}

void satcom::start_test(void)
{
	int res = read_at_command();

	if (res != SATCOM_RESULT_NA) {
		PX4_WARN("SOMETHING WAS IN BUFFER");
		printf("TEST RESULT: %d, LENGTH %d\nDATA:\n%s\nRAW DATA:\n", res, rx_command_len, rx_command_buf);

		for (int i = 0; i < rx_command_len; i++) {
			printf("%d ", rx_command_buf[i]);
		}

		printf("\n");
	}

	if (!is_modem_ready()) {
		PX4_WARN("MODEM NOT READY!");
		return;
	}

	if (strlen(test_command) != 0) {
		PX4_INFO("TEST %s", test_command);
		write_at(test_command);
		new_state = SATCOM_STATE_TEST;

	} else {
		PX4_INFO("TEST DONE");
	}
}

satcom_uart_status satcom::open_uart(char *uart_name)
{
	if (verbose) { PX4_INFO("opening satcom modem UART: %s", uart_name); }

	uart_fd = ::open(uart_name, O_RDWR | O_BINARY);

	if (uart_fd < 0) {
		if (verbose) { PX4_WARN("UART open failed!"); }

		return SATCOM_UART_OPEN_FAIL;
	}

	// set the UART speed to 19200
	struct termios uart_config;
	tcgetattr(uart_fd, &uart_config);
	cfsetspeed(&uart_config, 19200);
	tcsetattr(uart_fd, TCSANOW, &uart_config);

	if (verbose) { PX4_INFO("UART opened"); }

	return SATCOM_UART_OK;
}

bool satcom::is_modem_ready(void)
{
	write_at("AT");

	if (read_at_command() == SATCOM_RESULT_OK) {
		return true;

	} else {
		return false;
	}
}

void satcom::write_at(const char *command)
{
	if (verbose) { PX4_INFO("WRITING AT COMMAND: %s", command); }

	::write(uart_fd, command, strlen(command));
	::write(uart_fd, "\r", 1);
}

satcom_result_code satcom::read_at_command()
{
	return read_at(rx_command_buf, &rx_command_len);
}

satcom_result_code satcom::read_at_msg()
{
	return read_at(rx_msg_buf, &rx_msg_end_idx);
}

satcom_result_code satcom::read_at(uint8_t *rx_buf, int *rx_len)
{
	struct pollfd fds[1];
	fds[0].fd = uart_fd;
	fds[0].events = POLLIN;

	uint8_t buf = 0;
	int last_rn_idx = 0;
	int rx_buf_pos = 0;
	*rx_len = 0;

	while (1) {
		if (::poll(&fds[0], 1, 100) > 0) {
			if (::read(uart_fd, &buf, 1) > 0) {
				if (rx_buf_pos == 0 && (buf == '\r' || buf == '\n')) {
					// ignore the leading \r\n
					continue;
				}

				rx_buf[rx_buf_pos++] = buf;

				if (rx_buf[rx_buf_pos - 1] == '\n' && rx_buf[rx_buf_pos - 2] == '\r') {
					// found the \r\n delimiter
					if (rx_buf_pos == last_rn_idx + 2)
						; // second in a row, ignore it
					else if (!strncmp((const char *)&rx_buf[last_rn_idx], "OK\r\n", 4)) {
						rx_buf[*rx_len] = 0; 	// null terminator after the information response for printing purposes
						return SATCOM_RESULT_OK;

					} else if (!strncmp((const char *)&rx_buf[last_rn_idx], "ERROR\r\n", 7)) {
						return SATCOM_RESULT_ERROR;

					} else if (!strncmp((const char *)&rx_buf[last_rn_idx], "SBDRING\r\n", 9)) {
						ring_pending = true;
						rx_session_pending = true;

						if (verbose) { PX4_INFO("GET SBDRING");}

						return SATCOM_RESULT_SBDRING;

					} else if (!strncmp((const char *)&rx_buf[last_rn_idx], "READY\r\n", 7)) {
						return SATCOM_RESULT_READY;

					} else if (!strncmp((const char *)&rx_buf[last_rn_idx], "HARDWARE FAILURE", 16)) {
						PX4_WARN("HARDWARE FAILURE!");
						return SATCOM_RESULT_HWFAIL;

					} else {
						*rx_len = rx_buf_pos;	// that was the information response, result code incoming
					}

					last_rn_idx = rx_buf_pos;
				}
			}

		} else {
			break;
		}
	}

	return SATCOM_RESULT_NA;
}

void satcom::schedule_test(void)
{
	test_pending = true;
}

int satcom_main(int argc, char *argv[])
{
	if (!strcmp(argv[1], "start")) {
		return satcom::start(argc, argv);

	} else if (!strcmp(argv[1], "stop")) {
		return satcom::stop();

	} else if (!strcmp(argv[1], "status")) {
		satcom::status();
		return OK;

	} else if (!strcmp(argv[1], "test")) {
		satcom::test(argc, argv);
		return OK;
	}

	PX4_INFO("usage: satcom {start|stop|status|test} [-d uart_device | -v]");

	return PX4_ERROR;
}
