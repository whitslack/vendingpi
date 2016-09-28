#include <chrono>
#include <ctime>
#include <iostream>
#include <queue>
#include <system_error>
#include <thread>

#include <poll.h>

#include "bitcoin.h"
#include "evdev.h"
#include "led.h"
#include "common/log.h"
#include "common/memory.h"
#include "common/narrow.h"
#include "common/signal.h"
#include "common/termios.h"
#include "common/timer.h"
#include "common/webclient.h"


Log elog(Log::TRACE);

static const char SERIAL_DEVICE[] = "/dev/ttyAMA0";
static const char INPUT_DEVICE[] = "/dev/input/by-path/platform-micromech_input-event";
static constexpr std::chrono::steady_clock::duration session_timeout = std::chrono::seconds(30);

enum MicroMechPin {
	SEND = 3,
	INTERRUPT = 4,
// 	DATA = 5,
	ACCEPT_ENABLE = 6,
	DISPENSE25 = 7,
	DISPENSE10 = 8,
	DISPENSE5 = 9,
	RESET = 11,
};

enum MessageClass {
	COIN_MESSAGE = 0 << 1,
	STATUS_MESSAGE = 1 << 1,
	CLASS_MASK = 1 << 1,
};

enum Status {
	POWER_UP = 0x63,
	DEFECTIVE_SENSOR = 0x67,
	ESCROW_RETURN = 0x6E,
	SLUG = 0x6B,
	NO_STROBE = 0x6F,
	DOUBLE_ARRIVAL = 0x23,
	COIN_JAM = 0x27,
	DOLLAR_COIN_NOT_ACCEPTED = 0x03,
	TUBE25_SENSE_UPPER_FLAG = 1 << 4,
};

static const char * status_str(Status status) {
	switch (status) {
		case POWER_UP:
			return "POWER_UP";
		case DEFECTIVE_SENSOR:
			return "DEFECTIVE_SENSOR";
		case ESCROW_RETURN:
			return "ESCROW_RETURN";
		case SLUG:
			return "SLUG";
		case NO_STROBE:
			return "NO_STROBE";
		case DOUBLE_ARRIVAL:
			return "DOUBLE_ARRIVAL";
		case COIN_JAM:
			return "COIN_JAM";
		case DOLLAR_COIN_NOT_ACCEPTED:
			return "DOLLAR_COIN_NOT_ACCEPTED";
		default:
			return "?!";
	}
}

enum TubeStatus {
	TUBE_STATUS = 0x23,
	TUBE25_NOT_EMPTY = 1 << 2,
	TUBE10_NOT_EMPTY = 1 << 3,
	TUBE5_NOT_EMPTY = 1 << 4,
	TUBE_STATUS_MASK = TUBE25_NOT_EMPTY | TUBE10_NOT_EMPTY | TUBE5_NOT_EMPTY,
};

enum CoinRouting {
	CASH_BOX = 0 << 0,
	INVENTORY_TUBE = 1 << 0,
	ROUTING_MASK = 1 << 0,
};

enum CoinValue {
	COIN100 = 0 << 5,
	COIN25 = 1 << 5,
	COIN10 = 2 << 5,
	COIN5 = 3 << 5,
	COIN_MASK = 3 << 5,
};

struct print_data {
	uint8_t data;
	bool synchronous;
	explicit print_data(uint8_t data, bool synchronous) : data(data), synchronous(synchronous) { }
};
static std::ostream & operator << (std::ostream &os, const print_data &pd) {
	os << std::hex << std::showbase << static_cast<unsigned>(pd.data) << " (";
	if (pd.synchronous) {
		if ((pd.data & CLASS_MASK) == COIN_MESSAGE) {
			switch (pd.data & COIN_MASK) {
				case COIN100:
					os << "COIN100";
					break;
				case COIN25:
					os << "COIN25";
					break;
				case COIN10:
					os << "COIN10";
					break;
				case COIN5:
					os << "COIN5";
					break;
			}
			if ((pd.data & ROUTING_MASK) == CASH_BOX) {
				os << " | CASH_BOX";
			}
			else {
				os << " | INVENTORY_TUBE";
			}
			if (pd.data & TUBE25_NOT_EMPTY) {
				os << " | TUBE25_NOT_EMPTY";
			}
			if (pd.data & TUBE10_NOT_EMPTY) {
				os << " | TUBE10_NOT_EMPTY";
			}
			if (pd.data & TUBE5_NOT_EMPTY) {
				os << " | TUBE5_NOT_EMPTY";
			}
			if (pd.data & ~(CLASS_MASK | COIN_MASK | ROUTING_MASK | TUBE_STATUS_MASK)) {
				os << " | " << static_cast<unsigned>(pd.data & ~(CLASS_MASK | COIN_MASK | ROUTING_MASK | TUBE_STATUS_MASK));
			}
		}
		else {
			os << status_str(static_cast<Status>(pd.data & ~TUBE25_SENSE_UPPER_FLAG));
			if (pd.data & TUBE25_SENSE_UPPER_FLAG) {
				os << " | TUBE25_SENSE_UPPER_FLAG";
			}
		}
	}
	else if ((pd.data & ~TUBE_STATUS_MASK) == TUBE_STATUS) {
		os << "TUBE_STATUS";
		if (pd.data & TUBE25_NOT_EMPTY) {
			os << " | TUBE25_NOT_EMPTY";
		}
		if (pd.data & TUBE10_NOT_EMPTY) {
			os << " | TUBE10_NOT_EMPTY";
		}
		if (pd.data & TUBE5_NOT_EMPTY) {
			os << " | TUBE5_NOT_EMPTY";
		}
	}
	else {
		os << "?!";
	}
	return os << ')' << std::dec;
}

static const struct {
	unsigned tube25_low_threshold, tube25_mid_threshold, tube25_full_threshold, tube25_capacity;
	unsigned tube10_low_threshold, tube10_full_threshold, tube10_capacity;
	unsigned tube5_low_threshold, tube5_full_threshold, tube5_capacity;
}
MARS_TRC_6000 = {
	6, 6, 69, 69,
	7, 99, 99,
	6, 67, 67,
},
COINCO_9300_L = {
	7, 22, 77, 95,
	9, 113, 125,
	7, 78, 86,
},
*tube_levels = &MARS_TRC_6000;

static Bitcoin *bitcoin_ptr;

static constexpr std::chrono::microseconds timeval_to_duration(const struct timeval &tv) {
	return std::chrono::seconds(tv.tv_sec) + std::chrono::microseconds(tv.tv_usec);
}

static constexpr std::chrono::steady_clock::rep microtimestamp(std::chrono::steady_clock::time_point time_point = std::chrono::steady_clock::now()) {
	return std::chrono::duration_cast<std::chrono::microseconds>(time_point.time_since_epoch()).count();
}

static constexpr std::chrono::steady_clock::rep microtimestamp(const struct timeval &tv) {
	return microtimestamp(std::chrono::steady_clock::time_point(timeval_to_duration(tv)));
}

static void handler(int) {
}

static void bitcoin_handler(int, siginfo_t *info, void *) {
	bitcoin_ptr->return_payment(info->si_value.sival_int);
}

int main(int argc, char *argv[]) {
	bool testnet = argc > 1 && std::strcmp(argv[1], "--testnet") == 0;
	if (testnet) {
		argv[1] = argv[0], ++argv, --argc;
	}
	if (argc != 4) {
		std::clog << "usage: " << (argc > 0 ? argv[0] : "vending") << " [--testnet] <host> <port> <bitcoin-privkey>" << std::endl;
		return -1;
	}
	auto host = argv[1];
	auto port = narrow_check<in_port_t>(std::stoi(argv[2]));
	auto bitcoin_privkey = argv[3];

	{
		struct sigaction sa;
		sa.sa_handler = &handler;
		posix::sigemptyset(sa.sa_mask);
		sa.sa_flags = SA_RESTART;
		posix::sigaction(SIGALRM, &sa);
		sa.sa_handler = SIG_IGN;
		posix::sigaction(SIGPIPE, &sa);
		sa.sa_sigaction = &bitcoin_handler;
		sa.sa_flags = SA_RESTART | SA_SIGINFO;
		posix::sigaction(SIGRTMIN, &sa);
		posix::pthread_sigmask(SIG_BLOCK, posix::SignalSet { SIGALRM });
	}

	EventFD event_fd;
#ifndef NO_HARDWARE
	FileDescriptor data_fd(SERIAL_DEVICE, O_RDWR | O_CLOEXEC);
	{
		struct termios tios;
		posix::tcgetattr(data_fd, tios);
		cfmakeraw(&tios);
		posix::cfsetispeed(tios, B600);
		posix::cfsetospeed(tios, B600);
		posix::tcsetattr(data_fd, TCSANOW, tios);
	}
	EvDev input(INPUT_DEVICE);
	LED send_out("SEND_OUT");
	LED interrupt_out("INTERRUPT_OUT");
	LED accept_enable_out("ACCEPT_ENABLE_OUT");
	LED dispense25_out("DISPENSE25_OUT");
	LED dispense10_out("DISPENSE10_OUT");
	LED dispense5_out("DISPENSE5_OUT");
	LED reset_out("RESET_OUT");
	LED dispense_enable_out("DISPENSE_ENABLE_OUT");
	reset_out = true;
	dispense5_out = dispense10_out = dispense25_out = accept_enable_out = interrupt_out = send_out = false;
	dispense_enable_out = true;
#endif

	Bitcoin bitcoin(host, port, testnet, bitcoin_privkey, event_fd);
	bitcoin_ptr = &bitcoin;
	std::thread bitcoin_thread(std::mem_fn(&Bitcoin::run), &bitcoin);

	int credit = 0, virtual_cents_in = 0, virtual_cents_out = 0;

	unsigned tube25_full_threshold = tube_levels->tube25_full_threshold;
	struct {
		unsigned least_25c_coins, most_25c_coins;
		unsigned least_10c_coins, most_10c_coins;
		unsigned least_5c_coins, most_5c_coins;
	} tubes;
	bool tubes_dirty = false;
	try {
		FileDescriptor("tubes").read_fully(&tubes, sizeof tubes);
	}
	catch (...) {
		tubes.least_25c_coins = 0, tubes.most_25c_coins = tube_levels->tube25_capacity;
		tubes.least_10c_coins = 0, tubes.most_10c_coins = tube_levels->tube10_capacity;
		tubes.least_5c_coins = 0, tubes.most_5c_coins = tube_levels->tube5_capacity;
	}

	auto tube_status = [&]() {
		return (virtual_cents_in >= 25 - 2 || tubes.least_25c_coins >= tube_levels->tube25_low_threshold ? TUBE25_NOT_EMPTY : 0) |
				(virtual_cents_in >= 10 - 2 || tubes.least_10c_coins >= tube_levels->tube10_low_threshold ? TUBE10_NOT_EMPTY : 0) |
				(virtual_cents_in >= 5 - 2 || tubes.least_5c_coins >= tube_levels->tube5_low_threshold ? TUBE5_NOT_EMPTY : 0);
	};
	auto process_tube_status = [&](uint8_t data) {
#define _(C) \
		tubes_dirty |= data & TUBE##C##_NOT_EMPTY ? \
				(tubes.least_##C##c_coins < tube_levels->tube##C##_low_threshold && (tubes.least_##C##c_coins = tube_levels->tube##C##_low_threshold, true)) | \
				(tubes.most_##C##c_coins < tube_levels->tube##C##_low_threshold && (tubes.most_##C##c_coins = tube_levels->tube##C##_low_threshold, true)) \
				: \
				(tubes.least_##C##c_coins >= tube_levels->tube##C##_low_threshold && (tubes.least_##C##c_coins = tube_levels->tube##C##_low_threshold - 1, true)) | \
				(tubes.most_##C##c_coins >= tube_levels->tube##C##_low_threshold && (tubes.most_##C##c_coins = tube_levels->tube##C##_low_threshold - 1, true));
		_(25)
		_(10)
		_(5)
#undef _
	};

	enum { IDLE, INTERRUPTING, SENDING, WAITING, QUIESCENT } transmit_state = IDLE;
	std::queue<uint8_t> transmit_queue;
	posix::Timer<std::chrono::steady_clock> transmit_timer;
	std::chrono::steady_clock::time_point transmit_time = std::chrono::steady_clock::time_point::max();

	bool accept_enabled, receiving = false, session_ending = false;

	bool dispensing25_in = false, dispensing25_out = false;
	bool dispensing10_in = false, dispensing10_out = false;
	bool dispensing5_in = false, dispensing5_out = false;
	posix::Timer<std::chrono::steady_clock> dispense_timer;
	std::chrono::steady_clock::time_point dispense_time = std::chrono::steady_clock::time_point::max();

	auto interrupt_changed = [&](bool interrupt) {
		if (receiving != interrupt) {
			receiving = interrupt;
			if (elog.trace_enabled()) {
				elog.trace() << microtimestamp() << " SEND_OUT " << receiving << std::endl;
			}
			send_out = receiving;
		}
	};
	auto accept_enable_changed = [&](bool accept_enable) {
		if (elog.trace_enabled()) {
			elog.trace() << microtimestamp() << " ACCEPT_ENABLE_OUT " << accept_enable << std::endl;
		}
		accept_enable_out = accept_enable;
		if (!accept_enable) {
			// send tube status message
			uint8_t status = static_cast<uint8_t>(TUBE_STATUS | tube_status());
			if (elog.trace_enabled()) {
				elog.trace() << microtimestamp() << " DATA_OUT " << print_data(status, false) << std::endl;
			}
			data_fd.write_fully(&status, sizeof status);
		}
		accept_enabled = accept_enable;
	};
	auto reset_changed = [&](bool reset) {
		if (elog.trace_enabled()) {
			elog.trace() << microtimestamp() << " RESET_OUT " << reset << std::endl;
		}
		reset_out = reset;
	};

	{
		unsigned long bits[(SW_CNT + LONG_BIT - 1) / LONG_BIT];
		input.ioctl(EVIOCGSW(std::size(bits)), bits);
		bool interrupt = bits[INTERRUPT / LONG_BIT] & (1 << INTERRUPT % LONG_BIT);
		bool accept_enable = bits[ACCEPT_ENABLE / LONG_BIT] & (1 << ACCEPT_ENABLE % LONG_BIT);
		bool reset = bits[RESET / LONG_BIT] & (1 << RESET % LONG_BIT);
		if (elog.trace_enabled()) {
			elog.trace() << "INTERRUPT_IN " << interrupt << ',' << " ACCEPT_ENABLE_IN " << accept_enable << ',' << " RESET_IN " << reset << std::endl;
		}
		interrupt_changed(interrupt);
		accept_enable_changed(accept_enable);
		reset_changed(reset);
	}

	struct pollfd pfds[] = {
		{ event_fd, POLLIN, 0 },
#ifndef NO_HARDWARE
		{ data_fd, POLLIN, 0 },
		{ input, POLLIN, 0 },
#endif
		{ STDIN_FILENO, POLLIN, 0 },
	};
	nfds_t nfds = std::size(pfds);
	for (;;) {
		if (posix::ppoll(pfds, nfds, nullptr, posix::SignalSet{ }) > 0) {
			if (pfds[0].revents) {
				auto counter = event_fd.read();
				if (counter > 0) {
					if (elog.info_enabled()) {
						elog.info() << "credit " << counter << std::endl;
					}
					credit += static_cast<int>(counter);
					virtual_cents_in += static_cast<int>(counter);
				}
			}
#ifndef NO_HARDWARE
			if (pfds[1].revents) {
				uint8_t data;
				if (data_fd.read(&data, sizeof data) > 0) {
					if (elog.trace_enabled()) {
						elog.trace() << microtimestamp() << " DATA_IN " << print_data(data, receiving) << std::endl;
					}
					if (receiving) {
						receiving = false;
						if (elog.trace_enabled()) {
							elog.trace() << microtimestamp() << " SEND_OUT " << false << std::endl;
						}
						send_out = false;
						if ((data & CLASS_MASK) != COIN_MESSAGE) { // non-coin event
							transmit_queue.push(data);
							tube25_full_threshold = (data & TUBE25_SENSE_UPPER_FLAG) ? tube_levels->tube25_full_threshold : tube_levels->tube25_mid_threshold;
							if ((data & ~TUBE25_SENSE_UPPER_FLAG) == ESCROW_RETURN) {
								session_ending = true;
							}
						}
						else { // coin event
							transmit_queue.push(static_cast<uint8_t>(data | tube_status()));
							switch (data & COIN_MASK) {
#define _(C, F) \
								case COIN##C: \
									tubes_dirty |= (data & ROUTING_MASK) == CASH_BOX ? \
											(tubes.least_##C##c_coins < F && (tubes.least_##C##c_coins = F, true)) | \
											(tubes.most_##C##c_coins < F && (tubes.most_##C##c_coins = F, true)) \
											: \
											(tubes.least_##C##c_coins < F && (++tubes.least_##C##c_coins, true)) | \
											(tubes.most_##C##c_coins < F && (++tubes.most_##C##c_coins, true)); \
									break;
								_(25, tube25_full_threshold)
								_(10, tube_levels->tube10_full_threshold)
								_(5, tube_levels->tube5_full_threshold)
#undef _
							}
							process_tube_status(data);
						}
						if (!session_ending) {
							// (re)set session timeout
							dispense_timer.set(dispense_time = std::chrono::steady_clock::now() + session_timeout);
						}
					}
					else if ((data & ~TUBE_STATUS_MASK) == TUBE_STATUS) {
						process_tube_status(data);
					}
				}
			}
			if (pfds[2].revents) {
				auto event = input.read_event();
				if (event.type == EV_SW) {
					switch (static_cast<MicroMechPin>(event.code)) {
						case SEND: {
							bool send = event.value;
							if (elog.trace_enabled()) {
								elog.trace() << microtimestamp(event.time) << " SEND_IN " << send << std::endl;
							}
							if (send) {
								// VMC is ready to receive
								if (transmit_state == INTERRUPTING || transmit_state == WAITING) {
									transmit_state = SENDING;
									transmit_timer.clear();
									if (elog.trace_enabled()) {
										elog.trace() << microtimestamp() << " DATA_OUT " << print_data(transmit_queue.front(), true) << std::endl;
									}
									data_fd.write_fully(&transmit_queue.front(), sizeof(uint8_t));
								}
							}
							else if (transmit_state == SENDING) {
								// VMC has received a byte; wait ~5 ms in case VMC requests retransmission
								transmit_state = WAITING;
								transmit_timer.set(transmit_time = std::chrono::steady_clock::time_point(timeval_to_duration(event.time)) + std::chrono::microseconds(4500)); // 5 ms +/- 0.5 ms
							}
							break;
						}
						case INTERRUPT: {
							bool interrupt = event.value;
							if (elog.trace_enabled()) {
								elog.trace() << microtimestamp(event.time) << " INTERRUPT_IN " << interrupt << std::endl;
							}
							interrupt_changed(interrupt);
							break;
						}
						case ACCEPT_ENABLE: {
							bool accept_enable = event.value;
							if (elog.trace_enabled()) {
								elog.trace() << microtimestamp(event.time) << " ACCEPT_ENABLE_IN " << accept_enable << std::endl;
							}
							accept_enable_changed(accept_enable);
							break;
						}
#define _(C) \
						case DISPENSE##C: { \
							bool dispense##C = event.value; \
							if (elog.trace_enabled()) { \
								elog.trace() << microtimestamp(event.time) << " DISPENSE" #C "_IN " << dispense##C << std::endl; \
							} \
							if (dispense##C != dispensing##C##_in) { \
								if ((dispensing##C##_in = dispense##C)) { \
									if (session_ending) { \
										if (virtual_cents_in >= C - 2) { \
											auto cents = std::min(virtual_cents_in, C); \
											virtual_cents_in -= cents, virtual_cents_out += cents, credit += C - cents; \
										} \
										else { \
											if (elog.trace_enabled()) { \
												elog.trace() << microtimestamp() << " DISPENSE" #C "_OUT " << true << std::endl; \
											} \
											dispense##C##_out = dispensing##C##_out = true; \
											tubes_dirty |= (tubes.least_##C##c_coins > 0 && (--tubes.least_##C##c_coins, true)) | \
													(tubes.most_##C##c_coins > 0 && (--tubes.most_##C##c_coins, true)); \
										} \
										dispense_timer.set(dispense_time = std::chrono::steady_clock::time_point(timeval_to_duration(event.time)) + std::chrono::seconds(1)); \
									} \
									else { \
										credit += C; \
									} \
								} \
								else if (dispensing##C##_out) { \
									if (elog.trace_enabled()) { \
										elog.trace() << microtimestamp() << " DISPENSE" #C "_OUT " << false << std::endl; \
									} \
									dispense##C##_out = dispensing##C##_out = false; \
								} \
							} \
							break; \
						}
						_(25)
						_(10)
						_(5)
#undef _
						case RESET: {
							bool reset = event.value;
							if (elog.trace_enabled()) {
								elog.trace() << microtimestamp(event.time) << " RESET_IN " << reset << std::endl;
							}
							reset_changed(reset);
							break;
						}
					}
				}
			}
#endif
			if (pfds[std::size(pfds) - 1].revents) {
				int cents;
				if (!(std::cin >> cents)) {
					if (elog.debug_enabled()) {
						elog.debug() << "stdin reached EOF" << std::endl;
					}
					pfds[std::size(pfds) - 1].revents = 0;
					--nfds;
				}
				else if (cents > 0) {
					// simulate a payment
					event_fd.write(cents);
				}
				else if (cents < 0) {
					// force return of change
					posix::pthread_sigqueue(bitcoin_thread.native_handle(), SIGRTMIN, -cents);
				}
				else {
					// force session timeout
					dispense_time = std::chrono::steady_clock::time_point::min();
				}
			}
		}
#ifndef NO_HARDWARE
		if (tubes_dirty) {
			if (elog.debug_enabled()) {
				auto debug = elog.debug();
				debug << "tube status: 25c:" << tubes.least_25c_coins;
				if (tubes.most_25c_coins != tubes.least_25c_coins) {
					debug << '-' << tubes.most_25c_coins;
				}
				debug << ", 10c:" << tubes.least_10c_coins;
				if (tubes.most_10c_coins != tubes.least_10c_coins) {
					debug << '-' << tubes.most_10c_coins;
				}
				debug << ", 5c:" << tubes.least_5c_coins;
				if (tubes.most_5c_coins != tubes.least_5c_coins) {
					debug << '-' << tubes.most_5c_coins;
				}
				debug << std::endl;
			}
			FileDescriptor("tubes", O_WRONLY | O_CREAT | O_CLOEXEC, 0644).write_fully(&tubes, sizeof tubes);
			tubes_dirty = false;
		}
		auto now = std::chrono::steady_clock::now();
		if (now >= dispense_time) {
			if (session_ending) {
				// VMC has finished dispensing change; return to customer
				auto cents = std::min(credit, virtual_cents_in);
				virtual_cents_out += cents, credit -= cents;
				if (virtual_cents_out > 0) {
					if (elog.info_enabled()) {
						elog.info() << "return " << virtual_cents_out << std::endl;
					}
					posix::pthread_sigqueue(bitcoin_thread.native_handle(), SIGRTMIN, virtual_cents_out);
				}
				virtual_cents_in = virtual_cents_out = 0;
				session_ending = false;
			}
			else {
				if (elog.info_enabled()) {
					elog.info() << "session timed out" << std::endl;
				}
				transmit_queue.push(ESCROW_RETURN);
				session_ending = true;
			}
			dispense_time = std::chrono::steady_clock::time_point::max();
		}
		switch (transmit_state) {
			case QUIESCENT:
				if (now < transmit_time) {
					break;
				}
				transmit_state = IDLE;
				// fall through
			case IDLE:
				if (transmit_queue.empty()) {
					if (credit < 5 - 2 || !accept_enabled || session_ending) {
						break;
					}
#if 0 // reporting dollar coin insertions prevents coin return from working
					if (credit >= 100 - 2) {
						transmit_queue.push(static_cast<uint8_t>(CASH_BOX | COIN100 | tube_status()));
						credit -= 100;
					}
					else
#endif
					if (credit >= 25 - 2) {
						transmit_queue.push(static_cast<uint8_t>(INVENTORY_TUBE | COIN25 | tube_status()));
						credit -= 25;
					}
					else if (credit >= 10 - 2) {
						transmit_queue.push(static_cast<uint8_t>(INVENTORY_TUBE | COIN10 | tube_status()));
						credit -= 10;
					}
					else {
						transmit_queue.push(static_cast<uint8_t>(INVENTORY_TUBE | COIN5 | tube_status()));
						credit -= 5;
					}
					// (re)set session timeout
					dispense_timer.set(dispense_time = now + session_timeout);
				}
				// we have a byte to send; interrupt the VMC
				transmit_state = INTERRUPTING;
				if (elog.trace_enabled()) {
					elog.trace() << microtimestamp() << " INTERRUPT_OUT " << true << std::endl;
				}
				interrupt_out = true;
				break;
			case WAITING:
				if (now >= transmit_time) {
					// VMC did not request retransmission; assume byte was successfully received
					if ((transmit_queue.front() & ~TUBE25_SENSE_UPPER_FLAG) == ESCROW_RETURN) {
						dispense_timer.set(dispense_time = now + std::chrono::seconds(2));
					}
					transmit_state = QUIESCENT;
					transmit_queue.pop();
					if (elog.trace_enabled()) {
						elog.trace() << microtimestamp() << " INTERRUPT_OUT " << false << std::endl;
					}
					interrupt_out = false;
					// minimum quiescent period is unspecified; 20 ms is known to be too short; assume 50 ms is safe
					transmit_timer.set(transmit_time = now + std::chrono::milliseconds(50));
				}
				break;
			default:
				break;
		}
#endif
	}
}
