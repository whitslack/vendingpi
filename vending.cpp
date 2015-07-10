#if (RPI_REV != 1) && (RPI_REV != 2)
#error Must specify Raspberry Pi revision in RPI_REV macro.
#endif

#include <chrono>
#include <ctime>
#include <iostream>
#include <queue>
#include <system_error>
#include <thread>

#include <poll.h>

#include "bitcoin.h"
#include "gpio.h"
#include "saturate.h"
#include "common/log.h"
#include "common/narrow.h"
#include "common/signal.h"
#include "common/termios.h"
#include "common/timer.h"
#include "common/webclient.h"


Log elog(Log::TRACE);

const char SERIAL_DEVICE[] = "/dev/ttyAMA0";

enum Pin {
	PIN_SEND_IN = 22,
	PIN_INTERRUPT_OUT = 24,
	PIN_DATA_OUT = 14, // TXD
	PIN_ACCEPT_ENABLE_IN = 10,
	PIN_DISPENSE25_IN = 4,
	PIN_DISPENSE10_IN = 17,
	PIN_DISPENSE5_IN = 9,
#if RPI_REV == 1
	PIN_RESET_IN = 1,
#elif RPI_REV == 2
	PIN_RESET_IN = 3,
#endif
	PIN_SEND_OUT = 18,
	PIN_INTERRUPT_IN = 11,
	PIN_DATA_IN = 15, // RXD
	PIN_ACCEPT_ENABLE_OUT = 25,
	PIN_DISPENSE25_OUT = 23,
	PIN_DISPENSE10_OUT = 8,
	PIN_DISPENSE5_OUT = 7,
#if RPI_REV == 1
	PIN_RESET_OUT = 21,
#elif RPI_REV == 2
	PIN_RESET_OUT = 27,
#endif
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
	TUBE25_SENSE_UPPER = 1 << 4,
};

enum TubeStatus {
	TUBE_STATUS = 0x23,
	TUBE5_NOT_EMPTY = 1 << 4,
	TUBE10_NOT_EMPTY = 1 << 3,
	TUBE25_NOT_EMPTY = 1 << 2,
};

enum CoinRouting {
	CASH_BOX = 0 << 0,
	INVENTORY_TUBE = 1 << 0,
};

enum CoinValue {
	COIN5 = 3 << 5,
	COIN10 = 2 << 5,
	COIN25 = 1 << 5,
	COIN100 = 0 << 5,
};

extern const char _binary_wwwroot_index_html_start[], _binary_wwwroot_index_html_end[];

static Bitcoin *bitcoin_ptr;

static inline std::chrono::steady_clock::rep microtimestamp() {
	return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
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
	auto port = narrow_check<uint16_t>(std::stoi(argv[2]));
	auto bitcoin_privkey = argv[3];

	{
		struct sigaction sa;
		sa.sa_handler = &handler;
		posix::sigemptyset(sa.sa_mask);
		sa.sa_flags = SA_RESTART;
		posix::sigaction(SIGALRM, &sa);
		sa.sa_sigaction = &bitcoin_handler;
		sa.sa_flags = SA_RESTART | SA_SIGINFO;
		posix::sigaction(SIGRTMIN, &sa);
		posix::pthread_sigmask(SIG_BLOCK, { SIGALRM });
	}

	EventFD event_fd;
#ifndef NO_HARDWARE
	FileDescriptor data_fd(SERIAL_DEVICE, O_RDWR | O_CLOEXEC);
	GPIO send_in(PIN_SEND_IN, GPIO::Polarity::ACTIVE_LOW, GPIO::Direction::IN);
	GPIO interrupt_in(PIN_INTERRUPT_IN, GPIO::Polarity::ACTIVE_LOW, GPIO::Direction::IN);
	GPIO accept_enable_in(PIN_ACCEPT_ENABLE_IN, GPIO::Polarity::ACTIVE_LOW, GPIO::Direction::IN);
	GPIO dispense25_in(PIN_DISPENSE25_IN, GPIO::Polarity::ACTIVE_LOW, GPIO::Direction::IN);
	GPIO dispense10_in(PIN_DISPENSE10_IN, GPIO::Polarity::ACTIVE_LOW, GPIO::Direction::IN);
	GPIO dispense5_in(PIN_DISPENSE5_IN, GPIO::Polarity::ACTIVE_LOW, GPIO::Direction::IN);
	GPIO reset_in(PIN_RESET_IN, GPIO::Polarity::ACTIVE_HIGH, GPIO::Direction::IN);
	GPIO send_out(PIN_SEND_OUT, GPIO::Polarity::ACTIVE_LOW, GPIO::Direction::HIGH);
	GPIO interrupt_out(PIN_INTERRUPT_OUT, GPIO::Polarity::ACTIVE_LOW, GPIO::Direction::HIGH);
	GPIO accept_enable_out(PIN_ACCEPT_ENABLE_OUT, GPIO::Polarity::ACTIVE_LOW, GPIO::Direction::HIGH);
	GPIO dispense25_out(PIN_DISPENSE25_OUT, GPIO::Polarity::ACTIVE_LOW, GPIO::Direction::HIGH);
	GPIO dispense10_out(PIN_DISPENSE10_OUT, GPIO::Polarity::ACTIVE_LOW, GPIO::Direction::HIGH);
	GPIO dispense5_out(PIN_DISPENSE5_OUT, GPIO::Polarity::ACTIVE_LOW, GPIO::Direction::HIGH);
	GPIO reset_out(PIN_RESET_OUT, GPIO::Polarity::ACTIVE_HIGH, GPIO::Direction::LOW);
	{
		struct termios tios;
		posix::tcgetattr(data_fd, tios);
		cfmakeraw(&tios);
		posix::cfsetispeed(tios, B600);
		posix::cfsetospeed(tios, B600);
		posix::tcsetattr(data_fd, TCSANOW, tios);
	}
	send_in.edge(GPIO::Edge::BOTH);
	interrupt_in.edge(GPIO::Edge::BOTH);
	accept_enable_in.edge(GPIO::Edge::BOTH);
	dispense25_in.edge(GPIO::Edge::BOTH);
	dispense10_in.edge(GPIO::Edge::BOTH);
	dispense5_in.edge(GPIO::Edge::BOTH);
	reset_in.edge(GPIO::Edge::BOTH);
#endif

	Bitcoin bitcoin(host, port, testnet, bitcoin_privkey, event_fd);
	bitcoin_ptr = &bitcoin;
	std::thread bitcoin_thread(std::mem_fn(&Bitcoin::run), &bitcoin);

// 	Saturating<unsigned, 0, 86> least_5c_coins(0), most_5c_coins(86);
// 	Saturating<unsigned, 0, 125> least_10c_coins(0), most_10c_coins(125);
// 	Saturating<unsigned, 0, 95> least_25c_coins(0), most_25c_coins(95);

	int credit = 0, dispense = 0;

	enum { IDLE, INTERRUPTING, SENDING, WAITING, QUIESCENT } transmit_state = IDLE;
	std::queue<uint8_t> transmit_queue;
	posix::Timer<> transmit_timer;
	std::chrono::steady_clock::time_point transmit_time = std::chrono::steady_clock::now();

	bool dispensing25 = false, dispensing10 = false, dispensing5 = false;
	posix::Timer<> dispense_timer;
	std::chrono::steady_clock::time_point dispense_time = transmit_time;

	struct pollfd pfds[] = {
		{ event_fd, POLLIN, 0 },
#ifndef NO_HARDWARE
		{ data_fd, POLLIN, 0 },
		{ send_in, POLLPRI, 0 },
		{ interrupt_in, POLLPRI, 0 },
		{ accept_enable_in, POLLPRI, 0 },
		{ dispense25_in, POLLPRI, 0 },
		{ dispense10_in, POLLPRI, 0 },
		{ dispense5_in, POLLPRI, 0 },
		{ reset_in, POLLPRI, 0 },
#endif
		{ STDIN_FILENO, POLLIN, 0 },
	};
	nfds_t nfds = sizeof pfds / sizeof *pfds;
	for (;;) {
		if (posix::ppoll(pfds, nfds, nullptr, posix::SignalSet{ }) > 0) {
			if (pfds[0].revents) {
				auto counter = event_fd.read();
				if (counter > 0) {
					if (elog.info_enabled()) {
						elog.info() << "credit " << counter << std::endl;
					}
					credit += static_cast<int>(counter);
					for (; credit >= 100 - 1; credit -= 100) {
						transmit_queue.push(CASH_BOX | COIN100 | TUBE5_NOT_EMPTY | TUBE10_NOT_EMPTY | TUBE25_NOT_EMPTY);
					}
					for (; credit >= 25 - 1; credit -= 25) {
						transmit_queue.push(CASH_BOX | COIN25 | TUBE5_NOT_EMPTY | TUBE10_NOT_EMPTY | TUBE25_NOT_EMPTY);
					}
					for (; credit >= 10 - 1; credit -= 10) {
						transmit_queue.push(CASH_BOX | COIN10 | TUBE5_NOT_EMPTY | TUBE10_NOT_EMPTY | TUBE25_NOT_EMPTY);
					}
					for (; credit >= 5 - 1; credit -= 5) {
						transmit_queue.push(CASH_BOX | COIN5 | TUBE5_NOT_EMPTY | TUBE10_NOT_EMPTY | TUBE25_NOT_EMPTY);
					}
				}
			}
#ifndef NO_HARDWARE
			if (pfds[1].revents) {
				uint8_t data;
				if (data_fd.read(&data, sizeof data) > 0) {
					if (elog.trace_enabled()) {
						elog.trace() << microtimestamp() << " DATA_IN " << std::hex << std::showbase << static_cast<uint>(data) << std::dec << std::endl;
					}
				}
			}
			if (pfds[2].revents) {
				bool send = send_in.value();
				if (elog.trace_enabled()) {
					elog.trace() << microtimestamp() << " SEND_IN " << send << std::endl;
				}
				if (send) {
					// VMC is ready to receive
					if (transmit_state == INTERRUPTING || transmit_state == WAITING) {
						transmit_state = SENDING;
						transmit_timer.clear();
						if (elog.trace_enabled()) {
							elog.trace() << microtimestamp() << " DATA_OUT " << std::hex << std::showbase << static_cast<uint>(transmit_queue.front()) << std::dec << std::endl;
						}
						data_fd.write_fully(&transmit_queue.front(), sizeof(uint8_t));
					}
				}
				else if (transmit_state == SENDING) {
					// VMC has received a byte; wait ~5 ms in case VMC requests retransmission
					transmit_state = WAITING;
					transmit_timer.set(transmit_time = std::chrono::steady_clock::now() + std::chrono::microseconds(4500)); // 5 ms +/- 0.5 ms
				}
			}
			if (pfds[3].revents) {
				bool interrupt = interrupt_in.value();
				if (elog.trace_enabled()) {
					elog.trace() << microtimestamp() << " INTERRUPT_IN " << interrupt << std::endl;
				}
			}
			if (pfds[4].revents) {
				bool accept_enable = accept_enable_in.value();
				if (elog.trace_enabled()) {
					elog.trace() << microtimestamp() << " ACCEPT_ENABLE_IN " << accept_enable << std::endl;
				}
				if (!accept_enable) {
					// don't penalize the next payment if the last one was rounded up
					if (credit < 0) {
						credit = 0;
					}
					// send tube status message
					uint8_t tube_status = TUBE_STATUS | TUBE5_NOT_EMPTY | TUBE10_NOT_EMPTY | TUBE25_NOT_EMPTY;
					if (elog.trace_enabled()) {
						elog.trace() << microtimestamp() << " DATA_OUT " << std::hex << std::showbase << static_cast<uint>(tube_status) << std::dec << std::endl;
					}
					data_fd.write_fully(&tube_status, sizeof tube_status);
				}
			}
			if (pfds[5].revents) {
				bool dispense25 = dispense25_in.value();
				if (elog.trace_enabled()) {
					elog.trace() << microtimestamp() << " DISPENSE25_IN " << dispense25 << std::endl;
				}
				if (dispense25 != dispensing25 && (dispensing25 = dispense25)) {
					dispense += 25;
					dispense_timer.set(dispense_time = std::chrono::steady_clock::now() + std::chrono::seconds(1));
				}
			}
			if (pfds[6].revents) {
				bool dispense10 = dispense10_in.value();
				if (elog.trace_enabled()) {
					elog.trace() << microtimestamp() << " DISPENSE10_IN " << dispense10 << std::endl;
				}
				if (dispense10 != dispensing10 && (dispensing10 = dispense10)) {
					dispense += 10;
					dispense_timer.set(dispense_time = std::chrono::steady_clock::now() + std::chrono::seconds(1));
				}
			}
			if (pfds[7].revents) {
				bool dispense5 = dispense5_in.value();
				if (elog.trace_enabled()) {
					elog.trace() << microtimestamp() << " DISPENSE5_IN " << dispense5 << std::endl;
				}
				if (dispense5 != dispensing5 && (dispensing5 = dispense5)) {
					dispense += 5;
					dispense_timer.set(dispense_time = std::chrono::steady_clock::now() + std::chrono::seconds(1));
				}
			}
			if (pfds[8].revents) {
				bool reset = reset_in.value();
				if (elog.trace_enabled()) {
					elog.trace() << microtimestamp() << " RESET_IN " << reset << std::endl;
				}
				if (!reset) {
					transmit_queue.push(POWER_UP | TUBE25_SENSE_UPPER);
				}
			}
#endif
			if (pfds[sizeof pfds / sizeof *pfds - 1].revents) {
				int cents;
				if (!(std::cin >> cents)) {
					if (elog.debug_enabled()) {
						elog.debug() << "stdin reached EOF" << std::endl;
					}
					pfds[sizeof pfds / sizeof *pfds - 1].revents = 0;
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
					// simulate "escrow return" signal from customer
					transmit_queue.push(ESCROW_RETURN | TUBE25_SENSE_UPPER);
				}
			}
		}
#ifndef NO_HARDWARE
		switch (transmit_state) {
			case QUIESCENT:
				if (std::chrono::steady_clock::now() < transmit_time) {
					break;
				}
				transmit_state = IDLE;
				// fall through
			case IDLE:
				if (!transmit_queue.empty()) {
					// we have a byte to send; interrupt the VMC
					transmit_state = INTERRUPTING;
					if (elog.trace_enabled()) {
						elog.trace() << microtimestamp() << " INTERRUPT_OUT " << true << std::endl;
					}
					interrupt_out.value(true);
				}
				break;
			case WAITING:
				if (std::chrono::steady_clock::now() >= transmit_time) {
					// VMC did not request retransmission; assume byte was successfully received
					transmit_state = QUIESCENT;
					transmit_queue.pop();
					if (elog.trace_enabled()) {
						elog.trace() << microtimestamp() << " INTERRUPT_OUT " << false << std::endl;
					}
					interrupt_out.value(false);
					// minimum quiescent period is unspecified; 20 ms is known to be too short; assume 50 ms is safe
					transmit_timer.set(transmit_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(50));
				}
				break;
			default:
				break;
		}
		if (dispense > 0 && std::chrono::steady_clock::now() >= dispense_time) {
			// VMC has finished dispensing change; return to customer
			dispense += credit, credit = 0;
			if (elog.info_enabled()) {
				elog.info() << "return " << dispense << std::endl;
			}
			posix::pthread_sigqueue(bitcoin_thread.native_handle(), SIGRTMIN, dispense);
			dispense = 0;
		}
#endif
	}
}
