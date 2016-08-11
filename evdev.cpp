#include "evdev.h"

EvDev::EvDev(const char path[]) : FileDescriptor(path) {
	unsigned id = CLOCK_MONOTONIC;
	this->ioctl(EVIOCSCLOCKID, &id);
}

struct input_event EvDev::read_event() {
	struct input_event event;
	this->read_fully(&event, sizeof event);
	return event;
}
