#include "gpio.h"

#include <system_error>


static const char * direction_to_string(GPIO::Direction direction) {
	switch (direction) {
		case GPIO::Direction::IN:
			return "in";
		case GPIO::Direction::OUT:
			return "out";
		case GPIO::Direction::LOW:
			return "low";
		case GPIO::Direction::HIGH:
			return "high";
	}
	throw std::invalid_argument("invalid direction");
}

static const char * edge_to_string(GPIO::Edge edge) {
	switch (edge) {
		case GPIO::Edge::NONE:
			return "none";
		case GPIO::Edge::RISING:
			return "rising";
		case GPIO::Edge::FALLING:
			return "falling";
		case GPIO::Edge::BOTH:
			return "both";
	}
	throw std::invalid_argument("invalid edge");
}

static GPIO::Edge string_to_edge(const char str[]) {
	switch (str[0]) {
		case 'b':
			if (std::strcmp(str + 1, "both" + 1) == 0) {
				return GPIO::Edge::BOTH;
			}
			break;
		case 'f':
			if (std::strcmp(str + 1, "falling" + 1) == 0) {
				return GPIO::Edge::FALLING;
			}
			break;
		case 'n':
			if (std::strcmp(str + 1, "none" + 1) == 0) {
				return GPIO::Edge::NONE;
			}
			break;
		case 'r':
			if (std::strcmp(str + 1, "rising" + 1) == 0) {
				return GPIO::Edge::RISING;
			}
			break;
	}
	throw std::invalid_argument("invalid edge");
}


GPIO::GPIO(unsigned pin, GPIO::Polarity polarity, GPIO::Direction direction) : pin(pin) {
	std::string path("/sys/class/gpio/gpio"), pin_str = std::to_string(pin);
	path += pin_str;
	try {
		FileDescriptor("/sys/class/gpio/export", O_WRONLY | O_CLOEXEC).write_fully(pin_str.data(), pin_str.size());
	}
	catch (const std::system_error &e) {
		if (e.code().value() != EBUSY) {
			throw;
		}
	}
	{
		const char buf = polarity == Polarity::ACTIVE_LOW ? '1' : '0';
		FileDescriptor((path + "/active_low").c_str(), O_WRONLY | O_CLOEXEC).write_fully(&buf, sizeof buf);
	}
	{
		const char *direction_str = direction_to_string(direction);
		FileDescriptor((path + "/direction").c_str(), O_WRONLY | O_CLOEXEC).write_fully(direction_str, std::strlen(direction_str));
	}
	this->open((path + "/value").c_str(), (direction == Direction::IN ? O_RDONLY : O_RDWR) | O_CLOEXEC);
}

bool GPIO::value() {
	char buf;
	this->pread(&buf, sizeof buf, 0);
	return buf != '0';
}

void GPIO::value(bool value) {
	const char buf = value ? '1' : '0';
	this->pwrite(&buf, sizeof buf, 0);
}

GPIO::Edge GPIO::edge() {
	char buf[8];
	std::memset(buf, 0, sizeof buf);
	FileDescriptor(("/sys/class/gpio/gpio" + std::to_string(pin) + "/edge").c_str(), O_RDONLY | O_CLOEXEC).read(buf, sizeof buf - 1);
	return string_to_edge(buf);
}

void GPIO::edge(GPIO::Edge edge) {
	const char *str = edge_to_string(edge);
	FileDescriptor(("/sys/class/gpio/gpio" + std::to_string(pin) + "/edge").c_str(), O_WRONLY | O_CLOEXEC).write_fully(str, std::strlen(str));
}
