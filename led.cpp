#include "led.h"

#include <system_error>


LED::LED(const std::string &name) : FileDescriptor(("/sys/class/leds/" + name + "/brightness").c_str(), O_RDWR | O_CLOEXEC) {
}

unsigned LED::brightness() const {
	char buf[16];
	auto r = this->pread(buf, sizeof buf - 1, 0);
	if (r <= 0) {
		throw std::runtime_error("failed to read LED brightness");
	}
	buf[r] = '\0';
	return static_cast<unsigned>(std::strtoul(buf, nullptr, 10));
}

LED & LED::brightness(unsigned brightness) {
	if (brightness < 10) {
		char c = static_cast<char>('0' + brightness);
		this->pwrite_fully(&c, sizeof c, 0);
	}
	else {
		std::string str = std::to_string(brightness);
		this->pwrite_fully(str.data(), str.size(), 0);
	}
	return *this;
}
