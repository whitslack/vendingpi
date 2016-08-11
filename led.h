#include <string>

#include "common/fd.h"


class LED : public FileDescriptor {

public:
	explicit LED(const std::string &name);

public:
	unsigned brightness() const;
	LED & brightness(unsigned brightness);

	operator int () const { return this->brightness(); }
	int operator = (int brightness) { this->brightness(brightness); return brightness; }

};
