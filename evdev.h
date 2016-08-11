#include <linux/input.h>

#include "common/fd.h"


class EvDev : public FileDescriptor {

public:
	explicit EvDev(const char path[]);

public:
	struct input_event read_event();

};
