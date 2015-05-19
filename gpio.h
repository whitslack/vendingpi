#include "common/fd.h"


class GPIO : public FileDescriptor {

public:
	enum class Polarity {
		ACTIVE_HIGH, ACTIVE_LOW
	};

	enum class Direction {
		IN, OUT, LOW, HIGH
	};

	enum class Edge {
		NONE, RISING, FALLING, BOTH
	};

private:
	uint pin;

public:
	explicit GPIO(uint pin, Polarity polarity = Polarity::ACTIVE_HIGH, Direction direction = Direction::IN);

public:
	bool value();
	void value(bool value);

	Edge edge();
	void edge(Edge edge);

};
