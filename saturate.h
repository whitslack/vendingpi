#include <limits>
#include <stdexcept>


template <typename V, V min = std::numeric_limits<V>::min(), V max = std::numeric_limits<V>::max()>
class Saturating {

public:
	typedef V value_type;

public:
	static constexpr V min_value = min, max_value = max;

private:
	V value;

public:
	constexpr Saturating() : value() { }
	constexpr Saturating(V value) : value(value < min ? min : value > max ? max : value) { }

public:
	operator V () const { return value; }

	Saturating & operator ++ () {
		if (value < max) {
			++value;
		}
		return *this;
	}

	Saturating & operator -- () {
		if (value > min) {
			--value;
		}
		return *this;
	}

	Saturating operator ++ (int) {
		Saturating ret(*this);
		++*this;
		return ret;
	}

	Saturating operator -- (int) {
		Saturating ret(*this);
		--*this;
		return ret;
	}

	Saturating & operator += (V addend) {
		value = value < max - addend ? static_cast<V>(value + addend) : max;
		return *this;
	}

	Saturating & operator -= (V subtrahend) {
		value = value > min + subtrahend ? static_cast<V>(value - subtrahend) : min;
		return *this;
	}

	Saturating operator + (V addend) const {
		Saturating sum(*this);
		sum += addend;
		return sum;
	}

	Saturating operator - (V subtrahend) const {
		Saturating difference(*this);
		difference -= subtrahend;
		return difference;
	}

};
