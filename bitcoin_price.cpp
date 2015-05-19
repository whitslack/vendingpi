#include "bitcoin.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <thread>

#include "common/connect.h"
#include "common/json.h"
#include "common/log.h"
#include "common/webclient.h"

extern Log elog;


namespace {
	static const std::string method_get("GET"), protocol_version("HTTP/1.1"), connection_header("Connection"), connection_close("close"), host_header("Host"), user_agent_header("User-Agent"), user_agent("VendingPi/" VERSION " (vendingpi@mattwhitlock.name)");

	class PriceSource {
	public:
		static constexpr std::chrono::steady_clock::duration MIN_INTERVAL = std::chrono::seconds(1), MAX_INTERVAL = std::chrono::minutes(1);
	private:
		const std::string name, host, request_uri;
		const uint16_t port;
		std::chrono::nanoseconds interval;
		std::chrono::steady_clock::time_point last_check, expires;
		double price;
	protected:
		PriceSource(const std::string &name, const std::string &host, uint16_t port, const std::string &request_uri) :
				name(name), host(host), request_uri(request_uri), port(port), interval(MIN_INTERVAL), last_check(std::chrono::steady_clock::now() - MIN_INTERVAL), expires(last_check), price(NAN) { }
	public:
		double get() {
			auto now = std::chrono::steady_clock::now();
			if (now >= expires) {
				std::chrono::seconds ttl;
				auto delay = last_check + interval - now;
				if (delay > std::chrono::steady_clock::duration::zero()) {
					std::this_thread::sleep_for(delay);
					now = std::chrono::steady_clock::now();
				}
				last_check = now;
				try {
					HttpRequestHeaders request_headers(method_get, request_uri, protocol_version);
					request_headers.emplace_hint(request_headers.end(), connection_header, connection_close);
					request_headers.emplace_hint(request_headers.end(), host_header, host);
					request_headers.emplace_hint(request_headers.end(), user_agent_header, user_agent);
					if (elog.debug_enabled()) {
						elog.debug() << method_get << ' ' << HttpsConnection::protocol_name << "://" << host << ':' << port << request_uri << std::endl;
					}
					HttpsConnection conn(connect(host.c_str(), port, std::chrono::duration_cast<std::chrono::microseconds>(interval)), host);
					conn.request(request_headers);
					auto &response_headers = conn.get_response_headers();
					if (response_headers.status_code != 200) {
						throw std::ios_base::failure("server returned status code " + std::to_string(response_headers.status_code));
					}
					SourceBuf sb(conn);
					char buf[1500];
					sb.pubsetbuf(buf, sizeof buf);
					json::ValuePtr value_ptr;
					std::istream(&sb) >> value_ptr;
					price = this->parse(*value_ptr);
					HttpHeaders::const_iterator date_itr, expires_itr;
					if ((date_itr = response_headers.find("Date")) != response_headers.end() && (expires_itr = response_headers.find("Expires")) != response_headers.end()) {
						ttl = std::chrono::seconds(rfc2822_date(expires_itr->second.c_str()) - rfc2822_date(date_itr->second.c_str()));
					}
					else {
						ttl = std::chrono::duration_cast<std::chrono::seconds>(MAX_INTERVAL);
					}
					expires = now + ttl;
				}
				catch (const std::exception &e) {
					interval = std::min<std::chrono::nanoseconds>(interval + interval / 2, MAX_INTERVAL);
					if (elog.warn_enabled()) {
						elog.warn() << host << ": " << e.what() << std::endl;
					}
					throw;
				}
				interval = MIN_INTERVAL;
				if (elog.info_enabled()) {
					elog.info() << name << ": " << price << " (ttl=" << ttl.count() << "s)" << std::endl;
				}
			}
			return price;
		}
	protected:
		virtual double parse(const json::Value &value) const = 0;
	};

	constexpr std::chrono::steady_clock::duration PriceSource::MIN_INTERVAL, PriceSource::MAX_INTERVAL;
}

double Bitcoin::bitcoinaverage_price() {
	static class BitcoinAverage : public PriceSource {
	public:
		BitcoinAverage() : PriceSource("BitcoinAverage", "api.bitcoinaverage.com", 443, "/ticker/global/USD/last") { }
	protected:
		double parse(const json::Value &value) const override {
			return as_number(value);
		}
	} source;
	return source.get();
}

double Bitcoin::bitpay_bbb_price() {
	static class BitPayBBB : public PriceSource {
	public:
		BitPayBBB() : PriceSource("BitPay BBB", "bitpay.com", 443, "/api/rates") { }
	protected:
		double parse(const json::Value &value) const override {
			for (const json::ValuePtr &value_ptr : *as_array(value)) {
				auto &obj = as_object(*value_ptr);
				auto code_ptr = find(obj, "code");
				if (code_ptr && *as_string(*code_ptr) == "USD") {
					return as_number(json::get(obj, "rate"));
				}
			}
			throw std::ios_base::failure("missing USD rate");
		}
	} source;
	return source.get();
}

double Bitcoin::blockchain_price() {
	static class Blockchain : public PriceSource {
	public:
		Blockchain() : PriceSource("Blockchain.info", "blockchain.info", 443, "/ticker") { }
	protected:
		double parse(const json::Value &value) const override {
			return as_number(json::get(as_object(json::get(as_object(value), "USD")), "last"));
		}
	} source;
	return source.get();
}

double Bitcoin::coinbase_spot_price() {
	static class CoinbaseSpot : public PriceSource {
	public:
		CoinbaseSpot() : PriceSource("Coinbase Spot", "api.coinbase.com", 443, "/v1/prices/spot_rate") { }
	protected:
		double parse(const json::Value &value) const override {
			return std::stod(*as_string(json::get(as_object(value), "amount")));
		}
	} source;
	return source.get();
}

double Bitcoin::coindesk_bpi_price() {
	static class CoinDeskBPI : public PriceSource {
	public:
		CoinDeskBPI() : PriceSource("CoinDesk BPI", "api.coindesk.com", 443, "/v1/bpi/currentprice.json") { }
	protected:
		double parse(const json::Value &value) const override {
			return as_number(json::get(as_object(json::get(as_object(json::get(as_object(value), "bpi")), "USD")), "rate_float"));
		}
	} source;
	return source.get();
}

double Bitcoin::winkdex_price() {
	static class WinkDex : public PriceSource {
	public:
		WinkDex() : PriceSource("WinkDex", "winkdex.com", 443, "/api/v0/price") { }
	protected:
		double parse(const json::Value &value) const override {
			return as_number(json::get(as_object(value), "price")) / 100.0;
		}
	} source;
	return source.get();
}

double Bitcoin::median_price() {
	std::vector<double> prices;
	prices.reserve(6);
	do {
		prices.clear();
		auto bitcoinaverage_price = std::async(std::launch::async, &Bitcoin::bitcoinaverage_price);
		auto bitpay_bbb_price = std::async(std::launch::async, &Bitcoin::bitpay_bbb_price);
		auto blockchain_price = std::async(std::launch::async, &Bitcoin::blockchain_price);
		auto coinbase_spot_price = std::async(std::launch::async, &Bitcoin::coinbase_spot_price);
		auto coindesk_bpi_price = std::async(std::launch::async, &Bitcoin::coindesk_bpi_price);
		auto winkdex_price = std::async(std::launch::async, &Bitcoin::winkdex_price);
		try {
			prices.push_back(bitcoinaverage_price.get());
		}
		catch (...) {
		}
		try {
			prices.push_back(bitpay_bbb_price.get());
		}
		catch (...) {
		}
		try {
			prices.push_back(blockchain_price.get());
		}
		catch (...) {
		}
		try {
			prices.push_back(coinbase_spot_price.get());
		}
		catch (...) {
		}
		try {
			prices.push_back(coindesk_bpi_price.get());
		}
		catch (...) {
		}
		try {
			prices.push_back(winkdex_price.get());
		}
		catch (...) {
		}
	} while (prices.size() < 3);
	std::sort(prices.begin(), prices.end());
	auto size = prices.size();
	auto price = size & 1 ? prices[size / 2] : (prices[size / 2 - 1] + prices[size / 2 ]) / 2;
	if (elog.info_enabled()) {
		elog.info() << "median price: " << price << std::endl;
	}
	return price;
}
