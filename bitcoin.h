#include <deque>
#include <set>
#include <vector>

#include "common/eventfd.h"
#include "common/linux_aio.h"
#include "common/mpn.h"
#include "libsatoshi/node.h"


class Bitcoin : public satoshi::Node {

public:
	static constexpr uint64_t relay_fee_per_kb = 5000, dust_threshold = 546 * relay_fee_per_kb / 1000;
	static constexpr uint64_t mining_fee_per_kb = 11000;

public:
	static double bitcoinaverage_price();
	static double bitpay_bbb_price();
	static double blockchain_price();
	static double coinbase_spot_price();
	static double coindesk_bpi_price();
	static double winkdex_price();
	static double median_price();

private:
	const char * const host;
	const uint16_t port;
	const bool testnet;
	const satoshi::PrivateKey privkey;
	const satoshi::PublicKey pubkey;
	const satoshi::Address address;
	const satoshi::Script output_script;
	EventFD &event_fd;
	linux::AIOContext aio;
	size_t n_requested_blocks;
	std::set<digest256_t> tx_seen;
	std::deque<std::set<digest256_t>::const_iterator> tx_seen_deque;
	std::vector<satoshi::OutPoint> outpoints;
	std::vector<std::pair<uint64_t, satoshi::Script>> payments;
	satoshi::TxMessage last_tx_sent;
	uint64_t credit;

public:
	Bitcoin(const char *host, uint16_t port, bool testnet, const char privkey[], EventFD &event_fd);

public:
	void run() _noreturn;

	void return_payment(unsigned cents);

protected:
	void dispatch(const satoshi::VersionMessage &msg) override;
	void dispatch(const satoshi::InvMessage &msg) override;
	void dispatch(const satoshi::TxMessage &msg) override;
	void dispatch(const satoshi::PingMessage &msg) override;
	void dispatch(const satoshi::RejectMessage &msg) override;
	void dispatch(const satoshi::MerkleBlockMessage &msg) override;
	void dispatch(const satoshi::AlertMessage &msg) override;

private:
	void do_handshake();
	void send_get_blocks();
	bool record_seen_tx(const digest256_t &tx_hash);

};
