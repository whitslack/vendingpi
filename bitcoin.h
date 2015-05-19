#include <deque>
#include <set>
#include <vector>

#include "common/eventfd.h"
#include "common/linux_aio.h"
#include "common/mpn.h"
#include "libsatoshi/node.h"


class Bitcoin : public satoshi::Node {

public:
	struct PrivateKey {
		mp_limb_t d[MP_NLIMBS(32)];
		bool compress;
	};

	struct PublicKey {
		mp_limb_t Q[3][MP_NLIMBS(32)];
		bool compress;
	};

	struct Address {
		enum class Type : uint8_t {
			PUBKEY_HASH = 0,
			SCRIPT_HASH = 5,
			TESTNET_PUBKEY_HASH = 111,
			TESTNET_SCRIPT_HASH = 196,
		} type;
		digest160_t hash;
	};

public:
	static constexpr uint64_t fee_per_kb = 1000, dust_threshold = 5460;

public:
	static double bitcoinaverage_price();
	static double bitpay_bbb_price();
	static double blockchain_price();
	static double coinbase_spot_price();
	static double coindesk_bpi_price();
	static double winkdex_price();
	static double median_price();

private:
	const bool testnet;
	const PrivateKey privkey;
	const PublicKey pubkey;
	const Address address;
	const satoshi::Script output_script;
	EventFD &event_fd;
	linux::AIOContext aio;
	size_t n_requested_blocks;
	std::set<digest256_t> tx_seen;
	std::deque<std::set<digest256_t>::const_iterator> tx_seen_deque;
	std::vector<satoshi::OutPoint> outpoints;
	std::vector<std::pair<uint64_t, satoshi::Script>> payments;
	uint64_t credit;

public:
	Bitcoin(const char *host, uint16_t port, bool testnet, const char privkey[], EventFD &event_fd);

public:
	void return_payment(unsigned cents);

protected:
	void dispatch(const satoshi::VersionMessage &msg) override;
	void dispatch(const satoshi::InvMessage &msg) override;
	void dispatch(const satoshi::TxMessage &msg) override;
	void dispatch(const satoshi::RejectMessage &msg) override;
	void dispatch(const satoshi::MerkleBlockMessage &msg) override;
	void dispatch(const satoshi::AlertMessage &msg) override;

private:
	void send_get_blocks();
	bool record_seen_tx(const digest256_t &tx_hash);

};
