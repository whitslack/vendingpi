#include "bitcoin.h"

#include <cassert>
#include <map>
#include <system_error>
#include <thread>

#include <netinet/tcp.h>
#include <sys/uio.h>

#include "common/connect.h"
#include "common/ecp.h"
#include "common/log.h"
#include "common/mpn.h"
#include "common/optional.h"
#include "common/ripemd.h"
#include "common/serial.h"
#include "common/sha.h"
#include "libsatoshi/base58check.h"

using namespace satoshi;

extern Log elog;


namespace {
	template <typename T>
	struct deref_less {
		bool _pure operator () (const T *lhs, const T *rhs) const { return *lhs < *rhs; }
	};
}

static Socket set_keep_alive(Socket &&socket) {
	int optval = 5;
	socket.setsockopt(IPPROTO_TCP, TCP_KEEPIDLE, &optval, static_cast<socklen_t>(sizeof optval));
	socket.setsockopt(IPPROTO_TCP, TCP_KEEPCNT, &optval, static_cast<socklen_t>(sizeof optval));
	optval = 1;
	socket.setsockopt(IPPROTO_TCP, TCP_KEEPINTVL, &optval, static_cast<socklen_t>(sizeof optval));
	socket.setsockopt(SOL_SOCKET, SO_KEEPALIVE, &optval, static_cast<socklen_t>(sizeof optval));
	return std::move(socket);
}

static std::vector<uint8_t> serialize_pubkey(const PublicKey &pubkey) {
	std::vector<uint8_t> ret;
	ret.reserve(pubkey.compress ? 33 : 65);
	VectorSink vs(ret);
	vs << pubkey;
	return ret;
}

static optional<Address> scriptsig_to_address(const Script &txin_script, bool testnet) {
	auto itr = txin_script.begin(), end = txin_script.end();
	if (txin_script.valid() && itr != end) {
		auto size = itr.size();
		if (size >= 71 && size <= 73 && *itr.begin() == 0x30 && ++itr != end) {
			size = itr.size();
			const uint8_t *data;
			if ((size == 33 ? *(data = itr.data()) == 0x02 || *data == 0x03 : size == 65 && *(data = itr.data()) == 0x04) && ++itr == end) {
				Address address;
				address.type = testnet ? Address::Type::TESTNET_PUBKEY_HASH : Address::Type::PUBKEY_HASH;
				SHA256 sha;
				sha.write_fully(data, size);
				RIPEMD160 rmd;
				rmd << sha.digest();
				address.hash = rmd.digest();
				return address;
			}
		}
		// TODO recognize P2SH multisig scriptsigs
	}
	return nullopt;
}

static std::ostream & print_address_or_script(std::ostream &os, const Script &txout_script, bool testnet) {
	auto itr = txout_script.begin(), end = txout_script.end();
	if (txout_script.valid() && itr != end) {
		if (*itr == Script::OP_DUP) {
			if (++itr != end && *itr == Script::OP_HASH160 && ++itr != end && itr.size() == 20) {
				auto data = itr.data();
				if (++itr != end && *itr == Script::OP_EQUALVERIFY && ++itr != end && *itr == Script::OP_CHECKSIG && ++itr == end) {
					Address address;
					address.type = testnet ? Address::Type::TESTNET_PUBKEY_HASH : Address::Type::PUBKEY_HASH;
					std::copy(data, data + 20, address.hash.begin());
					return os << base58check_encode(&address, sizeof address);
				}
			}
		}
		else if (*itr == Script::OP_HASH160 && ++itr != end && itr.size() == 20) {
			auto data = itr.data();
			if (++itr != end && *itr == Script::OP_EQUAL && ++itr == end) {
				Address address;
				address.type = testnet ? Address::Type::TESTNET_SCRIPT_HASH : Address::Type::SCRIPT_HASH;
				std::copy(data, data + 20, address.hash.begin());
				return os << base58check_encode(&address, sizeof address);
			}
		}
	}
	return os << "[ " << txout_script << " ]";
}

static size_t der_integer_size(const uint8_t bytes[], size_t n) {
	while (n > 0 && *bytes == 0) {
		++bytes, --n;
	}
	return n == 0 || static_cast<int8_t>(*bytes) < 0 ? 1 + n : n;
}

static Sink & der_write_integer(Sink &sink, const uint8_t bytes[], size_t n) {
	while (n > 0 && *bytes == 0) {
		++bytes, --n;
	}
	sink << uint8_t(0x02); // INTEGER
	if (n == 0 || static_cast<int8_t>(*bytes) < 0) {
		assert(1 + n < 0x80);
		sink << static_cast<uint8_t>(1 + n) << uint8_t(0);
	}
	else {
		assert(n < 0x80);
		sink << static_cast<uint8_t>(n);
	}
	sink.write_fully(bytes, n);
	return sink;
}

static Source & der_read_integer(Source &source, uint8_t bytes[], size_t n) {
	uint8_t tag;
	source >> tag;
	if (tag != 0x02) { // INTEGER
		throw std::ios_base::failure("expected integer");
	}
	uint8_t size;
	source >> size;
	if (size < 0x80) {
		if (size <= n) {
			std::memset(bytes, 0, n - size);
			source.read_fully(bytes + n - size, size);
			return source;
		}
		if (size == n + 1) {
			uint8_t pad;
			source >> pad;
			if (pad == 0) {
				source.read_fully(bytes, n);
				return source;
			}
		}
	}
	throw std::ios_base::failure("integer is too large");
}

static Sink & der_write_signature(Sink &sink, const mp_limb_t (&r)[MP_NLIMBS(32)], const mp_limb_t (&s)[MP_NLIMBS(32)]) {
	uint8_t bytes_r[32], bytes_s[32];
	mpn_to_bytes(bytes_r, r, sizeof bytes_r);
	mpn_to_bytes(bytes_s, s, sizeof bytes_s);
	sink << uint8_t(0x30) /* SEQUENCE */ << static_cast<uint8_t>(2 + der_integer_size(bytes_r, 32) + 2 + der_integer_size(bytes_s, 32));
	der_write_integer(sink, bytes_r, sizeof bytes_r);
	der_write_integer(sink, bytes_s, sizeof bytes_s);
	return sink;
}

static Source & der_read_signature(Source &source, mp_limb_t (&r)[MP_NLIMBS(32)], mp_limb_t (&s)[MP_NLIMBS(32)]) {
	uint8_t tag;
	source >> tag;
	if (tag == 0x30) { // SEQUENCE
		uint8_t size;
		source >> size;
		if (size < 0x80) {
			LimitedSource ls(source, size);
			uint8_t bytes[32];
			der_read_integer(ls, bytes, sizeof bytes);
			bytes_to_mpn(r, bytes, sizeof bytes);
			der_read_integer(ls, bytes, sizeof bytes);
			bytes_to_mpn(s, bytes, sizeof bytes);
			if (ls.remaining == 0) {
				return source;
			}
		}
	}
	throw std::ios_base::failure("expected signature");
}

static std::vector<uint8_t> sign(const SHA256::digest_type &digest, const mp_limb_t (&privkey)[MP_NLIMBS(32)]) {
	mp_limb_t z[MP_NLIMBS(32)];
	bytes_to_mpn(z, digest.data(), digest.size());
	mp_limb_t r[MP_NLIMBS(32)], s[MP_NLIMBS(32)];
	ecp_sign(r, s, secp256k1_p, secp256k1_a, secp256k1_G, secp256k1_n, privkey, z);
	std::vector<uint8_t> buffer;
	VectorSink vs(buffer);
	der_write_signature(vs, r, s);
	return buffer;
}

static bool verify(const SHA256::digest_type &digest, const uint8_t signature[], size_t n_signature, const mp_limb_t (&pubkey)[3][MP_NLIMBS(32)]) {
	mp_limb_t z[MP_NLIMBS(32)];
	bytes_to_mpn(z, digest.data(), digest.size());
	MemorySource ms(signature, n_signature);
	mp_limb_t r[MP_NLIMBS(32)], s[MP_NLIMBS(32)];
	der_read_signature(ms, r, s);
	return ms.remaining == 0 && ecp_verify(secp256k1_p, secp256k1_a, secp256k1_G, secp256k1_n, pubkey, z, r, s);
}


Bitcoin::Bitcoin(const char *host, uint16_t port, bool testnet, const char privkey[], EventFD &event_fd) :
		Node(testnet ? MessageHeader::Magic::TESTNET3 : MessageHeader::Magic::MAIN, set_keep_alive(connect_with_retry(host, port))), host(host), port(port),
		testnet(testnet), privkey(decode_privkey(privkey, std::strlen(privkey))), pubkey(privkey_to_pubkey(this->privkey)), address(pubkey_to_address(pubkey, testnet)),
		output_script(address_to_script(address)), event_fd(event_fd), aio(64), n_requested_blocks(), credit() {
	try {
		FileDescriptor fd(testnet ? "tx_hashes-testnet" : "tx_hashes", O_RDONLY | O_CLOEXEC);
		struct stat st;
		fd.fstat(&st);
		fd.fadvise(0, 0, POSIX_FADV_SEQUENTIAL);
		BufferedSource<4096> bs(fd);
		for (size_t count = st.st_size / sizeof(digest256_t); count > 0; --count) {
			digest256_t tx_hash;
			bs >> tx_hash;
			auto pair = tx_seen.insert(tx_hash);
			if (pair.second) {
				tx_seen_deque.push_back(pair.first);
			}
		}
	}
	catch (const std::system_error &e) {
		if (e.code().value() != ENOENT) {
			throw;
		}
	}
}

void Bitcoin::run() {
	static constexpr std::chrono::steady_clock::duration min_reconnect_delay = std::chrono::seconds(1), max_reconnect_delay = std::chrono::seconds(10);
	for (std::chrono::steady_clock::duration reconnect_delay = min_reconnect_delay;;) {
		try {
			if (socket < 0) {
				socket = set_keep_alive(connect_with_retry(host, port));
			}
			this->do_handshake();
			reconnect_delay = min_reconnect_delay;
			this->Node::run();
		}
		catch (const std::exception &e) {
			if (elog.warn_enabled()) {
				elog.warn() << "Bitcoin client failed: " << e.what() << "; will attempt to reconnect" << std::endl;
			}
		}
		socket.close();
		std::this_thread::sleep_for(reconnect_delay);
		reconnect_delay = std::min(reconnect_delay + reconnect_delay / 2, max_reconnect_delay);
	}
}

void Bitcoin::return_payment(unsigned cents) {
	if (outpoints.empty() || payments.empty()) {
		outpoints.clear();
		payments.clear();
		credit = 0;
		return;
	}
	uint64_t amount = cents ? static_cast<uint64_t>(cents / median_price() * 1e6) : 0;
	TxMessage tx;
	tx.version = 1;
	tx.lock_time = 0;
	size_t tx_size = sizeof(uint32_t) + 1 + 1 + sizeof(int32_t);
	tx.inputs.reserve(outpoints.size());
	for (auto &outpoint : outpoints) {
		TxIn txin;
		txin.prevout = std::move(outpoint);
		txin.seq_num = UINT32_MAX;
		tx.inputs.emplace_back(std::move(txin));
		tx_size += sizeof(OutPoint) + 1 + (1 + (2 + 2 + 33 + 2 + 33 + 1) + 1 + (pubkey.compress ? 33 : 65)) + sizeof(uint32_t);
	}
	outpoints.clear();
	std::map<Script *, uint64_t, deref_less<Script>> outputs;
	for (auto itr = payments.rbegin(); amount > 0 && itr != payments.rend(); ++itr) {
		uint64_t output_amount = std::min(amount, itr->first);
		auto pair = outputs.emplace(&itr->second, output_amount);
		if (pair.second) {
			tx_size += sizeof(uint64_t) + 1 + pair.first->first->size();
		}
		else {
			pair.first->second += output_amount;
		}
		amount -= output_amount;
		credit -= output_amount;
	}
	int64_t fee = (tx_size + 1023) / 1024 * fee_per_kb;
	auto prune_output = [&tx_size, &outputs, &fee](std::map<Script *, uint64_t>::iterator outputs_itr) {
		fee -= (tx_size + 1023) / 1024 * fee_per_kb;
		tx_size -= sizeof(uint64_t) + 1 + outputs_itr->first->size();
		fee += (tx_size + 1023) / 1024 * fee_per_kb;
		fee -= outputs_itr->second;
		outputs.erase(outputs_itr);
	};
	for (auto itr = outputs.begin(); itr != outputs.end();) {
		if (itr->second < dust_threshold) {
			prune_output(itr++);
		}
		else {
			++itr;
		}
	}
	for (auto itr = payments.rbegin(); fee > 0 && itr != payments.rend(); ++itr) {
		uint64_t fee_amount = std::min<uint64_t>(fee, itr->first);
		auto outputs_itr = outputs.find(&itr->second);
		if (outputs_itr != outputs.end()) {
			if (outputs_itr->second - fee_amount < dust_threshold) {
				prune_output(outputs_itr);
			}
			else {
				outputs_itr->second -= fee_amount;
				fee -= fee_amount;
			}
		}
	}
	if (outputs.empty()) {
		payments.clear();
		credit = 0;
		return;
	}
	if (fee < 0) {
		credit -= fee, fee = 0;
	}
	tx.outputs.reserve(1 + outputs.size());
	if (credit >= dust_threshold) {
		fee -= (tx_size + 1023) / 1024 * fee_per_kb;
		tx_size += sizeof(uint64_t) + 1 + output_script.size();
		fee += (tx_size + 1023) / 1024 * fee_per_kb;
		if ((credit -= fee) >= dust_threshold) {
			tx.outputs.push_back({ htole(credit), output_script });
		}
	}
	for (auto &output : outputs) {
		tx.outputs.push_back({ htole(output.second), std::move(*output.first) });
	}
	outputs.clear();
	payments.clear();
	credit = 0;
	Tx tx1 = tx;
	Script output_script_copy = output_script;
	for (size_t txin_idx = 0; txin_idx < tx.inputs.size(); ++txin_idx) {
		using std::swap;
		swap(tx1.inputs[txin_idx].script, output_script_copy);
		auto &txin_script = tx.inputs[txin_idx].script;
		SHA256 isha, osha;
		isha << tx1 << htole(uint32_t(1)); // SIGHASH_ALL
		osha << isha.digest();
		auto signature = sign(osha.digest(), privkey.d);
		signature.push_back(1); // SIGHASH_ALL
		txin_script.push_data(signature.data(), signature.size());
		auto pubkey = serialize_pubkey(this->pubkey);
		txin_script.push_data(pubkey.data(), pubkey.size());
		swap(tx1.inputs[txin_idx].script, output_script_copy);
	}
	SHA256 isha, osha;
	isha << tx;
	osha << isha.digest();
	auto &tx_hash = osha.digest();
	this->record_seen_tx(tx_hash);
	if (elog.info_enabled()) {
		print_digest_le(elog.info() << "sending tx ", tx_hash) << std::endl;
		for (auto &txout : tx.outputs) {
			print_address_or_script(elog.info() << " - payment of " << txout.amount << " to ", txout.script, testnet) << std::endl;
		}
	}
	try {
		this->send(tx);
	}
	catch (const std::exception &e) {
		if (elog.warn_enabled()) {
			elog.warn() << "failed to send transaction: " << e.what() << "; will retry" << std::endl;
		}
	}
	last_tx_sent = std::move(tx);
}

void Bitcoin::dispatch(const VersionMessage &) {
	this->send(VerAckMessage());
	if (!last_tx_sent.outputs.empty()) {
		this->send(last_tx_sent);
	}
	if (elog.info_enabled()) {
		print_address_or_script(elog.info() << "watching for payments to ", output_script, testnet) << std::endl;
	}
	{
		FilterLoadMessage req;
		req.filter = BloomFilter(1, 1e-9);
		req.filter.insert(address.hash.data(), address.hash.size());
		req.nFlags = FilterLoadMessage::Flags::BLOOM_UPDATE_NONE;
		this->send(req);
	}
	this->send(MemPoolMessage());
	this->send_get_blocks();
}

void Bitcoin::dispatch(const InvMessage &msg) {
	GetDataMessage req;
	req.inventory.reserve(msg.inventory.size());
	for (auto &vec : msg.inventory) {
		switch (vec.type) {
			case InventoryVector::Type::MSG_BLOCK:
				req.inventory.push_back(vec);
				req.inventory.back().type = InventoryVector::Type::MSG_FILTERED_BLOCK;
				++n_requested_blocks;
				break;
			case InventoryVector::Type::MSG_TX:
				if (tx_seen.find(vec.hash) == tx_seen.end()) {
					req.inventory.push_back(vec);
				}
				break;
			default:
				break;
		}
	}
	if (!req.inventory.empty()) {
		this->send(req);
	}
}

void Bitcoin::dispatch(const TxMessage &msg) {
	SHA256 isha, osha;
	isha << msg;
	osha << isha.digest();
	auto &tx_hash = osha.digest();
	if (tx_seen.find(tx_hash) != tx_seen.end()) {
		return;
	}
	uint64_t amount = 0;
	size_t n_change_scripts = 0;
	const Script *change_script_ptr = nullptr;
	outpoints.reserve(outpoints.size() + msg.outputs.size());
	for (uint32_t txout_idx = 0; txout_idx < msg.outputs.size(); ++txout_idx) {
		auto &txout = msg.outputs[txout_idx];
		if (txout.script == output_script) {
			outpoints.push_back({ tx_hash, htole(txout_idx) });
			amount += txout.amount;
		}
		else {
			++n_change_scripts;
			change_script_ptr = &txout.script;
		}
	}
	if (amount == 0) {
		return;
	}
	credit += amount;
	if (elog.info_enabled()) {
		print_digest_le(elog.info() << "received tx ", tx_hash) << std::endl;
	}
	if (n_change_scripts == 1) {
		if (elog.info_enabled()) {
			print_address_or_script(elog.info() << " - payment of " << amount << " from ", *change_script_ptr, testnet) << std::endl;
		}
		payments.push_back({ amount, *change_script_ptr });
		goto done;
	}
	for (auto &txin : msg.inputs) {
		auto address = scriptsig_to_address(txin.script, testnet);
		if (address) {
			if (elog.info_enabled()) {
				elog.info() << " - payment of " << amount << " from " << base58check_encode(&*address, sizeof *address) << std::endl;
			}
			payments.push_back({ amount, address_to_script(*address) });
			goto done;
		}
	}
	if (elog.warn_enabled()) {
		elog.warn() << " - payment of " << amount << " from unknown sender" << std::endl;
	}
done:
	if (this->record_seen_tx(tx_hash)) {
		event_fd.write(static_cast<eventfd_t>(static_cast<double>(amount) * median_price() / 1e6));
	}
}

void Bitcoin::dispatch(const PingMessage &msg) {
	PongMessage pong;
	pong.nonce = msg.nonce;
	this->send(pong);
}

void Bitcoin::dispatch(const RejectMessage &msg) {
	if (elog.warn_enabled() && !elog.trace_enabled()) {
		elog.warn() << "received reject " << msg << std::endl;
	}
}

void Bitcoin::dispatch(const MerkleBlockMessage &msg) {
	SHA256 isha, osha;
	isha << static_cast<const BlockHeader &>(msg);
	osha << isha.digest();
	auto digest = osha.digest();
	FileDescriptor fd(testnet ? "block_hashes-testnet" : "block_hashes", O_RDWR | O_CLOEXEC);
	fd.fadvise(0, 0, POSIX_FADV_RANDOM);
	for (off_t pos = fd.lseek(0, SEEK_END), end = std::max<off_t>(pos - 100 * sizeof(digest256_t), 0); (pos -= sizeof(digest256_t)) >= end;) {
		digest256_t hash;
		fd.pread(hash.data(), sizeof hash, pos);
		if (hash == msg.parent_block_hash) {
			fd.pwrite(digest.data(), sizeof digest, pos += sizeof hash);
			fd.ftruncate(pos + sizeof digest);
			if (elog.info_enabled()) {
				print_digest_le(elog.info() << "accepted block header ", digest) << " (height=" << pos / sizeof hash << ')' << std::endl;
			}
			goto exit;
		}
	}
	if (elog.warn_enabled()) {
		print_digest_le(elog.warn() << "orphan block header ", digest) << std::endl;
	}
exit:
	if (--n_requested_blocks == 0) {
		this->send_get_blocks();
	}
}

void Bitcoin::dispatch(const AlertMessage &msg) {
	static constexpr mp_limb_t satoshi_pubkey[3][MP_NLIMBS(32)] = {
		{ MP_LIMB_C(0xB28A49E0, 0xA9110971), MP_LIMB_C(0x9BC716BD, 0xB095CDBB), MP_LIMB_C(0xEBECEDF5, 0x95DE8442), MP_LIMB_C(0x7840AAF1, 0xFC970284) },
		{ MP_LIMB_C(0xBCD68284, 0x56E7C5EC), MP_LIMB_C(0x4E9D6A69, 0x99692D52), MP_LIMB_C(0x2C093BB8, 0x9E037478), MP_LIMB_C(0xF0DB2220, 0xEAD8564F) },
		{ MP_LIMB_C(0x00000001, 0x00000000), MP_LIMB_C(0x00000000, 0x00000000), MP_LIMB_C(0x00000000, 0x00000000), MP_LIMB_C(0x00000000, 0x00000000) }
	};
	if (elog.warn_enabled()) {
		AlertPayload payload;
		VectorSource vs(msg.payload);
		vs >> payload;
		SHA256 isha, osha;
		isha.write_fully(msg.payload.data(), msg.payload.size());
		osha << isha.digest();
		elog.warn() << "ALERT! (signature " << (verify(osha.digest(), msg.signature.data(), msg.signature.size(), satoshi_pubkey) ? "valid" : "INVALID!") << ") " << payload << std::endl;
	}
}

void Bitcoin::do_handshake() {
	VersionMessage msg;
	this->init_version_message(msg);
	msg.user_agent = "/VendingPi:" VERSION "(vendingpi@mattwhitlock.name)/";
	{
		FileDescriptor fd(testnet ? "block_hashes-testnet" : "block_hashes", O_RDWR | O_CREAT | O_CLOEXEC, 0644);
		struct stat st;
		fd.fstat(&st);
		digest256_t genesis_hash, hash;
		if (testnet) {
			genesis_hash = { 0x43, 0x49, 0x7f, 0xd7, 0xf8, 0x26, 0x95, 0x71, 0x08, 0xf4, 0xa3, 0x0f, 0xd9, 0xce, 0xc3, 0xae, 0xba, 0x79, 0x97, 0x20, 0x84, 0xe9, 0x0e, 0xad, 0x01, 0xea, 0x33, 0x09, 0x00, 0x00, 0x00, 0x00 };
		}
		else {
			genesis_hash = { 0x6f, 0xe2, 0x8c, 0x0a, 0xb6, 0xf1, 0xb3, 0x72, 0xc1, 0xa6, 0xa2, 0x46, 0xae, 0x63, 0xf7, 0x4f, 0x93, 0x1e, 0x83, 0x65, 0xe1, 0x5a, 0x08, 0x9c, 0x68, 0xd6, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00 };
		}
		if (st.st_size == 0 || fd.read(hash.data(), hash.size()) != hash.size() || hash != genesis_hash) {
			fd.lseek(0);
			fd.ftruncate();
			fd.write_fully(genesis_hash.data(), st.st_size = genesis_hash.size());
		}
		msg.start_height = static_cast<int32_t>(st.st_size / sizeof(digest256_t)) - 1;
	}
	msg.relay = false;
	this->send(msg);
}

void Bitcoin::send_get_blocks() {
	GetBlocksMessage req;
	req.version = protocol_version;
	FileDescriptor fd(testnet ? "block_hashes-testnet" : "block_hashes");
	fd.fadvise(0, 0, POSIX_FADV_RANDOM);
	struct stat st;
	fd.fstat(&st);
	size_t n_blocks = static_cast<size_t>(st.st_size / sizeof(digest256_t));
	if (n_blocks > 0) {
		size_t n_hashes, n_iocbs, n_iovs;
		if (n_blocks > 11) {
			n_hashes = 10 + (n_iocbs = sizeof n_blocks * 8 - _clz(n_blocks - 10));
			n_iovs = 11;
		}
		else {
			n_iovs = n_hashes = n_blocks;
			n_iocbs = 1;
		}
		req.block_locator_hashes.resize(n_hashes);
		auto hash_itr = req.block_locator_hashes.begin();
		struct iocb iocbs[n_iocbs], *iocbps[n_iocbs];
		struct iovec iovs[n_iovs];
		auto iocb_ptr = iocbs + n_iocbs;
		auto iocbp_ptr = iocbps + n_iocbs;
		for (size_t iov_idx = n_iovs; iov_idx-- > 0;) {
			iovs[iov_idx].iov_base = (hash_itr++)->data();
			iovs[iov_idx].iov_len = sizeof(digest256_t);
		}
		off_t offset = static_cast<off_t>(n_blocks - n_iovs) * sizeof(digest256_t);
		io_prep_preadv(*--iocbp_ptr = --iocb_ptr, fd, iovs, static_cast<int>(n_iovs), offset);
		if (offset > 0) {
			for (off_t step = sizeof(digest256_t) << 1; (offset -= step) > 0; step <<= 1) {
				io_prep_pread(*--iocbp_ptr = --iocb_ptr, fd, (hash_itr++)->data(), sizeof(digest256_t), offset);
			}
			io_prep_pread(*--iocbp_ptr = --iocb_ptr, fd, (hash_itr++)->data(), sizeof(digest256_t), 0);
		}
		assert(hash_itr == req.block_locator_hashes.end());
		assert(iocb_ptr == iocbs && iocbp_ptr == iocbps);
		aio.submit(n_iocbs, iocbps);
		struct io_event events[n_iocbs];
		aio.getevents(n_iocbs, n_iocbs, events);
	}
	req.hash_stop.fill(0);
	this->send(req);
}

bool Bitcoin::record_seen_tx(const digest256_t &tx_hash) {
	auto tx_seen_pair = tx_seen.insert(tx_hash);
	if (!tx_seen_pair.second) {
		return false;
	}
	while (tx_seen_deque.size() >= 4096 / sizeof(digest256_t)) {
		tx_seen.erase(tx_seen_deque.front());
		tx_seen_deque.pop_front();
	}
	tx_seen_deque.push_back(tx_seen_pair.first);
	FileDescriptor fd(testnet ? "tx_hashes-testnet~" : "tx_hashes~", O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0600);
	BufferedSink<4096> bs(fd);
	for (auto &tx_hash_itr : tx_seen_deque) {
		bs << *tx_hash_itr;
	}
	bs.flush_fully();
	fd.fsync();
	posix::rename(testnet ? "tx_hashes-testnet~" : "tx_hashes~", testnet ? "tx_hashes-testnet" : "tx_hashes");
	FileDescriptor(".").fsync();
	return true;
}
