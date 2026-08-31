#include "replay_sha256.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace open_lmm::test::replay {
namespace {

constexpr std::array<std::uint32_t, 64> kRoundConstants = {
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU,
    0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U, 0xd807aa98U, 0x12835b01U,
    0x243185beU, 0x550c7dc3U, 0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U,
    0xc19bf174U, 0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
    0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU, 0x983e5152U,
    0xa831c66dU, 0xb00327c8U, 0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U,
    0x06ca6351U, 0x14292967U, 0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU,
    0x53380d13U, 0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
    0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U, 0xd192e819U,
    0xd6990624U, 0xf40e3585U, 0x106aa070U, 0x19a4c116U, 0x1e376c08U,
    0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU,
    0x682e6ff3U, 0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
    0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};

std::uint32_t RotateRight(std::uint32_t value, unsigned bits) {
  return (value >> bits) | (value << (32U - bits));
}

class Sha256State {
 public:
  void Update(const char* data, std::size_t size) {
    total_size_ += size;
    while (size > 0) {
      const std::size_t count =
          std::min(size, block_.size() - block_size_);
      for (std::size_t index = 0; index < count; ++index) {
        block_[block_size_ + index] =
            static_cast<std::uint8_t>(data[index]);
      }
      block_size_ += count;
      data += count;
      size -= count;
      if (block_size_ == block_.size()) {
        Transform(block_);
        block_size_ = 0;
      }
    }
  }

  std::string Finish() {
    const std::uint64_t bit_size = total_size_ * 8U;
    block_[block_size_++] = 0x80U;
    if (block_size_ > 56) {
      while (block_size_ < block_.size()) block_[block_size_++] = 0;
      Transform(block_);
      block_size_ = 0;
    }
    while (block_size_ < 56) block_[block_size_++] = 0;
    for (unsigned index = 0; index < 8; ++index) {
      block_[63U - index] =
          static_cast<std::uint8_t>(bit_size >> (index * 8U));
    }
    Transform(block_);

    std::ostringstream output;
    output << std::hex << std::setfill('0');
    for (std::uint32_t word : hash_) output << std::setw(8) << word;
    return output.str();
  }

 private:
  void Transform(const std::array<std::uint8_t, 64>& block) {
    std::array<std::uint32_t, 64> words{};
    for (std::size_t index = 0; index < 16; ++index) {
      words[index] =
          (static_cast<std::uint32_t>(block[index * 4]) << 24U) |
          (static_cast<std::uint32_t>(block[index * 4 + 1]) << 16U) |
          (static_cast<std::uint32_t>(block[index * 4 + 2]) << 8U) |
          static_cast<std::uint32_t>(block[index * 4 + 3]);
    }
    for (std::size_t index = 16; index < words.size(); ++index) {
      const std::uint32_t s0 = RotateRight(words[index - 15], 7) ^
                               RotateRight(words[index - 15], 18) ^
                               (words[index - 15] >> 3U);
      const std::uint32_t s1 = RotateRight(words[index - 2], 17) ^
                               RotateRight(words[index - 2], 19) ^
                               (words[index - 2] >> 10U);
      words[index] = words[index - 16] + s0 + words[index - 7] + s1;
    }

    std::uint32_t a = hash_[0];
    std::uint32_t b = hash_[1];
    std::uint32_t c = hash_[2];
    std::uint32_t d = hash_[3];
    std::uint32_t e = hash_[4];
    std::uint32_t f = hash_[5];
    std::uint32_t g = hash_[6];
    std::uint32_t h = hash_[7];
    for (std::size_t index = 0; index < words.size(); ++index) {
      const std::uint32_t sum1 = RotateRight(e, 6) ^ RotateRight(e, 11) ^
                                 RotateRight(e, 25);
      const std::uint32_t choose = (e & f) ^ (~e & g);
      const std::uint32_t temporary1 =
          h + sum1 + choose + kRoundConstants[index] + words[index];
      const std::uint32_t sum0 = RotateRight(a, 2) ^ RotateRight(a, 13) ^
                                 RotateRight(a, 22);
      const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t temporary2 = sum0 + majority;
      h = g;
      g = f;
      f = e;
      e = d + temporary1;
      d = c;
      c = b;
      b = a;
      a = temporary1 + temporary2;
    }
    hash_[0] += a;
    hash_[1] += b;
    hash_[2] += c;
    hash_[3] += d;
    hash_[4] += e;
    hash_[5] += f;
    hash_[6] += g;
    hash_[7] += h;
  }

  std::array<std::uint32_t, 8> hash_ = {
      0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
      0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U};
  std::array<std::uint8_t, 64> block_{};
  std::size_t block_size_ = 0;
  std::uint64_t total_size_ = 0;
};

}  // namespace

std::string Sha256(std::string_view bytes) {
  Sha256State state;
  state.Update(bytes.data(), bytes.size());
  return state.Finish();
}

std::string Sha256File(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("failed to open file for SHA-256: " +
                             path.string());
  }
  Sha256State state;
  std::array<char, 64 * 1024> buffer{};
  while (input.read(buffer.data(), buffer.size()) || input.gcount() > 0) {
    state.Update(buffer.data(), static_cast<std::size_t>(input.gcount()));
  }
  if (input.bad()) {
    throw std::runtime_error("failed to read file for SHA-256: " +
                             path.string());
  }
  return state.Finish();
}

}  // namespace open_lmm::test::replay
