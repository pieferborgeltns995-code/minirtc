#ifndef MINIRTC_AV1_CROP_METADATA_H_
#define MINIRTC_AV1_CROP_METADATA_H_

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace minirtc {
namespace av1_crop {

constexpr uint8_t kObuTypeSequenceHeader = 1;
constexpr uint8_t kObuTypeTemporalDelimiter = 2;
constexpr uint8_t kObuTypeMetadata = 5;

// Standard AV1 OBU_METADATA (type 5) with METADATA_TYPE_ITUT_T35 (0x04),
// carrying a 12-byte CrossDesk visible-dimension tag:
//   [0x2A, 0x0C, 0x04, 0xB5, 'C', 'D', 'C', 'R', 'P', '1', w_hi, w_lo, h_hi, h_lo]
// Compliant AV1 decoders (dav1d, VideoToolbox, libaom) ignore unknown T.35
// metadata, while Crop-aware decoders strip it and crop back to (width, height).
inline constexpr uint8_t kT35CountryCode = 0xB5;
inline constexpr uint8_t kMagic[] = {'C', 'D', 'C', 'R', 'P', '1'};

inline void AppendCropMetadataObu(std::vector<uint8_t>& out, size_t insert_at,
                                  uint16_t width, uint16_t height) {
  const uint8_t obu[14] = {
      0x2A,  // forbidden=0, type=5 (OBU_METADATA), ext=0, has_size=1
      0x0C,  // leb128 payload size = 12
      0x04,  // leb128 metadata_type = METADATA_TYPE_ITUT_T35
      kT35CountryCode,
      kMagic[0],
      kMagic[1],
      kMagic[2],
      kMagic[3],
      kMagic[4],
      kMagic[5],
      static_cast<uint8_t>((width >> 8) & 0xFF),
      static_cast<uint8_t>(width & 0xFF),
      static_cast<uint8_t>((height >> 8) & 0xFF),
      static_cast<uint8_t>(height & 0xFF),
  };
  const size_t offset = insert_at <= out.size() ? insert_at : out.size();
  out.insert(out.begin() + static_cast<ptrdiff_t>(offset), obu,
             obu + sizeof(obu));
}

inline bool ReadLeb128(const uint8_t* data, size_t size, size_t& pos,
                       uint64_t& value) {
  value = 0;
  for (int i = 0; i < 8; ++i) {
    if (pos >= size) return false;
    const uint8_t byte = data[pos++];
    value |= static_cast<uint64_t>(byte & 0x7F) << (i * 7);
    if ((byte & 0x80) == 0) return true;
  }
  return false;
}

inline void ExtractAndStripCropMetadata(const uint8_t* data, size_t size,
                                        uint32_t& out_width,
                                        uint32_t& out_height,
                                        bool& out_has_seq_header,
                                        std::vector<uint8_t>& stripped) {
  out_width = 0;
  out_height = 0;
  out_has_seq_header = false;
  stripped.clear();
  if (!data || size == 0) return;

  size_t pos = 0;
  bool found_crop = false;
  size_t copied_until = 0;

  while (pos < size) {
    const size_t obu_start = pos;
    const uint8_t header = data[pos++];
    if ((header & 0x80) != 0) {
      stripped.clear();
      return;
    }
    const uint8_t obu_type = static_cast<uint8_t>((header >> 3) & 0x0F);
    const bool has_ext = (header & 0x04) != 0;
    const bool has_size = (header & 0x02) != 0;
    if (has_ext) {
      if (pos >= size) {
        stripped.clear();
        return;
      }
      ++pos;
    }
    size_t payload_offset = pos;
    size_t payload_size = size - pos;
    if (has_size) {
      uint64_t leb = 0;
      if (!ReadLeb128(data, size, pos, leb) || leb > size - pos) {
        stripped.clear();
        return;
      }
      payload_offset = pos;
      payload_size = static_cast<size_t>(leb);
      pos += payload_size;
    } else {
      pos = size;
    }

    if (obu_type == kObuTypeSequenceHeader) {
      out_has_seq_header = true;
    } else if (obu_type == kObuTypeMetadata && !has_ext &&
               payload_size == 12 &&
               data[payload_offset] == 0x04 &&
               data[payload_offset + 1] == kT35CountryCode &&
               std::memcmp(data + payload_offset + 2, kMagic,
                           sizeof(kMagic)) == 0) {
      const uint32_t w =
          (static_cast<uint32_t>(data[payload_offset + 8]) << 8) |
          static_cast<uint32_t>(data[payload_offset + 9]);
      const uint32_t h =
          (static_cast<uint32_t>(data[payload_offset + 10]) << 8) |
          static_cast<uint32_t>(data[payload_offset + 11]);
      if (w >= 2 && h >= 2 && (w & 1U) == 0 && (h & 1U) == 0) {
        out_width = w;
        out_height = h;
        found_crop = true;
        stripped.insert(stripped.end(), data + copied_until, data + obu_start);
        copied_until = pos;
      }
    }
  }

  if (found_crop && copied_until < size) {
    stripped.insert(stripped.end(), data + copied_until, data + size);
  }
}

}  // namespace av1_crop
}  // namespace minirtc

#endif  // MINIRTC_AV1_CROP_METADATA_H_
