//! Pure Rust decompressor for QuickLZ Level 1
//! Bit-for-bit compatible with Revopoint 3DCamera.dll qlz_decompress (v1.5.0, level 1)

pub struct QuickLzDecompressor {
    hash_tab: [usize; 4096],
}

impl Default for QuickLzDecompressor {
    fn default() -> Self {
        Self::new()
    }
}

impl QuickLzDecompressor {
    pub fn new() -> Self {
        Self {
            hash_tab: [0; 4096],
        }
    }

    /// Decompress QuickLZ level 1 data into dst buffer.
    /// Returns the number of bytes decompressed.
    pub fn decompress(&mut self, src: &[u8], dst: &mut [u8]) -> Result<usize, &'static str> {
        if src.is_empty() {
            return Err("Empty input");
        }
        let flags = src[0];
        let hdr_len = if (flags & 2) != 0 { 9 } else { 3 };
        if src.len() < hdr_len {
            return Err("Input smaller than header");
        }

        let d_len = if hdr_len == 9 {
            u32::from_le_bytes([src[5], src[6], src[7], src[8]]) as usize
        } else {
            u16::from_le_bytes([src[3], src[4]]) as usize
        };

        if dst.len() < d_len {
            return Err("Destination buffer too small");
        }

        // Bit 0 = 0 means uncompressed
        if (flags & 1) == 0 {
            if src.len() < hdr_len + d_len {
                return Err("Truncated uncompressed payload");
            }
            dst[..d_len].copy_from_slice(&src[hdr_len..hdr_len + d_len]);
            return Ok(d_len);
        }

        const BITLUT: [usize; 16] = [4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0];
        self.hash_tab.fill(0);

        let mut src_pos = hdr_len;
        let mut dst_pos = 0;
        let mut cword_val = 1u32;
        // QuickLZ 1.5.0: last_matchstart = last_byte - UNCONDITIONAL_MATCHLEN (6) - UNCOMPRESSED_END (4).
        let last_matchstart = d_len.saturating_sub(11);
        let mut last_hashed: isize = -1;

        while dst_pos < d_len {
            if cword_val == 1 {
                if src_pos + 4 > src.len() {
                    return Err("Unexpected end of compressed stream (cword)");
                }
                cword_val = u32::from_le_bytes([
                    src[src_pos],
                    src[src_pos + 1],
                    src[src_pos + 2],
                    src[src_pos + 3],
                ]);
                src_pos += 4;
            }

            let fetch = if src_pos + 4 <= src.len() {
                u32::from_le_bytes([
                    src[src_pos],
                    src[src_pos + 1],
                    src[src_pos + 2],
                    src[src_pos + 3],
                ])
            } else {
                0
            };

            if (cword_val & 1) == 1 {
                cword_val >>= 1;
                let hash = ((fetch >> 4) & 0xfff) as usize;
                let offset2 = self.hash_tab[hash];

                let matchlen = if (fetch & 0xf) != 0 {
                    src_pos += 2;
                    ((fetch & 0xf) + 2) as usize
                } else {
                    if src_pos + 2 >= src.len() {
                        return Err("Unexpected end of stream in matchlen");
                    }
                    let m = src[src_pos + 2] as usize;
                    src_pos += 3;
                    m
                };

                if dst_pos + matchlen > d_len {
                    return Err("Match exceeds destination buffer");
                }

                // Copy match bytes
                for i in 0..matchlen {
                    dst[dst_pos + i] = dst[offset2 + i];
                }
                dst_pos += matchlen;

                // Update hash table up to dst_pos - matchlen
                let target = (dst_pos - matchlen) as isize;
                while last_hashed < target {
                    last_hashed += 1;
                    let idx = last_hashed as usize;
                    if idx + 3 <= d_len {
                        let v = (dst[idx] as u32)
                            | ((dst[idx + 1] as u32) << 8)
                            | ((dst[idx + 2] as u32) << 16);
                        let h = (((v >> 12) ^ v) & 0xfff) as usize;
                        self.hash_tab[h] = idx;
                    }
                }
                last_hashed = (dst_pos - 1) as isize;
            } else {
                if dst_pos < last_matchstart {
                    let n = BITLUT[(cword_val & 0xf) as usize];
                    if src_pos + n > src.len() || dst_pos + n > d_len {
                        return Err("Literal copy out of bounds");
                    }
                    for i in 0..n {
                        dst[dst_pos + i] = src[src_pos + i];
                    }
                    cword_val >>= n;
                    dst_pos += n;
                    src_pos += n;

                    let target = (dst_pos.saturating_sub(3)) as isize;
                    while last_hashed < target {
                        last_hashed += 1;
                        let idx = last_hashed as usize;
                        if idx + 3 <= d_len {
                            let v = (dst[idx] as u32)
                                | ((dst[idx + 1] as u32) << 8)
                                | ((dst[idx + 2] as u32) << 16);
                            let h = (((v >> 12) ^ v) & 0xfff) as usize;
                            self.hash_tab[h] = idx;
                        }
                    }
                } else {
                    while dst_pos < d_len {
                        if cword_val == 1 {
                            src_pos += 4;
                            cword_val = 1 << 31;
                        }
                        if src_pos >= src.len() {
                            return Err("End of stream in end-literals");
                        }
                        dst[dst_pos] = src[src_pos];
                        dst_pos += 1;
                        src_pos += 1;
                        cword_val >>= 1;
                    }
                    let target = (d_len.saturating_sub(3)) as isize;
                    while last_hashed < target {
                        last_hashed += 1;
                        let idx = last_hashed as usize;
                        if idx + 3 <= d_len {
                            let v = (dst[idx] as u32)
                                | ((dst[idx + 1] as u32) << 8)
                                | ((dst[idx + 2] as u32) << 16);
                            let h = (((v >> 12) ^ v) & 0xfff) as usize;
                            self.hash_tab[h] = idx;
                        }
                    }
                    break;
                }
            }
        }

        Ok(d_len)
    }
}

pub fn decompress_quicklz_level1(src: &[u8], dst: &mut [u8]) -> Result<usize, &'static str> {
    let mut dec = QuickLzDecompressor::new();
    dec.decompress(src, dst)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_uncompressed_roundtrip() {
        let mut decompressor = QuickLzDecompressor::new();
        let raw_payload = b"Hello Revopoint POP 4 Depth Data!";
        let mut comp = vec![0u8; 9 + raw_payload.len()];
        comp[0] = 0x02; // Flags: uncompressed, 9-byte header
        let c_len = (9 + raw_payload.len()) as u32;
        let d_len = raw_payload.len() as u32;
        comp[1..5].copy_from_slice(&c_len.to_le_bytes());
        comp[5..9].copy_from_slice(&d_len.to_le_bytes());
        comp[9..].copy_from_slice(raw_payload);

        let mut out = vec![0u8; 128];
        let bytes = decompressor.decompress(&comp, &mut out).expect("decompresses");
        assert_eq!(&out[..bytes], raw_payload);
    }

    /// The tail of a stream switches to plain literals at `last_matchstart = d_len - 11`
    /// (QuickLZ 1.5.0); with `d_len - 14` frames whose encoder put a match into the last 14
    /// bytes failed with "End of stream in end-literals" (about a third of the 6 B/px marker
    /// frames on 2026-11-01).
    #[test]
    fn decodes_matches_close_to_the_end() {
        let mut seed = 1u32;
        for len in (300..1500).step_by(7) {
            let data: Vec<u8> = (0..len)
                .map(|i| {
                    seed = seed.wrapping_mul(1103515245).wrapping_add(12345);
                    if i % 64 < 40 { (i % 9) as u8 } else { (seed >> 24) as u8 }
                })
                .collect();
            let comp = ::quicklz::compress(&data, ::quicklz::CompressionLevel::Lvl1);
            let mut out = vec![0u8; len];
            let n = decompress_quicklz_level1(&comp, &mut out).unwrap_or_else(|e| panic!("len {len}: {e}"));
            assert_eq!(&out[..n], &data[..], "len {len}");
        }
    }

    #[test]
    fn test_pop4_real_depth_decompression() {
        let fixture_path = std::path::Path::new("tests/fixtures/pop4_depth_frame.bin");
        if fixture_path.exists() {
            let compressed = std::fs::read(fixture_path).expect("read fixture");
            let mut out = vec![0u8; 655360];
            let bytes = decompress_quicklz_level1(&compressed, &mut out).expect("decompresses successfully");
            assert_eq!(bytes, 655360, "Must decompress to 640x512x2 = 655360 bytes");
            
            // Check that we have valid non-zero depth values
            let mut valid_depths = 0;
            let mut min_val = u16::MAX;
            let mut max_val = 0u16;
            let mut sum_val = 0u64;
            for chunk in out.chunks_exact(2) {
                let val = u16::from_le_bytes([chunk[0], chunk[1]]);
                if val > 100 && val < 60000 {
                    valid_depths += 1;
                    min_val = min_val.min(val);
                    max_val = max_val.max(val);
                    sum_val += val as u64;
                }
            }
            println!("POP4 Depth stats: valid={}, min={}, max={}, avg={}", 
                valid_depths, min_val, max_val, if valid_depths > 0 { sum_val / valid_depths as u64 } else { 0 });
            assert!(valid_depths > 10000, "Should have thousands of valid depth values in real scan frame");
        }

        let live_fixture = std::path::Path::new("tests/fixtures/live_pop4_frame.bin");
        if live_fixture.exists() {
            let compressed = std::fs::read(live_fixture).expect("read live fixture");
            let mut out = vec![0u8; 655360];
            let bytes = decompress_quicklz_level1(&compressed, &mut out).expect("decompresses live fixture");
            assert_eq!(bytes, 655360);
            let mut valid_depths = 0;
            let mut min_val = u16::MAX;
            let mut max_val = 0u16;
            for chunk in out.chunks_exact(2) {
                let val = u16::from_le_bytes([chunk[0], chunk[1]]);
                if val > 100 && val < 60000 {
                    valid_depths += 1;
                    min_val = min_val.min(val);
                    max_val = max_val.max(val);
                }
            }
            println!("LIVE POP4 Frame: valid={}, min={}mm, max={}mm", 
                valid_depths, min_val as f32 * 0.1, max_val as f32 * 0.1);
        }
    }
}
