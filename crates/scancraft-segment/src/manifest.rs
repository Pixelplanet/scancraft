//! Pinned file manifests and SHA-256 verification for SAM 2.1 ONNX models and ONNX Runtime.

use crate::{Result, SegmentError};
use sha2::{Digest, Sha256};
use std::fs::File;
use std::io::{BufReader, Read};
use std::path::{Path, PathBuf};

/// Pinned Hugging Face commit for `jax-image-tools/sam21-tiny-video-onnx` (Apache-2.0).
pub const SAM21_TINY_HF_REPO: &str = "jax-image-tools/sam21-tiny-video-onnx";
pub const SAM21_TINY_HF_COMMIT: &str = "3cb090d9d0b4c897281fdcac7765c0f3a888c49f";

pub const VISION_ENCODER_SHA256: &str = "98311f41182f40c57cac2522180fff34b53931ce981412ddd8643ec27fb830e3";
pub const MASK_DECODER_SHA256: &str = "6e75023b52586314ab1dcf5a49fb9731a983ffe2b7b2a419bd42cd2b2e77effa";
pub const MEMORY_ATTENTION_SHA256: &str = "afd5045a8fe48628d4da49a82b77351a43f7554f1f101a5052ab74aae69e3737";
pub const MEMORY_ENCODER_SHA256: &str = "3a97e8ba25180b39e938ab52a0c4d085007b25f3e919f5672d4ecbcdd2ab9ee3";
pub const CONSTANTS_JSON_SHA256: &str = "292eac72e80431c796041b4d550405b1b4648e8ef25a7c265ad58859315c8b7b";

/// Pinned Windows x64 binaries from NuGet:
/// `Microsoft.ML.OnnxRuntime.DirectML` 1.24.4 + `Microsoft.AI.DirectML` 1.15.4.
pub const ORT_VERSION_PIN: &str = "1.24.4";
pub const DIRECTML_VERSION_PIN: &str = "1.15.4";

pub const ONNXRUNTIME_DLL_SHA256: &str = "e7eedec6a6f26dc39dc948276a75ef6d2bee3fff944d874ceed0bbd3b97bff40";
pub const ONNXRUNTIME_PROVIDERS_SHARED_DLL_SHA256: &str = "265c8daf29637cb259cac8be9f08f2cd45f3883f0f0e4949cbfddd5b4cbec3b6";
pub const DIRECTML_DLL_SHA256: &str = "9c9e6d822561c6c41b90e6994b3e8857cf1d66dbfb1e0c4c799c7c89b4e92da1";

/// A file path paired with its expected lowercase/hex SHA-256 digest.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PinnedFile {
    pub path: PathBuf,
    pub sha256: String,
}

impl PinnedFile {
    pub fn new(path: impl Into<PathBuf>, sha256: impl Into<String>) -> Self {
        Self { path: path.into(), sha256: sha256.into() }
    }

    pub fn verify(&self) -> Result<()> {
        verify_sha256(&self.path, &self.sha256)
    }
}

/// Paths and pinned SHA-256 digests for the four SAM 2.1 video ONNX graphs plus `constants.json`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ModelFiles {
    pub vision_encoder: PinnedFile,
    pub mask_decoder: PinnedFile,
    pub memory_attention: PinnedFile,
    pub memory_encoder: PinnedFile,
    pub constants_json: PinnedFile,
}

impl ModelFiles {
    /// Builds the pinned `jax-image-tools/sam21-tiny-video-onnx` manifest from `dir`.
    pub fn sam21_tiny_from_dir(dir: impl AsRef<Path>) -> Self {
        let d = dir.as_ref();
        Self {
            vision_encoder: PinnedFile::new(d.join("vision_encoder.onnx"), VISION_ENCODER_SHA256),
            mask_decoder: PinnedFile::new(d.join("mask_decoder.onnx"), MASK_DECODER_SHA256),
            memory_attention: PinnedFile::new(d.join("memory_attention.onnx"), MEMORY_ATTENTION_SHA256),
            memory_encoder: PinnedFile::new(d.join("memory_encoder.onnx"), MEMORY_ENCODER_SHA256),
            constants_json: PinnedFile::new(d.join("constants.json"), CONSTANTS_JSON_SHA256),
        }
    }

    /// Verifies that every model file exists and matches its pinned SHA-256 digest.
    pub fn verify_all(&self) -> Result<()> {
        for f in [&self.vision_encoder, &self.mask_decoder, &self.memory_attention, &self.memory_encoder, &self.constants_json] {
            f.verify()?;
        }
        Ok(())
    }
}

/// Paths and pinned SHA-256 digests for the dynamic ONNX Runtime + DirectML binaries.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RuntimeFiles {
    pub onnxruntime_dll: PinnedFile,
    pub providers_shared_dll: Option<PinnedFile>,
    pub directml_dll: Option<PinnedFile>,
}

impl RuntimeFiles {
    /// Builds the pinned ORT 1.24.4 + DirectML 1.15.4 manifest from `dir`.
    pub fn ort_1244_dml_from_dir(dir: impl AsRef<Path>) -> Self {
        let d = dir.as_ref();
        Self {
            onnxruntime_dll: PinnedFile::new(d.join("onnxruntime.dll"), ONNXRUNTIME_DLL_SHA256),
            providers_shared_dll: Some(PinnedFile::new(d.join("onnxruntime_providers_shared.dll"), ONNXRUNTIME_PROVIDERS_SHARED_DLL_SHA256)),
            directml_dll: Some(PinnedFile::new(d.join("DirectML.dll"), DIRECTML_DLL_SHA256)),
        }
    }

    /// Builds a custom runtime manifest with explicit pinned files.
    pub fn custom(onnxruntime_dll: PinnedFile, providers_shared_dll: Option<PinnedFile>, directml_dll: Option<PinnedFile>) -> Self {
        Self { onnxruntime_dll, providers_shared_dll, directml_dll }
    }

    /// Verifies every configured runtime binary exists and matches its pinned SHA-256 digest.
    pub fn verify_all(&self) -> Result<()> {
        self.onnxruntime_dll.verify()?;
        if let Some(p) = &self.providers_shared_dll {
            p.verify()?;
        }
        if let Some(d) = &self.directml_dll {
            d.verify()?;
        }
        Ok(())
    }
}

/// Computes the lowercase hex SHA-256 digest of `path` using buffered streaming I/O.
pub fn sha256_file(path: &Path) -> Result<String> {
    if !path.exists() {
        return Err(SegmentError::MissingFile(path.to_path_buf()));
    }
    let file = File::open(path).map_err(|source| SegmentError::Io { path: path.to_path_buf(), source })?;
    let mut reader = BufReader::with_capacity(256 * 1024, file);
    let mut hasher = Sha256::new();
    let mut buf = [0u8; 64 * 1024];
    loop {
        let n = reader.read(&mut buf).map_err(|source| SegmentError::Io { path: path.to_path_buf(), source })?;
        if n == 0 {
            break;
        }
        hasher.update(&buf[..n]);
    }
    let digest = hasher.finalize();
    let mut hex = String::with_capacity(digest.len() * 2);
    for b in digest {
        use std::fmt::Write;
        let _ = write!(&mut hex, "{b:02x}");
    }
    Ok(hex)
}

/// Verifies `path` exists and matches `expected_hex` (case-insensitive).
pub fn verify_sha256(path: &Path, expected_hex: &str) -> Result<()> {
    let expected = expected_hex.trim().to_ascii_lowercase();
    let actual = sha256_file(path)?;
    if actual != expected {
        return Err(SegmentError::HashMismatch { path: path.to_path_buf(), expected, actual });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    fn temp_file(name: &str, bytes: &[u8]) -> PathBuf {
        let path = std::env::temp_dir().join(format!("scancraft_segment_test_{}_{name}", std::process::id()));
        let mut f = File::create(&path).unwrap();
        f.write_all(bytes).unwrap();
        path
    }

    #[test]
    fn sha256_passes_and_detects_tampering() {
        let p = temp_file("hello.bin", b"scancraft-sam21");
        let digest = sha256_file(&p).unwrap();
        assert_eq!(digest.len(), 64);
        verify_sha256(&p, &digest.to_ascii_uppercase()).unwrap();

        let mut tampered = b"scancraft-sam21".to_vec();
        tampered[0] ^= 0xff;
        std::fs::write(&p, &tampered).unwrap();
        let err = verify_sha256(&p, &digest).unwrap_err();
        assert!(matches!(err, SegmentError::HashMismatch { .. }), "{err:?}");
        let _ = std::fs::remove_file(&p);
    }

    #[test]
    fn missing_file_returns_missing_error() {
        let p = std::env::temp_dir().join("scancraft_segment_definitely_missing_file_12345.onnx");
        let err = verify_sha256(&p, VISION_ENCODER_SHA256).unwrap_err();
        assert!(matches!(err, SegmentError::MissingFile(_)));
    }

    #[test]
    fn manifest_forbids_cuda_tensorrt_and_download_binaries() {
        let toml_path = Path::new(env!("CARGO_MANIFEST_DIR")).join("Cargo.toml");
        let text = std::fs::read_to_string(toml_path).unwrap();
        for forbidden in ["download-binaries", "copy-dylibs", "cuda", "tensorrt"] {
            assert!(!text.contains(forbidden), "forbidden feature/dependency `{forbidden}` found in Cargo.toml");
        }
        assert!(text.contains("load-dynamic"));
        assert!(text.contains("directml"));
        assert!(text.contains("api-24"));
    }
}
