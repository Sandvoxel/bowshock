use std::io::Read;

use super::OrkError;

pub(super) fn unpack(bytes: &[u8]) -> Result<String, OrkError> {
    let raw = if bytes.starts_with(b"PK") {
        unzip(bytes)?
    } else if bytes.starts_with(&[0x1f, 0x8b]) {
        let mut out = Vec::new();
        flate2::read::GzDecoder::new(bytes).read_to_end(&mut out).map_err(|e| OrkError::Archive(e.to_string()))?;
        out
    } else {
        bytes.to_vec()
    };
    String::from_utf8(raw).map_err(|e| OrkError::Xml(format!("not UTF-8: {e}")))
}

fn le16(b: &[u8], at: usize) -> Option<usize> {
    Some(usize::from(u16::from_le_bytes(b.get(at..at + 2)?.try_into().ok()?)))
}

fn le32(b: &[u8], at: usize) -> Option<usize> {
    Some(u32::from_le_bytes(b.get(at..at + 4)?.try_into().ok()?) as usize)
}

/// Extracts the design entry of a ZIP: `rocket.ork`, else the first `*.ork` or `*.xml`. Stored and deflated entries.
pub(super) fn unzip(b: &[u8]) -> Result<Vec<u8>, OrkError> {
    let bad = || OrkError::Archive("truncated ZIP".to_string());
    let last = b.len().checked_sub(22).ok_or_else(bad)?;
    let eocd = (last.saturating_sub(65_535)..=last).rev().find(|&i| b[i..i + 4] == [0x50, 0x4b, 0x05, 0x06]).ok_or_else(bad)?;
    let entries = le16(b, eocd + 10).ok_or_else(bad)?;
    let mut p = le32(b, eocd + 16).ok_or_else(bad)?;
    let mut found: Vec<(String, usize, usize, usize, usize)> = Vec::new();
    for _ in 0..entries {
        if b.get(p..p + 4) != Some(&[0x50, 0x4b, 0x01, 0x02]) {
            return Err(OrkError::Archive("bad central directory".to_string()));
        }
        let method = le16(b, p + 10).ok_or_else(bad)?;
        let csize = le32(b, p + 20).ok_or_else(bad)?;
        let usize_ = le32(b, p + 24).ok_or_else(bad)?;
        let (n, m, k) = (le16(b, p + 28).ok_or_else(bad)?, le16(b, p + 30).ok_or_else(bad)?, le16(b, p + 32).ok_or_else(bad)?);
        let local = le32(b, p + 42).ok_or_else(bad)?;
        let name = String::from_utf8_lossy(b.get(p + 46..p + 46 + n).ok_or_else(bad)?).into_owned();
        found.push((name, method, csize, usize_, local));
        p += 46 + n + m + k;
    }
    let pick = found
        .iter()
        .find(|e| e.0 == "rocket.ork")
        .or_else(|| found.iter().find(|e| e.0.ends_with(".ork")))
        .or_else(|| found.iter().find(|e| e.0.ends_with(".xml")))
        .ok_or_else(|| OrkError::Archive("no rocket.ork in the archive".to_string()))?;
    let (_, method, csize, usize_, local) = pick;
    if b.get(*local..*local + 4) != Some(&[0x50, 0x4b, 0x03, 0x04]) {
        return Err(OrkError::Archive("bad local header".to_string()));
    }
    let start = local + 30 + le16(b, local + 26).ok_or_else(bad)? + le16(b, local + 28).ok_or_else(bad)?;
    let data = b.get(start..start + csize).ok_or_else(bad)?;
    match method {
        0 => Ok(data.to_vec()),
        8 => {
            let mut out = Vec::with_capacity(*usize_);
            flate2::read::DeflateDecoder::new(data).read_to_end(&mut out).map_err(|e| OrkError::Archive(e.to_string()))?;
            Ok(out)
        }
        m => Err(OrkError::Archive(format!("unsupported ZIP compression method {m}"))),
    }
}
