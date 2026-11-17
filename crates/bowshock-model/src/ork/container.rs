use std::io::Read;

use super::{EmbeddedCurve, OrkError};

/// The design XML of an `.ork` and the `thrustcurves/*.rse` entries stored beside it (ZIP only; others have none).
pub(super) fn unpack(bytes: &[u8], notes: &mut Vec<String>) -> Result<(String, Vec<EmbeddedCurve>), OrkError> {
    let mut curves = Vec::new();
    let raw = if bytes.starts_with(b"PK") {
        let entries = zip_entries(bytes)?;
        curves = embedded_curves(bytes, &entries, notes);
        extract_design(bytes, &entries)?
    } else if bytes.starts_with(&[0x1f, 0x8b]) {
        let mut out = Vec::new();
        flate2::read::GzDecoder::new(bytes).read_to_end(&mut out).map_err(|e| OrkError::Archive(e.to_string()))?;
        out
    } else {
        bytes.to_vec()
    };
    let xml = String::from_utf8(raw).map_err(|e| OrkError::Xml(format!("not UTF-8: {e}")))?;
    Ok((xml, curves))
}

fn le16(b: &[u8], at: usize) -> Option<usize> {
    Some(usize::from(u16::from_le_bytes(b.get(at..at + 2)?.try_into().ok()?)))
}

fn le32(b: &[u8], at: usize) -> Option<usize> {
    Some(u32::from_le_bytes(b.get(at..at + 4)?.try_into().ok()?) as usize)
}

/// One central-directory entry of a ZIP: name, compression method, compressed size, uncompressed size, local header offset.
type ZipEntry = (String, usize, usize, usize, usize);

/// Lists the entries of a ZIP archive.
fn zip_entries(b: &[u8]) -> Result<Vec<ZipEntry>, OrkError> {
    let bad = || OrkError::Archive("truncated ZIP".to_string());
    let last = b.len().checked_sub(22).ok_or_else(bad)?;
    let eocd = (last.saturating_sub(65_535)..=last).rev().find(|&i| b[i..i + 4] == [0x50, 0x4b, 0x05, 0x06]).ok_or_else(bad)?;
    let entries = le16(b, eocd + 10).ok_or_else(bad)?;
    let mut p = le32(b, eocd + 16).ok_or_else(bad)?;
    let mut found: Vec<ZipEntry> = Vec::new();
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
    Ok(found)
}

/// Extracts the design entry of a ZIP: `rocket.ork`, else the first `*.ork` or `*.xml`.
fn extract_design(b: &[u8], found: &[ZipEntry]) -> Result<Vec<u8>, OrkError> {
    let pick = found
        .iter()
        .find(|e| e.0 == "rocket.ork")
        .or_else(|| found.iter().find(|e| e.0.ends_with(".ork")))
        .or_else(|| found.iter().find(|e| e.0.ends_with(".xml")))
        .ok_or_else(|| OrkError::Archive("no rocket.ork in the archive".to_string()))?;
    read_entry(b, pick)
}

/// The user-defined thrust curves of a design, the `thrustcurves/<digest>.rse` entries, with a note for each that cannot be
/// read (a damaged entry costs only its motor).
fn embedded_curves(b: &[u8], found: &[ZipEntry], notes: &mut Vec<String>) -> Vec<EmbeddedCurve> {
    let mut curves: Vec<EmbeddedCurve> = Vec::new();
    for entry in found {
        let name = entry.0.as_str();
        let Some(file) = name.strip_prefix("thrustcurves/") else { continue };
        let Some(digest) = file.len().checked_sub(4).filter(|&n| file.is_char_boundary(n) && file[n..].eq_ignore_ascii_case(".rse")).map(|n| &file[..n]) else {
            continue;
        };
        if digest.is_empty() || digest.contains('/') || curves.iter().any(|c| c.digest == digest) {
            continue;
        }
        match read_entry(b, entry).and_then(|raw| String::from_utf8(raw).map_err(|e| OrkError::Archive(format!("not UTF-8: {e}")))) {
            Ok(rse) => curves.push(EmbeddedCurve { digest: digest.to_string(), rse }),
            Err(e) => notes.push(format!("thrust curve {name} in the file could not be read: {e}")),
        }
    }
    curves
}

/// The contents of one ZIP entry (stored or deflated).
fn read_entry(b: &[u8], entry: &ZipEntry) -> Result<Vec<u8>, OrkError> {
    let bad = || OrkError::Archive("truncated ZIP".to_string());
    let (_, method, csize, usize_, local) = entry;
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
