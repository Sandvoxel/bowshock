//! Error type of the facade.

use std::fmt;

/// Failure of an analysis: bad input, unsupported combination, grid problem or GPU failure.
#[derive(Debug)]
pub enum Error {
    /// An input value is out of range or inconsistent.
    InvalidInput(String),
    /// The request is valid but not supported.
    Unsupported(String),
    /// The grid could not be built or is too poor.
    Grid(String),
    /// The GPU backend failed.
    Gpu(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::InvalidInput(m) | Error::Unsupported(m) | Error::Grid(m) | Error::Gpu(m) => f.write_str(m),
        }
    }
}

impl std::error::Error for Error {}
