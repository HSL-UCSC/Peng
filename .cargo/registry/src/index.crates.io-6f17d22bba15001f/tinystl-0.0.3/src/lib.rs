// All based on the code from microstl.h - STL file format parser
// https://github.com/cry-inc/microstl/blob/master/include/microstl.h

//! # `TinySTL` - A small loader for STL files.
//! This project is heavily inspired by, and adapted from, [cry-inc's microstl library](https://github.com/cry-inc/microstl).
//! The goal is to provide a zero-dependency way to easily load and write STL files.
//! It is assumed that all binary files are little endian.
//!
//! # Example
//! ```rust,ignore
//! use tinystl::StlData;
//! let data = StlData::read_from_file("my_file.stl")?;
//! data.write_binary_file("my_binary_file.stl")?;
//! ```
//! # Features
//! ### Bytemuck
//! Derives ``Pod`` for ``Triangle``.
//! ### Serde
//! Derives ``Serialize`` and ``Deserialize`` for all types.

use std::io::{BufRead, BufReader, Read, Write};

#[cfg(feature = "bytemuck")]
use bytemuck::{Pod, Zeroable};

#[cfg(feature = "serde")]
use serde::{Deserialize, Deserializer, Serialize, Serializer};

const F32_SIZE: usize = std::mem::size_of::<f32>();

/// In binary STL files, a triangle will always consist of 50 bytes.
/// A triangle consists of:
///
/// ```text
/// Normal: [f32; 3] - 12 bytes
/// Vertex 1: [f32; 3] - 12 bytes
/// Vertex 2: [f32; 3] - 12 bytes
/// Vertex 3: [f32; 3] - 12 bytes
/// Attirbute byte count: [u8; 2] - 2 bytes (generally [0, 0])
/// ```
/// For more information see the [Wikipedia page][https://en.wikipedia.org/wiki/STL_(file_format)#Binary_STL] on the format.
const TRIANGLE_BINARY_SIZE: usize = 50;

/// The binary STL format contains an 80 byte header.
/// It should *never* start with `b"solid"`.
/// For more information see the [Wikipedia page][https://en.wikipedia.org/wiki/STL_(file_format)#Binary_STL] on the format.
const HEADER_BINARY_SIZE: usize = 80;

/// Each facet contains a copy of all three vertex coordinates and a normal
#[repr(C)]
#[derive(Default, Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Triangle {
    pub v1: [f32; 3],
    pub v2: [f32; 3],
    pub v3: [f32; 3],
}

#[cfg(feature = "bytemuck")]
unsafe impl Pod for Triangle {}
#[cfg(feature = "bytemuck")]
unsafe impl Zeroable for Triangle {}

pub type Result<T> = std::result::Result<T, Error>;

impl From<&[u8]> for Triangle {
    fn from(buffer: &[u8]) -> Self {
        const N_FLOAT_VALUES: usize = 9; // [[f32; 3]; 3];
        let mut values = [0.0; N_FLOAT_VALUES];
        for (value, bytes) in values
            .iter_mut()
            .zip(buffer[0..(N_FLOAT_VALUES * F32_SIZE)].chunks_exact(F32_SIZE))
        {
            let mut buf = [0; F32_SIZE];
            buf.copy_from_slice(bytes);
            *value = f32::from_le_bytes(buf);
        }

        let mut facet = Triangle::default();
        facet.v1.copy_from_slice(&values[0..3]);
        facet.v2.copy_from_slice(&values[3..6]);
        facet.v3.copy_from_slice(&values[6..9]);
        facet
    }
}

impl Triangle {
    /// Set the facet normal based on the vertices.
    #[must_use]
    fn calculate_normals(&self) -> [f32; 3] {
        let u = [
            self.v2[0] - self.v1[0],
            self.v2[1] - self.v1[1],
            self.v2[2] - self.v1[2],
        ];
        let v = [
            self.v3[0] - self.v1[0],
            self.v3[1] - self.v1[1],
            self.v3[2] - self.v1[2],
        ];

        let mut normal = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ];

        let len = (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
        normal[0] /= len;
        normal[1] /= len;
        normal[2] /= len;
        normal
    }

    /// Fix normals on the facet beneath a certain epsilon;
    #[must_use]
    fn check_and_fix_normals(&self, normal: [f32; 3]) -> [f32; 3] {
        const NORMAL_LENGTH_DEVIATION_LIMIT: f32 = 0.001;

        let normal = if normal.iter().all(|i| *i == 0.0) {
            self.calculate_normals()
        } else {
            normal
        };

        let len = (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
        if (len - 1.0).abs() > NORMAL_LENGTH_DEVIATION_LIMIT {
            return self.calculate_normals();
        }
        normal
    }
}

/// Possible errors that come from loading a file
#[derive(Debug)]
pub enum Error {
    /// Generally used when a binary buffer fails to parse
    /// but could also mean that an ASCII STL is missing data.
    MissingData,
    /// Represents unexpected data when parsing an ASCII STL file.
    Unexpected(usize),
    /// Represents a failure to parse a line, usually due to a malformed vertex.
    Parse(usize),
    /// Represents a failure to convert the number of triangles to a u32 (as required by the STL specification).
    TooManyFacets(<u32 as std::convert::TryFrom<usize>>::Error),
    /// Represents a failure to convert an int.
    TryFromInt(std::num::TryFromIntError),
    /// Represents any std::io result error.
    Io(std::io::Error),
}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}

impl From<std::num::TryFromIntError> for Error {
    fn from(e: std::num::TryFromIntError) -> Self {
        Self::TryFromInt(e)
    }
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::MissingData => write!(
                f,
                "STL data ended unexpectely and is incomplete or otherwise broken."
            ),
            Error::Unexpected(line) => {
                write!(
                    f,
                    "Found an unexpected keyword or token in an ASCII STL file on line {line}."
                )
            }
            Error::Parse(line) => write!(f, "Bad Vertex or Normal on line {line}."),
            Error::TooManyFacets(e) => {
                write!(f, "{e:?}")
            }
            Error::Io(e) => write!(f, "{e:?}"),
            Error::TryFromInt(e) => write!(f, "{e:?}"),
        }
    }
}

/// Attempts to parse a line formated `f0 f1 f2` into an `[f32; 3]`.
/// Returns an `Err` if the number of elements does not match three,
/// or if it fails to parse any of the floats.
fn parse_triplet(str: &str, line: usize) -> Result<[f32; 3]> {
    let mut result = [0.0; 3];
    let mut count = 0;
    for (r, v) in result.iter_mut().zip(str.split_whitespace()) {
        if let Ok(v) = v.parse() {
            *r = v;
        } else {
            return Err(Error::Parse(line));
        }
        count += 1;
    }
    if count != 3 {
        return Err(Error::Parse(line));
    }
    Ok(result)
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
/// The encoding type that populated a ``StlData``.
pub enum Encoding {
    Binary,
    Ascii,
}

#[cfg(feature = "serde")]
fn header_serialize<S: Serializer>(
    header: &Option<[u8; 80]>,
    s: S,
) -> std::result::Result<S::Ok, S::Error> {
    s.serialize_bytes(header.unwrap_or([0; 80]).as_slice())
}

#[cfg(feature = "serde")]
fn header_deserialize<'de, D>(d: D) -> std::result::Result<Option<[u8; 80]>, D::Error>
where
    D: Deserializer<'de>,
{
    let mut res = [0; 80];
    res.copy_from_slice(<&[u8]>::deserialize(d)?);
    Ok(Some(res))
}

/// The container for all STL data.
#[derive(Default, Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct StlData {
    // data
    pub triangles: Vec<Triangle>,
    pub normals: Vec<[f32; 3]>,
    pub name: String,
    #[cfg_attr(feature = "serde", serde(serialize_with = "header_serialize"))]
    #[cfg_attr(feature = "serde", serde(deserialize_with = "header_deserialize"))]
    pub header: Option<[u8; HEADER_BINARY_SIZE]>,
    pub encoding: Option<Encoding>,
    // input settings
    /// Set to true to force recalculatian normals using vertex data.
    /// By default, recalculation is only done for zero normal vectors
    /// or normal vectors with invalid length.
    pub force_normals: bool,
    /// Set to true to disable all reculation of normal vectors.
    /// By default, recalculation is only done for zero normal vectors
    /// or normal vectors with invalid length.
    pub disable_normals: bool,
    // output settings
    /// Set to true to write zero normals in the output.
    pub nullify_normals: bool,
}

impl StlData {
    /// Creates and populates a ``StlData`` from a file path.
    pub fn read_from_file<P: AsRef<std::path::Path>>(path: P) -> Result<Self> {
        // Optimization trick for reducing generic bloat
        // See https://www.possiblerust.com/pattern/non-generic-inner-functions
        fn read_file_path(path: &std::path::Path) -> Result<StlData> {
            let mut res = StlData::default();
            res.set_from_file(path)?;
            Ok(res)
        }
        read_file_path(path.as_ref())
    }

    /// Creates and populates a ``StlData`` from a buffer.
    pub fn read_buffer(reader: impl BufRead) -> Result<Self> {
        let mut res = Self::default();
        res.set_from_buffer(reader)?;
        Ok(res)
    }

    /// Sets contents of a ``StlData`` from a file path.
    /// If the method returns an `Err` then the state of the reader
    /// will be empty.
    pub fn set_from_file<P: AsRef<std::path::Path>>(&mut self, path: P) -> Result<()> {
        self.set_from_file_path(path.as_ref())
    }

    // Optimization trick for reducing generic bloat
    // See https://www.possiblerust.com/pattern/non-generic-inner-functions
    fn set_from_file_path(&mut self, path: &std::path::Path) -> Result<()> {
        let file = std::fs::File::open(path)?;
        let reader = BufReader::new(file);
        self.set_from_buffer(reader)?;
        Ok(())
    }

    /// Sets the contents of a ``StlData`` from a buffer.
    /// If the method returns an `Err` then the state of the reader
    /// will be empty.
    pub fn set_from_buffer(&mut self, mut reader: impl BufRead) -> Result<()> {
        self.clear();
        let buffer = reader.fill_buf()?;
        if buffer.len() < 5 {
            return Err(Error::MissingData);
        }
        if buffer[0..5] == *b"solid" {
            let set = self.read_ascii_buffer(reader);
            if set.is_err() {
                self.clear();
                return set;
            }
            self.encoding = Some(Encoding::Ascii);
        } else {
            let set = self.read_binary_buffer(reader);
            if set.is_err() {
                self.clear();
                return set;
            }
            self.encoding = Some(Encoding::Binary);
        }
        Ok(())
    }

    /// Reset all data within the reader.
    pub fn clear(&mut self) {
        self.triangles.clear();
        self.name.clear();
        self.header = None;
        self.encoding = None;
    }

    /// For internal use.
    /// Sets the contents ``StlData`` from an ASCII buffer.
    fn read_ascii_buffer(&mut self, reader: impl BufRead) -> Result<()> {
        // State machine variables
        let mut active_solid = false;
        let mut active_facet = false;
        let mut active_loop = false;
        let mut solid_count = 0;
        let mut loop_count = 0;
        let mut vertex_count = 0;

        let mut n = [0.0; 3];
        let mut v = [0.0; 9];

        // Line reader with loop to work the state machine
        for (line_number, line) in reader.lines().enumerate() {
            let line_number = line_number + 1; // offset from 0 start
            let line = line?;
            if line.trim().starts_with("solid") {
                if active_solid || solid_count != 0 {
                    return Err(Error::Unexpected(line_number));
                }
                active_solid = true;
                if line.trim().len() > 5 {
                    self.name = (line["solid".len()..].trim()).to_string();
                }
            }
            if line.trim().starts_with("endsolid") {
                if !active_solid || active_facet || active_loop {
                    return Err(Error::Unexpected(line_number));
                }
                active_solid = false;
                solid_count += 1;
            }
            if line.trim().starts_with("facet normal") {
                if !active_solid || active_loop || active_facet {
                    return Err(Error::Unexpected(line_number));
                }
                active_facet = true;
                n = parse_triplet(line.trim()["facet normal".len()..].trim(), line_number)?;
            }
            if line.trim().starts_with("endfacet") {
                if !active_solid || active_loop || !active_facet || loop_count != 1 {
                    return Err(Error::Unexpected(line_number));
                }
                active_facet = false;
                loop_count = 0;
                let mut facet = Triangle::default();
                facet.v1.copy_from_slice(&v[0..3]);
                facet.v2.copy_from_slice(&v[3..6]);
                facet.v3.copy_from_slice(&v[6..9]);

                let normal = if self.force_normals && !self.disable_normals {
                    facet.calculate_normals()
                } else if !self.disable_normals {
                    facet.check_and_fix_normals(n)
                } else {
                    n
                };

                self.normals.push(normal);
                self.triangles.push(facet);
            }
            if line.trim().starts_with("outer loop") {
                if !active_solid || !active_facet || active_loop {
                    return Err(Error::Unexpected(line_number));
                }
                active_loop = true;
            }
            if line.trim().starts_with("endloop") {
                if !active_solid || !active_facet || !active_loop || vertex_count != 3 {
                    return Err(Error::Unexpected(line_number));
                }
                active_loop = false;
                loop_count += 1;
                vertex_count = 0;
            }
            if line.trim().starts_with("vertex") {
                if !active_solid || !active_facet || !active_loop || vertex_count >= 3 {
                    return Err(Error::Unexpected(line_number));
                }
                let triplet = parse_triplet(line.trim()["vertex".len()..].trim(), line_number)?;
                v[vertex_count * 3] = triplet[0];
                v[vertex_count * 3 + 1] = triplet[1];
                v[vertex_count * 3 + 2] = triplet[2];

                vertex_count += 1;
            }
        }

        if active_solid || active_facet || active_loop || solid_count == 0 {
            return Err(Error::MissingData);
        }

        Ok(())
    }

    /// For internal use.
    /// Sets the contents ``StlData`` from a binary buffer.
    fn read_binary_buffer(&mut self, mut reader: impl BufRead) -> Result<()> {
        let mut buffer = vec![0; HEADER_BINARY_SIZE];

        let mut header_reader = (&mut reader).take(u64::try_from(HEADER_BINARY_SIZE)?);
        let header_bytes_read = header_reader.read_to_end(&mut buffer)?;
        if header_bytes_read != HEADER_BINARY_SIZE {
            return Err(Error::MissingData);
        }

        let mut header_buffer = [0; HEADER_BINARY_SIZE];
        header_buffer.copy_from_slice(&buffer[0..HEADER_BINARY_SIZE]);
        self.header = Some(header_buffer);
        buffer.clear();

        let mut facet_count_reader = (&mut reader).take(4);
        let facet_count_bytes_read = facet_count_reader.read_to_end(&mut buffer)?;
        if facet_count_bytes_read != 4 {
            return Err(Error::MissingData);
        }
        let mut facet_count_buf = [0; 4];
        facet_count_buf.copy_from_slice(&buffer[0..4]);

        let facet_count = u32::from_le_bytes(facet_count_buf);
        if facet_count == 0 {
            return Err(Error::MissingData);
        }
        buffer.clear();

        for _ in 0..facet_count {
            let mut facet_reader = (&mut reader).take(u64::try_from(TRIANGLE_BINARY_SIZE)?);
            let facet_buffer_bytes_read = facet_reader.read_to_end(&mut buffer)?;
            if facet_buffer_bytes_read != TRIANGLE_BINARY_SIZE {
                return Err(Error::MissingData);
            }
            let (normal_buffer, vertex_buffer) = buffer.split_at(F32_SIZE * 3);
            let facet = Triangle::from(vertex_buffer);
            let mut n = [0.0; 3];
            for (n, chunk) in n.iter_mut().zip(normal_buffer.chunks_exact(F32_SIZE)) {
                let mut bytes = [0; 4];
                bytes.copy_from_slice(chunk);
                *n = f32::from_le_bytes(bytes);
            }
            let normal = if self.force_normals && !self.disable_normals {
                facet.calculate_normals()
            } else if !self.disable_normals {
                facet.check_and_fix_normals(n)
            } else {
                n
            };
            self.normals.push(normal);
            self.triangles.push(facet);
            buffer.clear();
        }
        Ok(())
    }

    /// Write the contents of a ``StlData`` to a buffer using the binary specification.
    pub fn write_binary_buffer(&self, mut writer: impl Write) -> Result<()> {
        writer.write_all(self.header.unwrap_or([0; HEADER_BINARY_SIZE]).as_slice())?;
        let n_triangles = u32::try_from(self.triangles.len())?;
        writer.write_all(n_triangles.to_le_bytes().as_slice())?;
        let null_bytes = [0; 12];

        for (&Triangle { v1, v2, v3 }, &normal) in self.triangles.iter().zip(self.normals.iter()) {
            if self.nullify_normals {
                writer.write_all(&null_bytes)?;
            } else {
                for n in normal {
                    writer.write_all(n.to_le_bytes().as_slice())?;
                }
            }

            for vertex in [v1, v2, v3] {
                for v in vertex {
                    writer.write_all(v.to_le_bytes().as_slice())?;
                }
            }

            writer.write_all(&[0; 2])?;
        }

        Ok(())
    }

    /// Write the contents of a ``StlData`` to a buffer using the ASCII specification.
    pub fn write_ascii_buffer(&self, mut writer: impl Write) -> Result<()> {
        writeln!(writer, "solid {}", self.name)?;
        for (&Triangle { v1, v2, v3 }, &normal) in self.triangles.iter().zip(self.normals.iter()) {
            if self.nullify_normals {
                writeln!(writer, "  facet normal 0 0 0")?;
            } else {
                let [n0, n1, n2] = normal;
                writeln!(writer, "  facet normal {n0} {n1} {n2}")?;
            };
            writeln!(writer, "    outer loop")?;
            for v in [v1, v2, v3] {
                let [v0, v1, v2] = v;
                writeln!(writer, "      vertex {v0} {v1} {v2}")?;
            }
            writeln!(writer, "    endloop")?;
            writeln!(writer, "  endfacet")?;
        }
        writeln!(writer, "endsolid")?;
        Ok(())
    }

    /// Writes the data from a ``StlData`` to a given path conforming to the ASCII STL specification.
    ///
    /// **If the file exists at the given path, it will be overwritten**.
    pub fn write_ascii_file<P: AsRef<std::path::Path>>(&self, path: P) -> Result<()> {
        self.write_ascii_file_path(path.as_ref())
    }

    // Optimization trick for reducing generic bloat
    // See https://www.possiblerust.com/pattern/non-generic-inner-functions
    fn write_ascii_file_path(&self, path: &std::path::Path) -> Result<()> {
        let file = std::fs::OpenOptions::new()
            .write(true)
            .create(true)
            .open(path)?;
        let writer = std::io::BufWriter::new(file);
        self.write_ascii_buffer(writer)?;
        Ok(())
    }

    /// Writes the data from a ``StlData`` to a given path conforming to the binary STL specification.
    ///
    /// **If the file exists at the given path, it will be overwritten**.
    pub fn write_binary_file<P: AsRef<std::path::Path>>(&self, path: P) -> Result<()> {
        self.write_binary_file_path(path.as_ref())
    }

    // Optimization trick for reducing generic bloat
    // See https://www.possiblerust.com/pattern/non-generic-inner-functions
    fn write_binary_file_path(&self, path: &std::path::Path) -> Result<()> {
        let file = std::fs::OpenOptions::new()
            .write(true)
            .create(true)
            .open(path)?;
        let writer = std::io::BufWriter::new(file);
        self.write_binary_buffer(writer)?;
        Ok(())
    }
}
