// All based on the tests from microstl
// https://github.com/cry-inc/microstl/blob/master/tests/tests.cpp

use std::fs::File;
use std::path::PathBuf;
use tinystl::StlData;

fn read_file_unchecked<P: AsRef<std::path::Path>>(test_path: P) -> StlData {
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    path.push(test_path);
    let res = StlData::read_from_file(&path);
    assert!(res.is_ok());
    res.unwrap()
}

#[test]
fn simple_writer() {
    let res = read_file_unchecked("testdata/box_meshlab_ascii.stl");
    let tmpfile: File = tempfile::tempfile().unwrap();
    assert!(res.write_binary_buffer(tmpfile).is_ok());
    let tmpfile: File = tempfile::tempfile().unwrap();
    assert!(res.write_ascii_buffer(tmpfile).is_ok());
}

#[test]
fn nulled_normals() {
    let mut res = read_file_unchecked("testdata/box_meshlab_ascii.stl");
    res.nullify_normals = true;
    let tmp = tempfile::tempfile().unwrap();
    assert!(res.write_binary_buffer(tmp).is_ok());
    let tmp = tempfile::tempfile().unwrap();
    assert!(res.write_ascii_buffer(tmp).is_ok());
}

#[test]
fn default_impl() {
    let data = StlData {
        triangles: vec![tinystl::Triangle {
            v1: [0.0; 3],
            v2: [0.0, 0.0, 1.0],
            v3: [0.0, 1.0, 1.0],
        }],
        normals: vec![[-1.0, 0.0, 0.0]],
        ..Default::default()
    };
    let tmp = tempfile::tempfile().unwrap();
    assert!(data.write_ascii_buffer(tmp).is_ok());
}

#[test]
fn write_buffer() {
    let res = read_file_unchecked("testdata/box_meshlab_ascii.stl");
    let mut buffer = Vec::new();
    let write_ok = res.write_binary_buffer(&mut buffer);
    assert!(write_ok.is_ok());
    assert_eq!(buffer.len(), 80 + 4 + 12 * (12 * 4 + 2));
}

#[test]
fn invalid_path_write() {
    let res = read_file_unchecked("testdata/box_meshlab_ascii.stl");
    let write_err = res.write_binary_file("folder/does/not/exist/out.stl");
    assert!(write_err.is_err());
    if let Err(tinystl::Error::Io(e)) = write_err {
        assert_eq!(e.kind(), std::io::ErrorKind::NotFound);
    } else {
        panic!("Wrong error type");
    }
}

#[test]
fn full_cycle() {
    let res = read_file_unchecked("testdata/box_meshlab_ascii.stl");
    let binary_file = tempfile::tempfile().unwrap();
    let ascii_file = tempfile::Builder::new()
        .prefix("ascii")
        .suffix(".stl")
        .tempfile()
        .unwrap();
    let ascii_path = ascii_file.path().as_os_str().to_str().unwrap().to_string();

    assert!(res.write_binary_buffer(binary_file).is_ok());
    assert!(res.write_ascii_file(&ascii_path).is_ok());

    let data = StlData::read_from_file(&ascii_path);
    assert!(data.is_ok());
    let data = data.unwrap();
    assert_eq!(data.triangles.len(), 12);
    assert_eq!(data.encoding, Some(tinystl::Encoding::Ascii));
    assert_eq!(res.triangles, data.triangles);
}
