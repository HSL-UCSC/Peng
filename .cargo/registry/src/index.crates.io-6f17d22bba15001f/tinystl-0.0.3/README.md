# TinySTL - A small loader for STL files.
This project is heavily inspired by, and adapted from, [cry-inc's microstl library](https://github.com/cry-inc/microstl).
The goal is to provide a zero-dependency way to easily load and write STL files.
It is assumed that all binary files are little endian.

# Example
```rust
use tinystl::StlData;

let data = StlData::read_from_file("my_file.stl")?;
data.write_binary_file("my_binary_file.stl")?;
```

# Features

### Bytemuck
Derives ``Pod`` for ``Triangle``.

### Serde
Derives ``Serialize`` and ``Deserialize`` for all types.

