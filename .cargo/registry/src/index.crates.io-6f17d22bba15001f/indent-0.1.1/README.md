<!--
SPDX-FileCopyrightText: 2021 ilkecan <ilkecan@protonmail.com>

SPDX-License-Identifier: MPL-2.0
-->

# indent

[![Crates.io](https://img.shields.io/crates/v/indent?style=for-the-badge)](https://crates.io/crates/indent/)
[![Crates.io](https://img.shields.io/crates/l/indent?style=for-the-badge)](https://www.mozilla.org/en-US/MPL/2.0/)
[![docs.rs](https://img.shields.io/docsrs/indent?style=for-the-badge)](https://docs.rs/indent/)
[![Libraries.io dependency status for latest release](https://img.shields.io/librariesio/release/cargo/indent?style=for-the-badge)](https://libraries.io/cargo/indent)
[![Lines of code](https://img.shields.io/tokei/lines/git.sr.ht/~ilkecan/indent-rs?style=for-the-badge)](https://github.com/XAMPPRocky/tokei)
[![REUSE Compliance](https://img.shields.io/reuse/compliance/git.sr.ht/~ilkecan/indent-rs?style=for-the-badge)](https://api.reuse.software/info/git.sr.ht/~ilkecan/indent-rs)

This crate provides 4 functions useful for inserting a multiline string into an
already indented context in another string:

- `indent_by`: Indents every line that is not empty by the given number of spaces,
  starting from the second line.
- `indent_with`: Indents every line that is not empty with the given prefix,
  starting from the second line.
- `indent_all_by`: Indents every line that is not empty by the given number of spaces.
- `indent_all_with`: Indents every line that is not empty with the given prefix.

## Examples
```rust
fn main() {
    let string = "line one

line two
";
    assert_eq!("line one\n\n  line two\n", indent::indent_by(2, string));
}
```

## License
[Mozilla Public License 2.0](LICENSE.txt)

## Contribution
Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you shall be licensed under the Mozilla Public
License 2.0, without any additional terms or conditions.
