# re_rav1d changelog
Tracks differences since [rav1d 1.0.0](https://crates.io/crates/rav1d/1.0.0)

## re_rav1d 0.1.3
* Remove *.c files from the release again & move build.rs parts that are only relevant for the cli to the cli project [#4](https://github.com/rerun-io/re_rav1d/pull/4)
* Fix assembly on Windows [#5](https://github.com/rerun-io/re_rav1d/pull/5)

## re_rav1d 0.1.2
* Linux/x64: re-enable assembly routines and prevent illegal relocations [#3](https://github.com/rerun-io/re_rav1d/pull/3)

## re_rav1d 0.1.1
* Include `tools/compat/getopt.c` in the release

## re_rav1d 0.1.0
* Add `dav1d-rs` as a Rust API ([upstream PR](https://github.com/memorysafety/rav1d/pull/1364))
* Fix crash on erroneous videos ([upstream PR](https://github.com/memorysafety/rav1d/pull/1362))
* Disable `asm` feature on Linux
