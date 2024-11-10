# `re_rav1d` Release Checklist

* [ ] Update `CHANGELOG.md`
* [ ] Bump version numbers
* [ ] `git commit -m 'Release 0.x.0 - summary'`
* [ ] `cargo publish --quiet -p re_rav1d`
* [ ] `git tag -a 0.x.0 -m 'Release 0.x.0 - summary'`
* [ ] `git pull --tags && git tag -d latest ; git tag -a latest -m 'Latest release' && git push --tags origin latest --force && git push origin main ; git push --tags ; git push`
* [ ] Do a GitHub release: https://github.com/rerun-io/re_rav1d/releases/new
* [ ] Wait for documentation to build: https://docs.rs/releases/queue
* [ ] Post on Twitter
