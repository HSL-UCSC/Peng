// SPDX-FileCopyrightText: 2021 ilkecan <ilkecan@protonmail.com>
//
// SPDX-License-Identifier: MPL-2.0

use criterion::{criterion_group, criterion_main, Criterion};
use indent::indent_by;

fn bench_single_line_without_newline(c: &mut Criterion) {
    c.bench_function("single line without newline", |b| {
        b.iter(|| indent_by(2, "foo"))
    });
}

fn bench_single_line_with_newline(c: &mut Criterion) {
    c.bench_function("single line with newline", |b| {
        b.iter(|| indent_by(2, "foo\n"))
    });
}

fn bench_multiline_without_newline(c: &mut Criterion) {
    c.bench_function("multiline without newline", |b| {
        b.iter(|| {
            indent_by(
                2, "
foo
bar",
            )
        })
    });
}

fn bench_multiline_with_newline(c: &mut Criterion) {
    c.bench_function("multiline with newline", |b| {
        b.iter(|| {
            indent_by(
                2,
                "
foo
bar
",
            )
        })
    });
}

fn bench_multiline_without_newline2(c: &mut Criterion) {
    c.bench_function("multiline without newline2", |b| {
        b.iter(|| {
            indent_by(
                2,
                r##"
# aws_route53_record.dummytest_io_dummytest_io_SOA:
resource "aws_route53_record" "dummytest_io_dummytest_io_SOA" {
    fqdn    = "dummytest.io"
    id      = "Z07075823OSHKMMOKBP6G_dummy_test.io._SOA"
    name    = "dummytest.io"
    records = [
        "ns-1130.awsdns-13.org. awsdns-hostmaster.amazon.com. 1 7200 900 1209600 86400",
    ]
    ttl     = 900
    type    = "SOA"
    zone_id = "Z07075823OSHKMMOKBP6G"
}"##,
            )
        })
    });
}

fn bench_multiline_with_newline2(c: &mut Criterion) {
    c.bench_function("multiline with newline2", |b| {
        b.iter(|| {
            indent_by(
                2,
                r##"
# aws_route53_record.dummytest_io_dummytest_io_SOA:
resource "aws_route53_record" "dummytest_io_dummytest_io_SOA" {
    fqdn    = "dummytest.io"
    id      = "Z07075823OSHKMMOKBP6G_dummy_test.io._SOA"
    name    = "dummytest.io"
    records = [
        "ns-1130.awsdns-13.org. awsdns-hostmaster.amazon.com. 1 7200 900 1209600 86400",
    ]
    ttl     = 900
    type    = "SOA"
    zone_id = "Z07075823OSHKMMOKBP6G"
}
"##,
            )
        })
    });
}

criterion_group!(
    benches,
    bench_single_line_without_newline,
    bench_single_line_with_newline,
    bench_multiline_without_newline,
    bench_multiline_with_newline,
    bench_multiline_without_newline2,
    bench_multiline_with_newline2,
);
criterion_main!(benches);
