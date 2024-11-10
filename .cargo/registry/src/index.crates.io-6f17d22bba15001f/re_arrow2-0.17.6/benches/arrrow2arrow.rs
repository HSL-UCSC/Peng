//! Verify that the arrow2 <-> arrow1 conversion is efficient (zero-copy).

use criterion::{criterion_group, criterion_main, Criterion};

use re_arrow2::array::Arrow2Arrow;
use re_arrow2::array::*;
use re_arrow2::util::bench_util::*;

fn add_benchmark(c: &mut Criterion) {
    (10..=20).step_by(2).for_each(|log2_size| {
        let size = 2usize.pow(log2_size);

        let arrow2 = create_primitive_array::<f32>(size, 0.0);
        let arrow1 = arrow2.to_data();
        c.bench_function(&format!("arrrow2arrow to_arrow1 2^{log2_size} f32"), |b| {
            b.iter(|| {
                criterion::black_box(arrow2.to_data());
            })
        });
        c.bench_function(
            &format!("arrrow2arrow from_arrow1 2^{log2_size} f32"),
            |b| {
                b.iter(|| {
                    criterion::black_box(PrimitiveArray::<f32>::from_data(&arrow1));
                })
            },
        );

        let arrow2 = create_string_array::<i32>(1, size, 0.0, 0);
        let arrow1 = arrow2.to_data();
        c.bench_function(&format!("arrrow2arrow to_arrow1 2^{log2_size} utf8"), |b| {
            b.iter(|| {
                criterion::black_box(arrow2.to_data());
            })
        });
        c.bench_function(
            &format!("arrrow2arrow from_arrow1 2^{log2_size} utf8"),
            |b| {
                b.iter(|| {
                    criterion::black_box(Utf8Array::<i32>::from_data(&arrow1));
                })
            },
        );
    });
}

criterion_group!(benches, add_benchmark);
criterion_main!(benches);
