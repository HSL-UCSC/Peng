// Copyright © 2018–2024 Trevor Spiteri

// This library is free software: you can redistribute it and/or
// modify it under the terms of either
//
//   * the Apache License, Version 2.0 or
//   * the MIT License
//
// at your option.
//
// You should have recieved copies of the Apache License and the MIT
// License along with the library. If not, see
// <https://www.apache.org/licenses/LICENSE-2.0> and
// <https://opensource.org/licenses/MIT>.

// the value must be positive for all public functions

pub mod frac_part {
    use core::num::{NonZeroU128, NonZeroU16, NonZeroU32, NonZeroU64, NonZeroU8};

    // MAX / 1000 (0) < val <= MAX (255)
    // -3 <= log <= -1
    #[inline]
    pub const fn u8(val: NonZeroU8) -> i32 {
        let val = val.get();
        if val > 25 {
            -1
        } else if val > 2 {
            -2
        } else {
            -3
        }
    }

    // MAX / 100_000 (0) < val <= MAX (65_535)
    // -5 <= log <= -1
    #[inline]
    pub const fn u16(val: NonZeroU16) -> i32 {
        let val = val.get();
        if val > 6553 {
            -1
        } else if val > 655 {
            -2
        } else if val > 65 {
            -3
        } else if val > 6 {
            -4
        } else {
            -5
        }
    }

    // 0 < val <= MAX
    // -10 <= log <= -1
    pub const fn u32(val: NonZeroU32) -> i32 {
        const MAX: u32 = u32::MAX;
        let mut val = val.get();
        if val <= MAX / 100_000_000 {
            val *= 100_000_000;
            // At this point, we have shifted out 8 digits, and we can only shift out 2 more.
            // We can only check up to -2 more because -10 <= log <= -8.
            if val > MAX / 10 {
                -9
            } else {
                debug_assert!(val > MAX / 100);
                -10
            }
        } else {
            greater_equal_m8_u32(val)
        }
    }

    // 0 < val <= MAX
    // -20 <= log <= -1
    pub const fn u64(val: NonZeroU64) -> i32 {
        const MAX: u64 = u64::MAX;
        let mut val = val.get();
        let mut log = 0;
        if val <= MAX / 10_000_000_000_000_000 {
            // After this, we can only check up to -4 more because -20 <= log <= -16.
            // That is, we can skip the checks against MAX / 100_000_000 and MAX / 10_000.
            val *= 10_000_000_000_000_000;
            log += -16;
        } else {
            if val <= MAX / 100_000_000 {
                val *= 100_000_000;
                log += -8;
            }
            if val <= MAX / 10_000 {
                val *= 10_000;
                log += -4;
            }
        }
        log + if val > MAX / 10 {
            -1
        } else if val > MAX / 100 {
            -2
        } else if val > MAX / 1000 {
            -3
        } else {
            debug_assert!(val > MAX / 10_000);
            -4
        }
    }

    // 0 < val <= MAX
    // -39 <= log <= -1
    pub const fn u128(val: NonZeroU128) -> i32 {
        const MAX: u128 = u128::MAX;
        let mut val = val.get();
        let mut log = 0;
        if val <= MAX / 100_000_000_000_000_000_000_000_000_000_000 {
            val *= 100_000_000_000_000_000_000_000_000_000_000;
            // At this point we have shifted out 32 digits, and we can only shift out 7 more.
            // We can
            //   * use val >> 96 because we have shifted out 32 decimal digits (106 bits)
            //   * only check up to -8 more because -39 <= log <= -32
            return -32 + greater_equal_m8_u32((val >> 96) as u32);
        }
        if val <= MAX / 10_000_000_000_000_000 {
            val *= 10_000_000_000_000_000;
            log += -16;
        }
        if val <= MAX / 100_000_000 {
            val *= 100_000_000;
            log += -8;
        }
        if log == -24 {
            // At this point we have shifted out 24 digits, and we can only shift out 15 more.
            // We can
            //   * use val >> 64 because we have shifted out 24 decimal digits (79 bits)
            //   * only check up to -8 more because -32 <= log <= -24
            return -24 + greater_equal_m8_u64((val >> 64) as u64);
        }
        // We have *not* shifted out enough decimal digits, so we must *not* convert to u32 or u64.
        if val <= MAX / 10_000 {
            val *= 10_000;
            log += -4;
        }
        log + if val > MAX / 10 {
            -1
        } else if val > MAX / 100 {
            -2
        } else if val > MAX / 1000 {
            -3
        } else {
            debug_assert!(val > MAX / 10_000);
            -4
        }
    }

    // MAX / 100_000_000 < val <= MAX
    // -8 <= log <= -1
    const fn greater_equal_m8_u32(mut val: u32) -> i32 {
        const MAX: u32 = u32::MAX;
        debug_assert!(val > MAX / 100_000_000);
        let mut log = 0;
        if val <= MAX / 10_000 {
            val *= 10_000;
            log += -4;
        }
        log + if val > MAX / 10 {
            -1
        } else if val > MAX / 100 {
            -2
        } else if val > MAX / 1000 {
            -3
        } else {
            debug_assert!(val > MAX / 10_000);
            -4
        }
    }

    // MAX / 100_000_000 < val <= MAX
    // -8 <= log <= 1
    const fn greater_equal_m8_u64(mut val: u64) -> i32 {
        const MAX: u64 = u64::MAX;
        debug_assert!(val > MAX / 100_000_000);
        let mut log = 0;
        if val <= MAX / 10_000 {
            val *= 10_000;
            log += -4;
        }
        log + if val > MAX / 10 {
            -1
        } else if val > MAX / 100 {
            -2
        } else if val > MAX / 1000 {
            -3
        } else {
            debug_assert!(val > MAX / 10_000);
            -4
        }
    }
}

// check log10() and log(10) in tandem
#[cfg(test)]
mod tests {
    use crate::log::Base;
    use crate::{log, log10};
    use core::num::{NonZeroU128, NonZeroU16, NonZeroU32, NonZeroU64, NonZeroU8};

    const DEC: Base = match Base::new(10) {
        Some(s) => s,
        None => unreachable!(),
    };

    macro_rules! check_loop {
        ($T:ident, $NZ:ident) => {
            for i in 0..=$T::MAX.ilog10() as i32 {
                let p = (10 as $T).pow(i as u32);
                let nz = $NZ::new(p).unwrap();
                if i > 0 {
                    let nz_m1 = $NZ::new(p - 1).unwrap();
                    assert_eq!((p - 1).ilog10() as i32, i - 1);
                    assert_eq!(log::int_part::$T(nz_m1, DEC), i - 1);
                }
                assert_eq!(p.ilog10() as i32, i);
                assert_eq!(log::int_part::$T(nz, DEC), i);
                let nz_p1 = $NZ::new(p + 1).unwrap();
                assert_eq!((p + 1).ilog10() as i32, i);
                assert_eq!(log::int_part::$T(nz_p1, DEC), i);
            }

            for i in 0..-log10::frac_part::$T($NZ::new(1).unwrap()) {
                let p = <$T>::MAX / (10 as $T).pow(i as u32);
                let nz = $NZ::new(p).unwrap();
                if p > 1 {
                    let nz_m1 = $NZ::new(p - 1).unwrap();
                    assert_eq!(log10::frac_part::$T(nz_m1), -1 - i);
                    assert_eq!(log::frac_part::$T(nz_m1, DEC), -1 - i);
                }
                assert_eq!(log10::frac_part::$T(nz), -1 - i);
                assert_eq!(log::frac_part::$T(nz, DEC), -1 - i);
                if i > 0 {
                    let nz_p1 = $NZ::new(p + 1).unwrap();
                    assert_eq!(log10::frac_part::$T(nz_p1), -i);
                    assert_eq!(log::frac_part::$T(nz_p1, DEC), -i);
                }
            }
        };
    }

    #[test]
    fn log10_u8() {
        let one = NonZeroU8::new(1).unwrap();

        assert_eq!(1u8.ilog10(), 0);
        assert_eq!(log::int_part::u8(one, DEC), 0);
        assert_eq!(u8::MAX.ilog10(), 2);
        assert_eq!(log::int_part::u8(NonZeroU8::MAX, DEC), 2);
        assert_eq!(log10::frac_part::u8(one), -3);
        assert_eq!(log::frac_part::u8(one, DEC), -3);
        assert_eq!(log10::frac_part::u8(NonZeroU8::MAX), -1);
        assert_eq!(log::frac_part::u8(NonZeroU8::MAX, DEC), -1);

        check_loop! { u8, NonZeroU8 }
    }

    #[test]
    fn log10_u16() {
        let one = NonZeroU16::new(1).unwrap();

        assert_eq!(1u16.ilog10(), 0);
        assert_eq!(log::int_part::u16(one, DEC), 0);
        assert_eq!(u16::MAX.ilog10(), 4);
        assert_eq!(log::int_part::u16(NonZeroU16::MAX, DEC), 4);
        assert_eq!(log10::frac_part::u16(one), -5);
        assert_eq!(log::frac_part::u16(one, DEC), -5);
        assert_eq!(log10::frac_part::u16(NonZeroU16::MAX), -1);
        assert_eq!(log::frac_part::u16(NonZeroU16::MAX, DEC), -1);

        check_loop! { u16, NonZeroU16 }
    }

    #[test]
    fn log10_u32() {
        let one = NonZeroU32::new(1).unwrap();

        assert_eq!(1u32.ilog10(), 0);
        assert_eq!(log::int_part::u32(NonZeroU32::new(1).unwrap(), DEC), 0);
        assert_eq!(u32::MAX.ilog10(), 9);
        assert_eq!(log::int_part::u32(NonZeroU32::MAX, DEC), 9);
        assert_eq!(log10::frac_part::u32(one), -10);
        assert_eq!(log::frac_part::u32(one, DEC), -10);
        assert_eq!(log10::frac_part::u32(NonZeroU32::MAX), -1);
        assert_eq!(log::frac_part::u32(NonZeroU32::MAX, DEC), -1);

        check_loop! { u32, NonZeroU32 }
    }

    #[test]
    fn log10_u64() {
        let one = NonZeroU64::new(1).unwrap();

        assert_eq!(1u64.ilog10(), 0);
        assert_eq!(log::int_part::u64(one, DEC), 0);
        assert_eq!(u64::MAX.ilog10(), 19);
        assert_eq!(log::int_part::u64(NonZeroU64::MAX, DEC), 19);
        assert_eq!(log10::frac_part::u64(one), -20);
        assert_eq!(log::frac_part::u64(one, DEC), -20);
        assert_eq!(log10::frac_part::u64(NonZeroU64::MAX), -1);
        assert_eq!(log::frac_part::u64(NonZeroU64::MAX, DEC), -1);

        check_loop! { u64, NonZeroU64 }
    }

    #[test]
    fn log10_u128() {
        let one = NonZeroU128::new(1).unwrap();

        assert_eq!(1u128.ilog10(), 0);
        assert_eq!(log::int_part::u128(one, DEC), 0);
        assert_eq!(u128::MAX.ilog10(), 38);
        assert_eq!(log::int_part::u128(NonZeroU128::MAX, DEC), 38);
        assert_eq!(log10::frac_part::u128(one), -39);
        assert_eq!(log::frac_part::u128(one, DEC), -39);
        assert_eq!(log10::frac_part::u128(NonZeroU128::MAX), -1);
        assert_eq!(log::frac_part::u128(NonZeroU128::MAX, DEC), -1);

        check_loop! { u128, NonZeroU128 }
    }
}
