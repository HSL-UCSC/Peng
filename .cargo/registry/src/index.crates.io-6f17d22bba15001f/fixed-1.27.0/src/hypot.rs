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

use crate::int256;
use crate::int256::U256;
use core::num::{NonZeroU128, NonZeroU16, NonZeroU32, NonZeroU64};

// The square root method used is based on code by Martin Guy @UKC, June 1985.
// His method of square root by abacus method is from a book on programming
// abaci by Mr C. Woo.
// http://medialab.freaknet.org/martin/src/sqrt/

macro_rules! impl_hypot {
    ($Single:ident, $Double:ident, $NonZeroDouble:ident $(, $Half:ident)?) => {
        pub const fn $Single(a: $Single, b: $Single) -> ($Single, bool) {
            $(
                if a <= ($Half::MAX as $Single) && b <= ($Half::MAX as $Single) {
                    let val = match $Half(a as $Half, b as $Half) {
                        (val, false) => val as $Single,
                        (val, true) => (val as $Single) + (1 << $Half::BITS),
                    };
                    return (val, false);
                }
            )?

            let aa = (a as $Double) * (a as $Double);
            let bb = (b as $Double) * (b as $Double);
            let (sum, overflow) = aa.overflowing_add(bb);

            let mut x = sum;
            let mut y;
            let mut bit;
            if overflow {
                // Perform initialization and two iterations assuming word wider than n bits.
                // Initialization:
                //     Set y = 0
                //     Set bit = 1 << n
                // First iteration:
                //     Since x >= y + bit (because the overflow is equivalent to 1 << n on x):
                //         x -= y + bit (removing the effect of the overflow on x)
                //         y = (y >> 1) + bit (y becomes 1 << n)
                //     bit >>= 2 (bit becomes 1 << (n - 2))
                // Second iteration:
                //     Now x < y (because y is 1 << n):
                //         y >>= 1 (y becomes 1 << (n - 1))
                //     bit >>= 2 (bit becomes 1 << (n - 4))
                // In effect, we can set y = 1 << (n - 1), and bit = 1 << (n - 4)
                y = 1 << ($Double::BITS - 1);
                bit = 1 << ($Double::BITS - 4);
            } else {
                let sum_lz = match $NonZeroDouble::new(sum) {
                    None => return (0, false),
                    Some(s) => s.leading_zeros(),
                };
                y = 0;
                bit = 1 << ($Double::BITS - 2 - sum_lz / 2 * 2);
            }
            while bit != 0 {
                let y_plus_bit = y + bit;
                y >>= 1;
                if x >= y_plus_bit {
                    x -= y_plus_bit;
                    y += bit;
                }
                bit >>= 2;
            }
            debug_assert!((y >> $Single::BITS) as $Single == overflow as $Single);
            (y as $Single, overflow)
        }
    };
}

impl_hypot! { u8, u16, NonZeroU16 }
impl_hypot! { u16, u32, NonZeroU32, u8 }
impl_hypot! { u32, u64, NonZeroU64, u16 }
impl_hypot! { u64, u128, NonZeroU128, u32 }

pub const fn u128(a: u128, b: u128) -> (u128, bool) {
    if a <= (u64::MAX as u128) && b <= (u64::MAX as u128) {
        let val = match u64(a as u64, b as u64) {
            (val, false) => val as u128,
            (val, true) => (val as u128) + (1 << 64),
        };
        return (val, false);
    }

    let aa = int256::wide_mul_u128(a, a);
    let bb = int256::wide_mul_u128(b, b);
    let (sum, overflow) = int256::overflowing_add_u256(aa, bb);

    let mut x = sum;
    let mut y;
    let mut bit;
    if overflow {
        // Perform initialization and two iterations assuming word wider than n bits.
        // Initialization:
        //     Set y = 0
        //     Set bit = 1 << n
        // First iteration:
        //     Since x >= y + bit (because the overflow is equivalent to 1 << n on x):
        //         x -= y + bit (removing the effect of the overflow on x)
        //         y = (y >> 1) + bit (y becomes 1 << n)
        //     bit >>= 2 (bit becomes 1 << (n - 2))
        // Second iteration:
        //     Now x < y (because y is 1 << n):
        //         y >>= 1 (y becomes 1 << (n - 1))
        //     bit >>= 2 (bit becomes 1 << (n - 4))
        // In effect, we can set y = 1 << (n - 1), and bit = 1 << (n - 4)
        y = U256 {
            lo: 0,
            hi: 1 << 127,
        };
        bit = 1 << 124;
    } else {
        y = U256 { lo: 0, hi: 0 };
        bit = match NonZeroU128::new(sum.hi) {
            None => panic!("small operands; should have used crate::hypot::u64"),
            Some(s) => 1 << (126 - s.leading_zeros() / 2 * 2),
        };
    }
    while bit != 0 {
        let y_hi_plus_bit = y.hi + bit;
        y.hi >>= 1;
        if x.hi >= y_hi_plus_bit {
            x.hi -= y_hi_plus_bit;
            y.hi += bit;
        }
        bit >>= 2;
    }
    bit = 1 << 126;
    while bit != 0 {
        let y_plus_bit = U256 {
            lo: y.lo + bit,
            hi: y.hi,
        };
        y.lo = (y.lo >> 1) + ((y.hi & 1) << 127);
        y.hi >>= 1;
        if (x.hi > y_plus_bit.hi) || (x.hi == y_plus_bit.hi && x.lo >= y_plus_bit.lo) {
            x = int256::wrapping_sub_u256(x, y_plus_bit);
            y.lo += bit;
        }
        bit >>= 2
    }

    debug_assert!(y.hi == overflow as u128);
    (y.lo, overflow)
}

#[cfg(test)]
mod tests {
    use crate::hypot;
    use crate::types::{U1F127, U1F15, U1F31, U1F63, U1F7};

    #[test]
    fn check_max() {
        assert_eq!(hypot::u8(u8::MAX, u8::MAX), (104, true));
        assert_eq!(hypot::u16(u16::MAX, u16::MAX), (27_144, true));
        assert_eq!(hypot::u32(u32::MAX, u32::MAX), (1_779_033_702, true));
        assert_eq!(
            hypot::u64(u64::MAX, u64::MAX),
            (7_640_891_576_956_012_807, true)
        );
        assert_eq!(
            hypot::u128(u128::MAX, u128::MAX),
            (140_949_571_415_070_559_626_692_937_523_481_902_396, true)
        );
    }

    #[test]
    fn check_zero() {
        assert_eq!(hypot::u8(0, 0), (0, false));
        assert_eq!(hypot::u16(0, 0), (0, false));
        assert_eq!(hypot::u32(0, 0), (0, false));
        assert_eq!(hypot::u64(0, 0), (0, false));
        assert_eq!(hypot::u128(0, 0), (0, false));
    }

    #[test]
    fn check_zero_max() {
        assert_eq!(hypot::u8(u8::MAX, 0), (u8::MAX, false));
        assert_eq!(hypot::u8(0, u8::MAX), (u8::MAX, false));
        assert_eq!(hypot::u16(u16::MAX, 0), (u16::MAX, false));
        assert_eq!(hypot::u16(0, u16::MAX), (u16::MAX, false));
        assert_eq!(hypot::u32(u32::MAX, 0), (u32::MAX, false));
        assert_eq!(hypot::u32(0, u32::MAX), (u32::MAX, false));
        assert_eq!(hypot::u64(u64::MAX, 0), (u64::MAX, false));
        assert_eq!(hypot::u64(0, u64::MAX), (u64::MAX, false));
        assert_eq!(hypot::u128(u128::MAX, 0), (u128::MAX, false));
        assert_eq!(hypot::u128(0, u128::MAX), (u128::MAX, false));
    }

    #[test]
    fn check_max_plus() {
        // hypot(2^n - 1, x) = 2^n; x = sqrt(2^(n+1) - 1)
        // e.g. for u32, sqrt(2^33 - 1) = 92681.9
        assert_eq!(hypot::u8(u8::MAX, 22), (u8::MAX, false));
        assert_eq!(hypot::u8(u8::MAX, 23), (0, true));
        assert_eq!(hypot::u16(u16::MAX, 362), (u16::MAX, false));
        assert_eq!(hypot::u16(u16::MAX, 363), (0, true));
        assert_eq!(hypot::u32(u32::MAX, 92_681), (u32::MAX, false));
        assert_eq!(hypot::u32(u32::MAX, 92_682), (0, true));
        assert_eq!(hypot::u64(u64::MAX, 6_074_000_999), (u64::MAX, false));
        assert_eq!(hypot::u64(u64::MAX, 6_074_001_000), (0, true));
        assert_eq!(
            hypot::u128(u128::MAX, 26_087_635_650_665_564_424),
            (u128::MAX, false)
        );
        assert_eq!(
            hypot::u128(u128::MAX, 26_087_635_650_665_564_425),
            (0, true)
        );
    }

    #[test]
    fn check_sqrt_2() {
        assert_eq!(hypot::u8(1 << 7, 1 << 7), (U1F7::SQRT_2.to_bits(), false));
        assert_eq!(
            hypot::u16(1 << 15, 1 << 15),
            (U1F15::SQRT_2.to_bits(), false)
        );
        assert_eq!(
            hypot::u32(1 << 31, 1 << 31),
            (U1F31::SQRT_2.to_bits(), false)
        );
        assert_eq!(
            hypot::u64(1 << 63, 1 << 63),
            (U1F63::SQRT_2.to_bits(), false)
        );
        assert_eq!(
            hypot::u128(1 << 127, 1 << 127),
            (U1F127::SQRT_2.to_bits(), false)
        );
    }
}
