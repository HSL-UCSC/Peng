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

#[derive(Clone, Copy, Debug)]
pub struct Base(u32);

impl Base {
    pub const fn new(base: u32) -> Option<Base> {
        if base >= 2 {
            Some(Base(base))
        } else {
            None
        }
    }

    pub const fn get(self) -> u32 {
        self.0
    }
}

macro_rules! impl_int_part {
    ($u:ident, $NZ:ident) => {
        pub const fn $u(val: $NZ, base: Base) -> i32 {
            const MAX_TABLE_SIZE: usize = ($u::BITS.ilog2() - 1) as usize;

            let val = val.get();
            let base = base.get();

            let baseu = base as $u;
            if baseu as u32 != base || val < baseu {
                return 0;
            }

            // base^1, base^2, base^4, etc.
            let mut base_powers: [$u; MAX_TABLE_SIZE] = [0; MAX_TABLE_SIZE];

            let mut i = 0;
            let mut partial_log = 1u32;
            let mut partial_val = baseu;

            loop {
                let square = match partial_val.checked_mul(partial_val) {
                    Some(s) if val >= s => s,
                    _ => break,
                };
                base_powers[i] = partial_val;
                i += 1;
                partial_log *= 2;
                partial_val = square;
            }
            let mut dlog = partial_log;
            while i > 0 {
                i -= 1;
                dlog /= 2;
                if let Some(mid) = partial_val.checked_mul(base_powers[i]) {
                    if val >= mid {
                        partial_val = mid;
                        partial_log += dlog;
                    }
                }
            }
            return partial_log as i32;
        }
    };
}

pub mod int_part {
    use crate::log::Base;
    use core::num::{NonZeroU128, NonZeroU16, NonZeroU32, NonZeroU64, NonZeroU8};

    impl_int_part! { u8, NonZeroU8 }
    impl_int_part! { u16, NonZeroU16 }
    impl_int_part! { u32, NonZeroU32 }
    impl_int_part! { u64, NonZeroU64 }
    impl_int_part! { u128, NonZeroU128 }
}

macro_rules! impl_frac_part {
    ($u:ident, $NZ:ident) => {
        pub const fn $u(val: $NZ, base: Base) -> i32 {
            const MAX_TABLE_SIZE: usize = ($u::BITS.ilog2() - 1) as usize;

            let val = val.get();
            let base = base.get();

            let baseu = base as $u;
            if baseu as u32 != base || val.checked_mul(baseu).is_none() {
                return -1;
            }

            // base^1, base^2, base^4, etc.
            let mut base_powers: [$u; MAX_TABLE_SIZE] = [0; MAX_TABLE_SIZE];

            let mut i = 0;
            let mut partial_log = 1u32;
            let mut partial_val = baseu;

            loop {
                let square = match partial_val.checked_mul(partial_val) {
                    Some(s) if val.checked_mul(s).is_some() => s,
                    _ => break,
                };
                base_powers[i] = partial_val;
                i += 1;
                partial_log *= 2;
                partial_val = square;
            }
            let mut dlog = partial_log;
            while i > 0 {
                i -= 1;
                dlog /= 2;
                if let Some(mid) = partial_val.checked_mul(base_powers[i]) {
                    if val.checked_mul(mid).is_some() {
                        partial_val = mid;
                        partial_log += dlog;
                    }
                }
            }
            return -1 - partial_log as i32;
        }
    };
}

pub mod frac_part {
    use crate::log::Base;
    use core::num::{NonZeroU128, NonZeroU16, NonZeroU32, NonZeroU64, NonZeroU8};

    impl_frac_part! { u8, NonZeroU8 }
    impl_frac_part! { u16, NonZeroU16 }
    impl_frac_part! { u32, NonZeroU32 }
    impl_frac_part! { u64, NonZeroU64 }
    impl_frac_part! { u128, NonZeroU128 }
}

#[cfg(test)]
mod tests {
    use crate::log;
    use crate::log::Base;
    use core::num::{NonZeroU128, NonZeroU16, NonZeroU32, NonZeroU64, NonZeroU8};

    // these tests require the maximum table sizes
    #[test]
    fn check_table_size_is_sufficient() {
        let bin = Base::new(2).unwrap();

        assert_eq!(log::int_part::u8(NonZeroU8::MAX, bin), 7);
        assert_eq!(log::int_part::u16(NonZeroU16::MAX, bin), 15);
        assert_eq!(log::int_part::u32(NonZeroU32::MAX, bin), 31);
        assert_eq!(log::int_part::u64(NonZeroU64::MAX, bin), 63);
        assert_eq!(log::int_part::u128(NonZeroU128::MAX, bin), 127);

        assert_eq!(log::frac_part::u8(NonZeroU8::new(1).unwrap(), bin), -8);
        assert_eq!(log::frac_part::u16(NonZeroU16::new(1).unwrap(), bin), -16);
        assert_eq!(log::frac_part::u32(NonZeroU32::new(1).unwrap(), bin), -32);
        assert_eq!(log::frac_part::u64(NonZeroU64::new(1).unwrap(), bin), -64);
        assert_eq!(
            log::frac_part::u128(NonZeroU128::new(1).unwrap(), bin),
            -128
        );
    }
}
