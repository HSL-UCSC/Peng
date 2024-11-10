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

macro_rules! fixed_const {
    (
        Self = $Self:ident,
        Signedness = $Signedness:ident,
        LeEqU = $LeEqU:ident,
        [nm4 ..= n] = [$nm4:literal, $nm3:literal, $nm2:literal, $nm1:literal, $n:literal],
        [LeEqUC0 ..= LeEqUC3] = [$LeEqUC0:ident, $LeEqUC1:ident, $LeEqUC2:ident, $LeEqUC3:ident],
    ) => {
        impl<Frac: Unsigned> $Self<Frac> {
            const fn from_const<SrcFrac: Unsigned>(src: FixedU128<SrcFrac>) -> $Self<Frac> {
                let right = SrcFrac::U32 - Frac::U32;
                let bits128 = if right == 128 {
                    0
                } else {
                    src.to_bits() >> right
                };
                $Self::from_bits(bits128 as _)
            }
        }

        comment! {
            "This block contains constants in the range 0&nbsp;<&nbsp;<i>x</i>&nbsp;<&nbsp;0.5.

# Examples

```rust
use fixed::{consts, types::extra::U", $n, ", ", stringify!($Self), "};
type Fix = ", stringify!($Self), "<U", $n, ">;
assert_eq!(Fix::LOG10_2, Fix::from_num(consts::LOG10_2));
```
";
            impl<Frac: $LeEqU> $Self<Frac> {
                /// 1/τ = 0.159154…
                pub const FRAC_1_TAU: $Self<Frac> = Self::from_const(consts::FRAC_1_TAU);

                /// 2/τ = 0.318309…
                pub const FRAC_2_TAU: $Self<Frac> = Self::from_const(consts::FRAC_2_TAU);

                /// π/8 = 0.392699…
                pub const FRAC_PI_8: $Self<Frac> = Self::from_const(consts::FRAC_PI_8);

                /// 1/π = 0.318309…
                pub const FRAC_1_PI: $Self<Frac> = Self::from_const(consts::FRAC_1_PI);

                /// log<sub>10</sub> 2 = 0.301029…
                pub const LOG10_2: $Self<Frac> = Self::from_const(consts::LOG10_2);

                /// log<sub>10</sub> e = 0.434294…
                pub const LOG10_E: $Self<Frac> = Self::from_const(consts::LOG10_E);
            }
        }

        comment! {
            "This block contains constants in the range 0.5&nbsp;≤&nbsp;<i>x</i>&nbsp;<&nbsp;1",
            if_signed_else_empty_str! {
                $Signedness;
                ", and &minus;1.

These constants are not representable in signed fixed-point numbers with less
than 1 integer bit"
            },
            ".

# Examples

```rust
use fixed::{consts, types::extra::U",
            if_signed_unsigned!($Signedness, $nm1, $n),
            ", ", stringify!($Self), "};
type Fix = ", stringify!($Self), "<U",
            if_signed_unsigned!($Signedness, $nm1, $n),
            ">;
assert_eq!(Fix::LN_2, Fix::from_num(consts::LN_2));
assert!(0.5 <= Fix::LN_2 && Fix::LN_2 < 1);
```
",
            if_signed_else_empty_str! {
                $Signedness;
                "
The following example fails to compile, since the maximum
representable value with ", $n, " fractional bits and 0 integer
bits is <&nbsp;0.5.

```rust,compile_fail
use fixed::{consts, types::extra::U", $n, ", ", stringify!($Self), "};
type Fix = ", stringify!($Self), "<U", $n, ">;
let _ = Fix::LN_2;
```
"
            };
            impl<Frac: Unsigned> $Self<Frac>
            where
                Frac: IsLessOrEqual<$LeEqUC0, Output = True>,
            {
                if_signed! {
                    $Signedness;
                    comment! {
                        "Negative one.

# Examples

```rust
use fixed::{types::extra::U", $nm1, ", ", stringify!($Self), "};
type Fix = ", stringify!($Self), "<U", $nm1, ">;
assert_eq!(Fix::NEG_ONE, Fix::from_num(-1));
```

The following would fail as
<code>[", stringify!($Self), "]&lt;[U", $nm1, "]></code>
cannot represent 1, so there is no
<code>[", stringify!($Self), "]::&lt;[U", $nm1, "]>::[ONE]</code>.

[ONE]: ", stringify!($Self), "::ONE

```rust,compile_fail
use fixed::{types::extra::U", $nm1, ", ", stringify!($Self), "};
const _ERROR: ", stringify!($Self), "<U", $nm1, "> = ", stringify!($Self), "::ONE.unwrapped_neg();
```
";
                        pub const NEG_ONE: $Self<Frac> = Self::from_bits(-1 << Frac::U32);
                    }
                }

                /// τ/8 = 0.785398…
                pub const FRAC_TAU_8: $Self<Frac> = Self::from_const(consts::FRAC_TAU_8);

                /// τ/12 = 0.523598…
                pub const FRAC_TAU_12: $Self<Frac> = Self::from_const(consts::FRAC_TAU_12);

                /// 4/τ = 0.636619…
                pub const FRAC_4_TAU: $Self<Frac> = Self::from_const(consts::FRAC_4_TAU);

                /// π/4 = 0.785398…
                pub const FRAC_PI_4: $Self<Frac> = Self::from_const(consts::FRAC_PI_4);

                /// π/6 = 0.523598…
                pub const FRAC_PI_6: $Self<Frac> = Self::from_const(consts::FRAC_PI_6);

                /// 2/π = 0.636619…
                pub const FRAC_2_PI: $Self<Frac> = Self::from_const(consts::FRAC_2_PI);

                /// 1/√π = 0.564189…
                pub const FRAC_1_SQRT_PI: $Self<Frac> = Self::from_const(consts::FRAC_1_SQRT_PI);

                /// 1/√2 = 0.707106…
                pub const FRAC_1_SQRT_2: $Self<Frac> = Self::from_const(consts::FRAC_1_SQRT_2);

                /// 1/√3 = 0.577350…
                pub const FRAC_1_SQRT_3: $Self<Frac> = Self::from_const(consts::FRAC_1_SQRT_3);

                /// ln 2 = 0.693147…
                pub const LN_2: $Self<Frac> = Self::from_const(consts::LN_2);

                /// The golden ratio conjugate, Φ = 1/φ = 0.618033…
                pub const FRAC_1_PHI: $Self<Frac> = Self::from_const(consts::FRAC_1_PHI);

                /// The Euler-Mascheroni constant, γ = 0.577215…
                pub const GAMMA: $Self<Frac> = Self::from_const(consts::GAMMA);

                /// Catalan’s constant = 0.915965…
                pub const CATALAN: $Self<Frac> = Self::from_const(consts::CATALAN);
            }
        }

        comment! {
            "This block contains constants in the range 1&nbsp;≤&nbsp;<i>x</i>&nbsp;<&nbsp;2.

These constants are not representable in ",
            if_signed_unsigned!($Signedness, "signed", "unsigned"),
            " fixed-point numbers with less than ",
            if_signed_unsigned!($Signedness, "2 integer bits", "1 integer bit"),
            ".

# Examples

```rust
use fixed::{consts, types::extra::U",
            if_signed_unsigned!($Signedness, $nm2, $nm1),
            ", ", stringify!($Self), "};
type Fix = ", stringify!($Self), "<U",
            if_signed_unsigned!($Signedness, $nm2, $nm1),
            ">;
assert_eq!(Fix::LOG2_E, Fix::from_num(consts::LOG2_E));
assert!(1 <= Fix::LOG2_E && Fix::LOG2_E < 2);
```

The following example fails to compile, since the maximum
representable value with ",
            if_signed_unsigned!($Signedness, $nm1, $n),
            " fractional bits and ",
            if_signed_unsigned!($Signedness, "1 integer bit", "0 integer bits"),
            " is <&nbsp;1.

```rust,compile_fail
use fixed::{consts, types::extra::U",
            if_signed_unsigned!($Signedness, $nm1, $n),
            ", ", stringify!($Self), "};
type Fix = ", stringify!($Self), "<U",
            if_signed_unsigned!($Signedness, $nm1, $n),
            ">;
let _ = Fix::LOG2_E;
```
";
            impl<Frac: Unsigned> $Self<Frac>
            where
                Frac: IsLessOrEqual<$LeEqUC1, Output = True>,
            {
                comment! {
                    "One.

# Examples

```rust
use fixed::{types::extra::U4, ", stringify!($Self), "};
type Fix = ", stringify!($Self), "<U4>;
assert_eq!(Fix::ONE, Fix::from_num(1));
```
";
                    pub const ONE: $Self<Frac> = Self::from_bits(1 << Frac::U32);
                }

                /// τ/4 = 1.57079…
                pub const FRAC_TAU_4: $Self<Frac> = Self::from_const(consts::FRAC_TAU_4);

                /// τ/6 = 1.04719…
                pub const FRAC_TAU_6: $Self<Frac> = Self::from_const(consts::FRAC_TAU_6);

                /// π/2 = 1.57079…
                pub const FRAC_PI_2: $Self<Frac> = Self::from_const(consts::FRAC_PI_2);

                /// π/3 = 1.04719…
                pub const FRAC_PI_3: $Self<Frac> = Self::from_const(consts::FRAC_PI_3);

                /// √π = 1.77245…
                pub const SQRT_PI: $Self<Frac> = Self::from_const(consts::SQRT_PI);

                /// 2/√π = 1.12837…
                pub const FRAC_2_SQRT_PI: $Self<Frac> = Self::from_const(consts::FRAC_2_SQRT_PI);

                /// √2 = 1.41421…
                pub const SQRT_2: $Self<Frac> = Self::from_const(consts::SQRT_2);

                /// √3 = 1.73205…
                pub const SQRT_3: $Self<Frac> = Self::from_const(consts::SQRT_3);

                /// √e = 1.64872…
                pub const SQRT_E: $Self<Frac> = Self::from_const(consts::SQRT_E);

                /// log<sub>2</sub> e = 1.44269…
                pub const LOG2_E: $Self<Frac> = Self::from_const(consts::LOG2_E);

                /// The golden ratio, φ = 1.61803…
                pub const PHI: $Self<Frac> = Self::from_const(consts::PHI);

                /// √φ = 1.27201…
                pub const SQRT_PHI: $Self<Frac> = Self::from_const(consts::SQRT_PHI);
            }
        }

        comment! {
            "This block contains constants in the range 2&nbsp;≤&nbsp;<i>x</i>&nbsp;<&nbsp;4.

These constants are not representable in ",
            if_signed_unsigned!($Signedness, "signed", "unsigned"),
            " fixed-point numbers with less than ",
            if_signed_unsigned!($Signedness, 3, 2),
            " integer bits.

# Examples

```rust
use fixed::{consts, types::extra::U",
            if_signed_unsigned!($Signedness, $nm3, $nm2),
            ", ", stringify!($Self), "};
type Fix = ", stringify!($Self), "<U",
            if_signed_unsigned!($Signedness, $nm3, $nm2),
            ">;
assert_eq!(Fix::E, Fix::from_num(consts::E));
assert!(2 <= Fix::E && Fix::E < 4);
```

The following example fails to compile, since the maximum
representable value with ",
            if_signed_unsigned!($Signedness, $nm2, $nm1),
            " fractional bits and ",
            if_signed_unsigned!($Signedness, "2 integer bits", "1 integer bit"),
            " is <&nbsp;2.

```rust,compile_fail
use fixed::{consts, types::extra::U",
            if_signed_unsigned!($Signedness, $nm2, $nm1),
            ", ", stringify!($Self), "};
type Fix = ", stringify!($Self), "<U",
            if_signed_unsigned!($Signedness, $nm2, $nm1),
            ">;
let _ = Fix::E;
```
";
            impl<Frac: Unsigned> $Self<Frac>
            where
                Frac: IsLessOrEqual<$LeEqUC2, Output = True>,
            {
                /// τ/2 = 3.14159…
                pub const FRAC_TAU_2: $Self<Frac> = Self::from_const(consts::FRAC_TAU_2);

                /// τ/3 = 2.09439…
                pub const FRAC_TAU_3: $Self<Frac> = Self::from_const(consts::FRAC_TAU_3);

                /// Archimedes’ constant, π = 3.14159…
                pub const PI: $Self<Frac> = Self::from_const(consts::PI);

                /// Euler’s number, e = 2.71828…
                pub const E: $Self<Frac> = Self::from_const(consts::E);

                /// log<sub>2</sub> 10 = 3.32192…
                pub const LOG2_10: $Self<Frac> = Self::from_const(consts::LOG2_10);

                /// ln 10 = 2.30258…
                pub const LN_10: $Self<Frac> = Self::from_const(consts::LN_10);
            }
        }

        comment! {
            "This block contains constants in the range 4&nbsp;≤&nbsp;<i>x</i>&nbsp;<&nbsp;8.

These constants are not representable in ",
            if_signed_unsigned!($Signedness, "signed", "unsigned"),
            " fixed-point numbers with less than ",
            if_signed_unsigned!($Signedness, 4, 3),
            " integer bits.

# Examples

```rust
use fixed::{consts, types::extra::U",
            if_signed_unsigned!($Signedness, $nm4, $nm3),
            ", ", stringify!($Self), "};
type Fix = ", stringify!($Self), "<U",
            if_signed_unsigned!($Signedness, $nm4, $nm3),
            ">;
assert_eq!(Fix::TAU, Fix::from_num(consts::TAU));
assert!(4 <= Fix::TAU && Fix::TAU < 8);
```

The following example fails to compile, since the maximum
representable value with ",
            if_signed_unsigned!($Signedness, $nm3, $nm2),
            " fractional bits and ",
            if_signed_unsigned!($Signedness, 3, 2),
            " integer bits is <&nbsp;4.

```rust,compile_fail
use fixed::{consts, types::extra::U",
            if_signed_unsigned!($Signedness, $nm3, $nm2),
            ", ", stringify!($Self), "};
type Fix = ", stringify!($Self), "<U",
            if_signed_unsigned!($Signedness, $nm3, $nm2),
            ">;
let _ = Fix::TAU;
```
";
            impl<Frac: Unsigned> $Self<Frac>
            where
                Frac: IsLessOrEqual<$LeEqUC3, Output = True>,
            {
                /// A turn, τ = 6.28318…
                pub const TAU: $Self<Frac> = Self::from_const(consts::TAU);
            }
        }
    };
}
