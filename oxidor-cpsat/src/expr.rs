use std::collections::BTreeMap;
use std::iter::Sum;
use std::ops::{Add, AddAssign, Mul, Neg, Sub, SubAssign};

/// An integer variable of a [`CpModelBuilder`](crate::CpModelBuilder).
///
/// A cheap copyable handle; the variable's state lives in the model. A handle
/// is only meaningful to the builder that created it (and that builder's
/// clones) — using it with another model is a programmer error and panics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct IntVar {
    pub(crate) model: u32,
    pub(crate) index: i32,
}

/// A Boolean literal: a Boolean variable of a
/// [`CpModelBuilder`](crate::CpModelBuilder) or its negation (see [`Self::not`]).
///
/// In linear expressions a literal contributes `1` when true and `0` when
/// false. A cheap copyable handle using CP-SAT's literal index encoding
/// (`-index - 1` for negations); like [`IntVar`], it is only meaningful to
/// the builder that created it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BoolVar {
    pub(crate) model: u32,
    pub(crate) index: i32,
}

impl BoolVar {
    /// The negation of this literal. Involutive: `b.not().not() == b`.
    #[allow(clippy::should_implement_trait)] // `Not` would collide with `!` on exprs later; explicit reads better.
    pub fn not(self) -> Self {
        Self {
            model: self.model,
            index: -self.index - 1,
        }
    }

    pub(crate) fn literal_index(self) -> i32 {
        self.index
    }
}

/// A linear expression: a sum of integer variables and Boolean literals with
/// `i64` coefficients, plus a constant.
///
/// Built with ordinary operators from [`IntVar`], [`BoolVar`], and `i64`:
///
/// ```
/// use oxidor_cpsat::{CpModelBuilder, LinearExpr};
///
/// let mut model = CpModelBuilder::new();
/// let x = model.new_int_var(0..=10);
/// let b = model.new_bool_var();
///
/// let expr = 2 * x - 3 * b.not() + 1;
/// let total: LinearExpr = [x, x, x].into_iter().sum();
/// # let _ = (expr, total);
/// ```
///
/// # Panics
///
/// Combining variables of two different models in one expression is a
/// programmer error and panics.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct LinearExpr {
    /// The model the variables in `terms` belong to; `None` for a
    /// constant-only expression.
    pub(crate) model: Option<u32>,
    pub(crate) terms: Vec<(i32, i64)>,
    pub(crate) constant: i64,
}

impl LinearExpr {
    /// The sum of the given items (variables, literals, expressions, constants).
    pub fn sum<T: Into<LinearExpr>>(items: impl IntoIterator<Item = T>) -> Self {
        items
            .into_iter()
            .map(Into::into)
            .fold(Self::default(), Add::add)
    }

    /// Absorbs another expression's model identity, panicking on a mix of
    /// two different models.
    fn merge_model(&mut self, other: Option<u32>) {
        match (self.model, other) {
            (Some(mine), Some(theirs)) if mine != theirs => {
                panic!("cannot mix variables from two different models in one expression")
            }
            (None, Some(theirs)) => self.model = Some(theirs),
            _ => {}
        }
    }

    /// Collapses to `(vars, coeffs, constant)` with duplicate variables
    /// merged, zero coefficients dropped, and variables sorted by index.
    pub(crate) fn into_parts(self) -> (Vec<i32>, Vec<i64>, i64) {
        let mut merged: BTreeMap<i32, i64> = BTreeMap::new();
        for (var, coeff) in self.terms {
            *merged.entry(var).or_default() += coeff;
        }
        merged.retain(|_, coeff| *coeff != 0);
        let (vars, coeffs) = merged.into_iter().unzip();
        (vars, coeffs, self.constant)
    }
}

impl From<IntVar> for LinearExpr {
    fn from(var: IntVar) -> Self {
        Self {
            model: Some(var.model),
            terms: vec![(var.index, 1)],
            constant: 0,
        }
    }
}

impl From<BoolVar> for LinearExpr {
    fn from(literal: BoolVar) -> Self {
        let model = Some(literal.model);
        let index = literal.index;
        if index >= 0 {
            Self {
                model,
                terms: vec![(index, 1)],
                constant: 0,
            }
        } else {
            // not(x) contributes 1 - x.
            Self {
                model,
                terms: vec![(-index - 1, -1)],
                constant: 1,
            }
        }
    }
}

impl From<i64> for LinearExpr {
    fn from(constant: i64) -> Self {
        Self {
            model: None,
            terms: Vec::new(),
            constant,
        }
    }
}

impl<R: Into<LinearExpr>> Add<R> for LinearExpr {
    type Output = LinearExpr;
    fn add(mut self, rhs: R) -> LinearExpr {
        self += rhs;
        self
    }
}

impl<R: Into<LinearExpr>> Sub<R> for LinearExpr {
    type Output = LinearExpr;
    fn sub(mut self, rhs: R) -> LinearExpr {
        self -= rhs;
        self
    }
}

impl<R: Into<LinearExpr>> AddAssign<R> for LinearExpr {
    fn add_assign(&mut self, rhs: R) {
        let rhs = rhs.into();
        self.merge_model(rhs.model);
        self.terms.extend(rhs.terms);
        self.constant += rhs.constant;
    }
}

impl<R: Into<LinearExpr>> SubAssign<R> for LinearExpr {
    fn sub_assign(&mut self, rhs: R) {
        let rhs = rhs.into();
        self.merge_model(rhs.model);
        self.terms
            .extend(rhs.terms.into_iter().map(|(var, coeff)| (var, -coeff)));
        self.constant -= rhs.constant;
    }
}

impl Mul<i64> for LinearExpr {
    type Output = LinearExpr;
    fn mul(mut self, factor: i64) -> LinearExpr {
        for (_, coeff) in &mut self.terms {
            *coeff *= factor;
        }
        self.constant *= factor;
        self
    }
}

impl Neg for LinearExpr {
    type Output = LinearExpr;
    fn neg(self) -> LinearExpr {
        self * -1
    }
}

impl<T: Into<LinearExpr>> Sum<T> for LinearExpr {
    fn sum<I: Iterator<Item = T>>(iter: I) -> Self {
        Self::sum(iter)
    }
}

/// Forwards arithmetic on variable handles to [`LinearExpr`].
macro_rules! forward_ops_to_linear_expr {
    ($($handle:ty),+) => {$(
        impl<R: Into<LinearExpr>> Add<R> for $handle {
            type Output = LinearExpr;
            fn add(self, rhs: R) -> LinearExpr { LinearExpr::from(self) + rhs }
        }
        impl<R: Into<LinearExpr>> Sub<R> for $handle {
            type Output = LinearExpr;
            fn sub(self, rhs: R) -> LinearExpr { LinearExpr::from(self) - rhs }
        }
        impl Mul<i64> for $handle {
            type Output = LinearExpr;
            fn mul(self, factor: i64) -> LinearExpr { LinearExpr::from(self) * factor }
        }
        impl Neg for $handle {
            type Output = LinearExpr;
            fn neg(self) -> LinearExpr { -LinearExpr::from(self) }
        }
    )+};
}
forward_ops_to_linear_expr!(IntVar, BoolVar);

/// `i64`-on-the-left arithmetic (`3 * x`, `1 - b`). Coherence requires one
/// impl per concrete right-hand type.
macro_rules! impl_i64_lhs_ops {
    ($($rhs:ty),+) => {$(
        impl Add<$rhs> for i64 {
            type Output = LinearExpr;
            fn add(self, rhs: $rhs) -> LinearExpr { LinearExpr::from(self) + rhs }
        }
        impl Sub<$rhs> for i64 {
            type Output = LinearExpr;
            fn sub(self, rhs: $rhs) -> LinearExpr { LinearExpr::from(self) - rhs }
        }
        impl Mul<$rhs> for i64 {
            type Output = LinearExpr;
            fn mul(self, rhs: $rhs) -> LinearExpr { LinearExpr::from(rhs) * self }
        }
    )+};
}
impl_i64_lhs_ops!(IntVar, BoolVar, LinearExpr);

#[cfg(test)]
mod tests {
    use super::*;

    fn int_var(index: i32) -> IntVar {
        IntVar { model: 0, index }
    }

    fn bool_var(index: i32) -> BoolVar {
        BoolVar { model: 0, index }
    }

    #[test]
    fn merges_duplicate_terms_and_drops_zeros() {
        let x = int_var(0);
        let y = int_var(1);
        let (vars, coeffs, constant) = (2 * x + y - x - y + 7).into_parts();
        assert_eq!(vars, vec![0]);
        assert_eq!(coeffs, vec![1]);
        assert_eq!(constant, 7);
    }

    #[test]
    fn negated_literal_expands_to_one_minus_var() {
        let b = bool_var(3);
        let (vars, coeffs, constant) = LinearExpr::from(b.not()).into_parts();
        assert_eq!(vars, vec![3]);
        assert_eq!(coeffs, vec![-1]);
        assert_eq!(constant, 1);
    }

    #[test]
    fn literal_negation_is_involutive() {
        let b = bool_var(5);
        assert_eq!(b.not().not(), b);
        assert_eq!(b.not().literal_index(), -6);
    }

    #[test]
    fn i64_lhs_operators_build_expressions() {
        let x = int_var(2);
        let (vars, coeffs, constant) = (10 - 3 * x).into_parts();
        assert_eq!(vars, vec![2]);
        assert_eq!(coeffs, vec![-3]);
        assert_eq!(constant, 10);
    }

    #[test]
    fn sum_accepts_mixed_items() {
        let x = int_var(0);
        let b = bool_var(1);
        let total = LinearExpr::sum([LinearExpr::from(x), b.into(), 4.into()]);
        let (vars, coeffs, constant) = total.into_parts();
        assert_eq!(vars, vec![0, 1]);
        assert_eq!(coeffs, vec![1, 1]);
        assert_eq!(constant, 4);
    }

    #[test]
    fn constants_carry_no_model_identity() {
        let expr = LinearExpr::from(7) + int_var(0);
        assert_eq!(expr.model, Some(0));
    }

    #[test]
    #[should_panic(expected = "two different models")]
    fn mixing_models_in_an_expression_panics() {
        let x = IntVar { model: 0, index: 0 };
        let y = IntVar { model: 1, index: 0 };
        let _ = x + y;
    }
}
