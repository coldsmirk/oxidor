use std::collections::BTreeMap;
use std::iter::Sum;
use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

/// A decision variable of a [`Model`](crate::Model).
///
/// A cheap copyable handle; the variable's state lives in the model. A handle
/// is only meaningful to the model that created it — using it with another
/// model is a programmer error and panics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Variable {
    pub(crate) model: u32,
    pub(crate) id: i64,
}

/// A linear expression: a sum of variables with `f64` coefficients, plus a
/// constant.
///
/// Built with ordinary operators from [`Variable`] and numbers:
///
/// ```
/// use oxidor_mathopt::Model;
///
/// let mut model = Model::new();
/// let x = model.new_continuous_variable(0.0..=10.0);
/// let y = model.new_continuous_variable(0.0..=10.0);
///
/// let expr = 2.5 * x - y + 1.0;
/// # let _ = expr;
/// ```
///
/// # Panics
///
/// Combining variables of two different models in one expression is a
/// programmer error and panics.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct LinearExpr {
    /// The model the variables in `terms` belong to; `None` for a
    /// constant-only expression.
    pub(crate) model: Option<u32>,
    pub(crate) terms: Vec<(i64, f64)>,
    pub(crate) constant: f64,
}

impl LinearExpr {
    /// The sum of the given items (variables, expressions, constants).
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

    /// Collapses to `(ids, coefficients, constant)` with duplicate variables
    /// merged, exact-zero coefficients dropped, and ids sorted — the order
    /// MathOpt's sparse containers require.
    pub(crate) fn into_parts(self) -> (Vec<i64>, Vec<f64>, f64) {
        let mut merged: BTreeMap<i64, f64> = BTreeMap::new();
        for (id, coefficient) in self.terms {
            *merged.entry(id).or_default() += coefficient;
        }
        merged.retain(|_, coefficient| *coefficient != 0.0);
        let (ids, coefficients) = merged.into_iter().unzip();
        (ids, coefficients, self.constant)
    }
}

impl From<Variable> for LinearExpr {
    fn from(variable: Variable) -> Self {
        Self {
            model: Some(variable.model),
            terms: vec![(variable.id, 1.0)],
            constant: 0.0,
        }
    }
}

impl From<f64> for LinearExpr {
    fn from(constant: f64) -> Self {
        Self {
            model: None,
            terms: Vec::new(),
            constant,
        }
    }
}

impl From<i64> for LinearExpr {
    fn from(constant: i64) -> Self {
        Self::from(constant as f64)
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
        self.terms.extend(
            rhs.terms
                .into_iter()
                .map(|(id, coefficient)| (id, -coefficient)),
        );
        self.constant -= rhs.constant;
    }
}

impl Neg for LinearExpr {
    type Output = LinearExpr;
    fn neg(self) -> LinearExpr {
        self * -1.0
    }
}

impl<T: Into<LinearExpr>> Sum<T> for LinearExpr {
    fn sum<I: Iterator<Item = T>>(iter: I) -> Self {
        Self::sum(iter)
    }
}

/// Scaling by a scalar type, for expressions and variable handles alike.
macro_rules! impl_scalar_mul_div {
    ($($scalar:ty),+) => {$(
        impl Mul<$scalar> for LinearExpr {
            type Output = LinearExpr;
            fn mul(mut self, factor: $scalar) -> LinearExpr {
                let factor = factor as f64;
                for (_, coefficient) in &mut self.terms {
                    *coefficient *= factor;
                }
                self.constant *= factor;
                self
            }
        }
        impl Mul<$scalar> for Variable {
            type Output = LinearExpr;
            fn mul(self, factor: $scalar) -> LinearExpr { LinearExpr::from(self) * factor }
        }
        impl Mul<Variable> for $scalar {
            type Output = LinearExpr;
            fn mul(self, variable: Variable) -> LinearExpr { LinearExpr::from(variable) * self }
        }
        impl Mul<LinearExpr> for $scalar {
            type Output = LinearExpr;
            fn mul(self, expr: LinearExpr) -> LinearExpr { expr * self }
        }
        impl Div<$scalar> for LinearExpr {
            type Output = LinearExpr;
            fn div(self, divisor: $scalar) -> LinearExpr { self * (1.0 / divisor as f64) }
        }
        impl Div<$scalar> for Variable {
            type Output = LinearExpr;
            fn div(self, divisor: $scalar) -> LinearExpr { LinearExpr::from(self) / divisor }
        }
    )+};
}
impl_scalar_mul_div!(f64, i64);

impl<R: Into<LinearExpr>> Add<R> for Variable {
    type Output = LinearExpr;
    fn add(self, rhs: R) -> LinearExpr {
        LinearExpr::from(self) + rhs
    }
}

impl<R: Into<LinearExpr>> Sub<R> for Variable {
    type Output = LinearExpr;
    fn sub(self, rhs: R) -> LinearExpr {
        LinearExpr::from(self) - rhs
    }
}

impl Neg for Variable {
    type Output = LinearExpr;
    fn neg(self) -> LinearExpr {
        -LinearExpr::from(self)
    }
}

/// Scalar-on-the-left addition and subtraction. Coherence requires one impl
/// per concrete right-hand type.
macro_rules! impl_scalar_lhs_add_sub {
    ($($scalar:ty),+) => {$(
        impl Add<Variable> for $scalar {
            type Output = LinearExpr;
            fn add(self, rhs: Variable) -> LinearExpr { LinearExpr::from(self as f64) + rhs }
        }
        impl Add<LinearExpr> for $scalar {
            type Output = LinearExpr;
            fn add(self, rhs: LinearExpr) -> LinearExpr { LinearExpr::from(self as f64) + rhs }
        }
        impl Sub<Variable> for $scalar {
            type Output = LinearExpr;
            fn sub(self, rhs: Variable) -> LinearExpr { LinearExpr::from(self as f64) - rhs }
        }
        impl Sub<LinearExpr> for $scalar {
            type Output = LinearExpr;
            fn sub(self, rhs: LinearExpr) -> LinearExpr { LinearExpr::from(self as f64) - rhs }
        }
    )+};
}
impl_scalar_lhs_add_sub!(f64, i64);

#[cfg(test)]
mod tests {
    use super::*;

    fn variable(id: i64) -> Variable {
        Variable { model: 0, id }
    }

    #[test]
    fn merges_duplicate_terms_and_sorts_ids() {
        let x = variable(3);
        let y = variable(1);
        let (ids, coefficients, constant) = (2.0 * x + y + 0.5 * x - 4.0).into_parts();
        assert_eq!(ids, vec![1, 3]);
        assert_eq!(coefficients, vec![1.0, 2.5]);
        assert_eq!(constant, -4.0);
    }

    #[test]
    fn drops_exactly_cancelled_terms() {
        let x = variable(0);
        let (ids, coefficients, _) = (x - x).into_parts();
        assert!(ids.is_empty());
        assert!(coefficients.is_empty());
    }

    #[test]
    fn integer_scalars_participate() {
        let x = variable(0);
        let (ids, coefficients, constant) = (2 * x + 1).into_parts();
        assert_eq!(ids, vec![0]);
        assert_eq!(coefficients, vec![2.0]);
        assert_eq!(constant, 1.0);
    }

    #[test]
    fn division_scales_by_the_reciprocal() {
        let x = variable(0);
        let (ids, coefficients, constant) = ((x + 1.0) / 2.0).into_parts();
        assert_eq!(ids, vec![0]);
        assert_eq!(coefficients, vec![0.5]);
        assert_eq!(constant, 0.5);
    }

    #[test]
    #[should_panic(expected = "two different models")]
    fn mixing_models_in_an_expression_panics() {
        let x = Variable { model: 0, id: 0 };
        let y = Variable { model: 1, id: 0 };
        let _ = x + y;
    }
}
