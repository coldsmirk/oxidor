use std::ops::RangeInclusive;
use std::sync::atomic::{AtomicU32, Ordering};

use oxidor_protos::math_opt::{ModelProto, ObjectiveProto, SparseDoubleVectorProto};

use crate::expr::{LinearExpr, Variable};

/// Source of the process-unique identity every model stamps on its handles.
static NEXT_MODEL_ID: AtomicU32 = AtomicU32::new(0);

/// A linear constraint of a [`Model`], as a cheap copyable handle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct LinearConstraint(pub(crate) i64);

/// Builds a MathOpt optimization model: variables (continuous or integer),
/// linear constraints, and an objective — an LP or MIP depending on what you
/// add.
///
/// This is a thin layer over the wire format (`ModelProto`): every method
/// appends directly to the proto, and [`solve`](Model::solve) hands the
/// serialized bytes to the solver picked per call.
///
/// ```no_run
/// use oxidor_mathopt::{Model, SolverType};
///
/// let mut model = Model::new();
/// let x = model.new_continuous_variable(0.0..=10.0);
/// let y = model.new_continuous_variable(0.0..=10.0);
/// model.add_less_or_equal(x + y, 14.0);
/// model.maximize(2.0 * x + 3.0 * y);
///
/// let result = model.solve(SolverType::Glop).expect("solver available");
/// let solution = result.primal_solution().expect("optimal");
/// assert_eq!(solution.value(x), 4.0);
/// ```
///
/// # Handle identity
///
/// Variable handles are only meaningful to the model that created them.
/// Constraint and objective methods check this and **panic** when handed a
/// handle from a different model — a programmer error that would otherwise
/// silently encode the wrong model. Cloning a model preserves its identity:
/// handles created before the clone are valid for both copies.
#[derive(Debug, Clone)]
pub struct Model {
    id: u32,
    proto: ModelProto,
}

impl Default for Model {
    fn default() -> Self {
        Self::new()
    }
}

impl Model {
    /// An empty model.
    pub fn new() -> Self {
        Self {
            id: NEXT_MODEL_ID.fetch_add(1, Ordering::Relaxed),
            proto: ModelProto::default(),
        }
    }

    /// A new continuous variable within `bounds` (use
    /// `f64::NEG_INFINITY..=f64::INFINITY` for a free variable).
    pub fn new_continuous_variable(&mut self, bounds: RangeInclusive<f64>) -> Variable {
        self.append_variable(bounds, false, String::new())
    }

    /// Like [`new_continuous_variable`](Self::new_continuous_variable), with
    /// a name for logs and debugging output.
    pub fn new_continuous_variable_named(
        &mut self,
        bounds: RangeInclusive<f64>,
        name: impl Into<String>,
    ) -> Variable {
        self.append_variable(bounds, false, name.into())
    }

    /// A new integer variable within `bounds`.
    pub fn new_integer_variable(&mut self, bounds: RangeInclusive<f64>) -> Variable {
        self.append_variable(bounds, true, String::new())
    }

    /// Like [`new_integer_variable`](Self::new_integer_variable), with a name
    /// for logs and debugging output.
    pub fn new_integer_variable_named(
        &mut self,
        bounds: RangeInclusive<f64>,
        name: impl Into<String>,
    ) -> Variable {
        self.append_variable(bounds, true, name.into())
    }

    /// Constrains `expr` to stay within `bounds`.
    pub fn add_linear_constraint(
        &mut self,
        expr: impl Into<LinearExpr>,
        bounds: RangeInclusive<f64>,
    ) -> LinearConstraint {
        let (ids, coefficients, constant) = self.owned_expr(expr).into_parts();

        let constraints = self.proto.linear_constraints.get_or_insert_default();
        let row_id = constraints.ids.len() as i64;
        constraints.ids.push(row_id);
        // The proto has no constant term; fold it into the bounds. IEEE
        // arithmetic keeps ±infinity intact.
        constraints.lower_bounds.push(bounds.start() - constant);
        constraints.upper_bounds.push(bounds.end() - constant);
        constraints.names.push(String::new());

        let matrix = self.proto.linear_constraint_matrix.get_or_insert_default();
        // Appending whole rows in id order with sorted column ids keeps the
        // matrix in the required row-major sorted order.
        for (column_id, coefficient) in ids.into_iter().zip(coefficients) {
            matrix.row_ids.push(row_id);
            matrix.column_ids.push(column_id);
            matrix.coefficients.push(coefficient);
        }

        LinearConstraint(row_id)
    }

    /// Constrains `left == right`.
    pub fn add_equality(
        &mut self,
        left: impl Into<LinearExpr>,
        right: impl Into<LinearExpr>,
    ) -> LinearConstraint {
        self.add_linear_constraint(left.into() - right, 0.0..=0.0)
    }

    /// Constrains `left <= right`.
    pub fn add_less_or_equal(
        &mut self,
        left: impl Into<LinearExpr>,
        right: impl Into<LinearExpr>,
    ) -> LinearConstraint {
        self.add_linear_constraint(left.into() - right, f64::NEG_INFINITY..=0.0)
    }

    /// Constrains `left >= right`.
    pub fn add_greater_or_equal(
        &mut self,
        left: impl Into<LinearExpr>,
        right: impl Into<LinearExpr>,
    ) -> LinearConstraint {
        self.add_linear_constraint(left.into() - right, 0.0..=f64::INFINITY)
    }

    /// Sets the objective to minimizing `expr`, replacing any previous
    /// objective.
    pub fn minimize(&mut self, expr: impl Into<LinearExpr>) {
        let expr = self.owned_expr(expr);
        self.set_objective(expr, false);
    }

    /// Sets the objective to maximizing `expr`, replacing any previous
    /// objective.
    pub fn maximize(&mut self, expr: impl Into<LinearExpr>) {
        let expr = self.owned_expr(expr);
        self.set_objective(expr, true);
    }

    /// The model as its wire representation.
    pub fn proto(&self) -> &ModelProto {
        &self.proto
    }

    /// Consumes the builder, returning the wire representation.
    pub fn into_proto(self) -> ModelProto {
        self.proto
    }

    #[cfg(feature = "solve")]
    pub(crate) fn id(&self) -> u32 {
        self.id
    }

    /// Converts to an expression, panicking on a handle from another model.
    #[track_caller]
    fn owned_expr(&self, expr: impl Into<LinearExpr>) -> LinearExpr {
        let expr = expr.into();
        assert!(
            expr.model.is_none_or(|model| model == self.id),
            "the expression uses variables from a different Model",
        );
        expr
    }

    fn append_variable(
        &mut self,
        bounds: RangeInclusive<f64>,
        integer: bool,
        name: String,
    ) -> Variable {
        let variables = self.proto.variables.get_or_insert_default();
        let id = variables.ids.len() as i64;
        variables.ids.push(id);
        variables.lower_bounds.push(*bounds.start());
        variables.upper_bounds.push(*bounds.end());
        variables.integers.push(integer);
        variables.names.push(name);
        Variable { model: self.id, id }
    }

    fn set_objective(&mut self, expr: LinearExpr, maximize: bool) {
        let (ids, values, constant) = expr.into_parts();
        self.proto.objective = Some(ObjectiveProto {
            maximize,
            offset: constant,
            linear_coefficients: Some(SparseDoubleVectorProto { ids, values }),
            ..Default::default()
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn variables_get_dense_sorted_ids() {
        let mut model = Model::new();
        let x = model.new_continuous_variable(0.0..=1.0);
        let y = model.new_integer_variable(-5.0..=5.0);
        assert_eq!((x.id, y.id), (0, 1));

        let variables = model.proto().variables.as_ref().expect("variables set");
        assert_eq!(variables.ids, vec![0, 1]);
        assert_eq!(variables.integers, vec![false, true]);
    }

    #[test]
    fn constraint_folds_constant_and_keeps_matrix_sorted() {
        let mut model = Model::new();
        let x = model.new_continuous_variable(0.0..=10.0);
        let y = model.new_continuous_variable(0.0..=10.0);
        model.add_less_or_equal(y + 2.0 * x + 5.0, 9.0);

        let constraints = model.proto().linear_constraints.as_ref().expect("set");
        assert_eq!(constraints.lower_bounds, vec![f64::NEG_INFINITY]);
        assert_eq!(constraints.upper_bounds, vec![4.0]);

        let matrix = model
            .proto()
            .linear_constraint_matrix
            .as_ref()
            .expect("set");
        assert_eq!(matrix.row_ids, vec![0, 0]);
        assert_eq!(matrix.column_ids, vec![0, 1]);
        assert_eq!(matrix.coefficients, vec![2.0, 1.0]);
    }

    #[test]
    fn maximize_is_a_flag_not_a_negation() {
        let mut model = Model::new();
        let x = model.new_continuous_variable(0.0..=1.0);
        model.maximize(3.0 * x + 1.0);

        let objective = model.proto().objective.as_ref().expect("set");
        assert!(objective.maximize);
        assert_eq!(objective.offset, 1.0);
        let coefficients = objective.linear_coefficients.as_ref().expect("set");
        assert_eq!(coefficients.values, vec![3.0]);
    }

    #[test]
    fn equality_folds_the_right_hand_side() {
        let mut model = Model::new();
        let x = model.new_continuous_variable(0.0..=10.0);
        model.add_equality(x + 2.0, 5.0);

        let constraints = model.proto().linear_constraints.as_ref().expect("set");
        assert_eq!(constraints.lower_bounds, vec![3.0]);
        assert_eq!(constraints.upper_bounds, vec![3.0]);
    }

    #[test]
    #[should_panic(expected = "different Model")]
    fn rejects_a_variable_from_another_model() {
        let mut model_a = Model::new();
        let mut model_b = Model::new();
        let x = model_a.new_continuous_variable(0.0..=1.0);
        model_b.maximize(2.0 * x);
    }
}
