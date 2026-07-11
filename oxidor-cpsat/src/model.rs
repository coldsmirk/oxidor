use oxidor_protos::sat::{
    AllDifferentConstraintProto, BoolArgumentProto, ConstraintProto, CpModelProto,
    CpObjectiveProto, CumulativeConstraintProto, IntegerVariableProto, IntervalConstraintProto,
    LinearConstraintProto, LinearExpressionProto, NoOverlapConstraintProto, constraint_proto,
};

use crate::domain::Domain;
use crate::expr::{BoolVar, IntVar, LinearExpr};

/// An interval variable of a [`CpModelBuilder`]: a named span
/// `[start, start + size)` used by scheduling constraints such as
/// [`add_no_overlap`](CpModelBuilder::add_no_overlap).
///
/// A cheap copyable handle; the interval's state lives in the model.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct IntervalVar(pub(crate) i32);

/// Builds a CP-SAT model: variables, constraints, and an optional objective.
///
/// This is a thin, allocation-conscious layer over the wire format
/// (`CpModelProto`): every method appends directly to the proto, and
/// [`solve`](CpModelBuilder::solve) hands the serialized bytes to the native
/// solver.
///
/// ```no_run
/// use oxidor_cpsat::CpModelBuilder;
///
/// let mut model = CpModelBuilder::new();
/// let x = model.new_int_var(0..=10);
/// let y = model.new_int_var(0..=10);
/// model.add_less_or_equal(x + y, 14);
/// model.maximize(2 * x + 3 * y);
///
/// let response = model.solve();
/// let solution = response.solution().expect("model is feasible");
/// assert_eq!(solution.value(2 * x + 3 * y), 38);
/// ```
#[derive(Debug, Clone, Default)]
pub struct CpModelBuilder {
    proto: CpModelProto,
}

impl CpModelBuilder {
    /// An empty model.
    pub fn new() -> Self {
        Self::default()
    }

    /// A new integer variable taking values in `domain`.
    ///
    /// ```
    /// # let mut model = oxidor_cpsat::CpModelBuilder::new();
    /// let x = model.new_int_var(0..=23);
    /// let y = model.new_int_var(oxidor_cpsat::Domain::from_values([2, 4, 8]));
    /// ```
    pub fn new_int_var(&mut self, domain: impl Into<Domain>) -> IntVar {
        IntVar(self.append_variable(domain.into().flattened(), String::new()))
    }

    /// Like [`new_int_var`](Self::new_int_var), with a name for logs and
    /// debugging output.
    pub fn new_int_var_named(
        &mut self,
        domain: impl Into<Domain>,
        name: impl Into<String>,
    ) -> IntVar {
        IntVar(self.append_variable(domain.into().flattened(), name.into()))
    }

    /// A new Boolean variable.
    pub fn new_bool_var(&mut self) -> BoolVar {
        BoolVar(self.append_variable(vec![0, 1], String::new()))
    }

    /// Like [`new_bool_var`](Self::new_bool_var), with a name for logs and
    /// debugging output.
    pub fn new_bool_var_named(&mut self, name: impl Into<String>) -> BoolVar {
        BoolVar(self.append_variable(vec![0, 1], name.into()))
    }

    /// An integer variable fixed to `value`.
    pub fn new_constant(&mut self, value: i64) -> IntVar {
        IntVar(self.append_variable(vec![value, value], String::new()))
    }

    /// Constrains `expr` to take a value inside `domain`.
    pub fn add_linear_constraint(
        &mut self,
        expr: impl Into<LinearExpr>,
        domain: impl Into<Domain>,
    ) -> Constraint<'_> {
        let (vars, coeffs, constant) = expr.into().into_parts();
        let linear = LinearConstraintProto {
            vars,
            coeffs,
            // The proto has no constant term; fold it into the domain.
            domain: domain.into().flattened_shifted(constant),
        };
        self.append_constraint(constraint_proto::Constraint::Linear(linear))
    }

    /// Constrains `left == right`.
    pub fn add_equality(
        &mut self,
        left: impl Into<LinearExpr>,
        right: impl Into<LinearExpr>,
    ) -> Constraint<'_> {
        self.add_linear_constraint(left.into() - right, Domain::new(0, 0))
    }

    /// Constrains `left <= right`.
    pub fn add_less_or_equal(
        &mut self,
        left: impl Into<LinearExpr>,
        right: impl Into<LinearExpr>,
    ) -> Constraint<'_> {
        self.add_linear_constraint(left.into() - right, Domain::new(i64::MIN, 0))
    }

    /// Constrains `left >= right`.
    pub fn add_greater_or_equal(
        &mut self,
        left: impl Into<LinearExpr>,
        right: impl Into<LinearExpr>,
    ) -> Constraint<'_> {
        self.add_linear_constraint(left.into() - right, Domain::new(0, i64::MAX))
    }

    /// Constrains `left != right`.
    pub fn add_not_equal(
        &mut self,
        left: impl Into<LinearExpr>,
        right: impl Into<LinearExpr>,
    ) -> Constraint<'_> {
        self.add_linear_constraint(
            left.into() - right,
            Domain::from_intervals([(i64::MIN, -1), (1, i64::MAX)]),
        )
    }

    /// Constrains all expressions to take pairwise distinct values.
    pub fn add_all_different<T: Into<LinearExpr>>(
        &mut self,
        exprs: impl IntoIterator<Item = T>,
    ) -> Constraint<'_> {
        let all_diff = AllDifferentConstraintProto {
            exprs: exprs
                .into_iter()
                .map(|e| linear_expression_proto(e.into()))
                .collect(),
        };
        self.append_constraint(constraint_proto::Constraint::AllDiff(all_diff))
    }

    /// Constrains at least one literal to be true.
    pub fn add_bool_or(&mut self, literals: impl IntoIterator<Item = BoolVar>) -> Constraint<'_> {
        let arg = bool_argument_proto(literals);
        self.append_constraint(constraint_proto::Constraint::BoolOr(arg))
    }

    /// Constrains all literals to be true.
    pub fn add_bool_and(&mut self, literals: impl IntoIterator<Item = BoolVar>) -> Constraint<'_> {
        let arg = bool_argument_proto(literals);
        self.append_constraint(constraint_proto::Constraint::BoolAnd(arg))
    }

    /// Constrains at most one literal to be true.
    pub fn add_at_most_one(
        &mut self,
        literals: impl IntoIterator<Item = BoolVar>,
    ) -> Constraint<'_> {
        let arg = bool_argument_proto(literals);
        self.append_constraint(constraint_proto::Constraint::AtMostOne(arg))
    }

    /// Constrains exactly one literal to be true.
    pub fn add_exactly_one(
        &mut self,
        literals: impl IntoIterator<Item = BoolVar>,
    ) -> Constraint<'_> {
        let arg = bool_argument_proto(literals);
        self.append_constraint(constraint_proto::Constraint::ExactlyOne(arg))
    }

    /// Constrains `condition => consequence`.
    pub fn add_implication(&mut self, condition: BoolVar, consequence: BoolVar) -> Constraint<'_> {
        self.add_bool_or([condition.not(), consequence])
    }

    /// A new interval variable spanning `[start, start + size)` with
    /// `start + size == end` enforced.
    pub fn new_interval_var(
        &mut self,
        start: impl Into<LinearExpr>,
        size: impl Into<LinearExpr>,
        end: impl Into<LinearExpr>,
    ) -> IntervalVar {
        self.append_interval(start, size, end, None)
    }

    /// Like [`new_interval_var`](Self::new_interval_var), but the interval
    /// only exists — and only constrains others — when `presence` is true.
    pub fn new_optional_interval_var(
        &mut self,
        start: impl Into<LinearExpr>,
        size: impl Into<LinearExpr>,
        end: impl Into<LinearExpr>,
        presence: BoolVar,
    ) -> IntervalVar {
        self.append_interval(start, size, end, Some(presence))
    }

    /// Constrains the intervals to be pairwise disjoint (a single-resource
    /// scheduling constraint).
    pub fn add_no_overlap(
        &mut self,
        intervals: impl IntoIterator<Item = IntervalVar>,
    ) -> Constraint<'_> {
        let no_overlap = NoOverlapConstraintProto {
            intervals: intervals.into_iter().map(|interval| interval.0).collect(),
        };
        self.append_constraint(constraint_proto::Constraint::NoOverlap(no_overlap))
    }

    /// Constrains the sum of demands of the intervals overlapping any point in
    /// time to stay within `capacity`.
    ///
    /// # Panics
    ///
    /// Panics if `intervals` and `demands` have different lengths.
    pub fn add_cumulative(
        &mut self,
        capacity: i64,
        intervals: &[IntervalVar],
        demands: &[i64],
    ) -> Constraint<'_> {
        assert_eq!(
            intervals.len(),
            demands.len(),
            "add_cumulative needs one demand per interval",
        );
        let cumulative = CumulativeConstraintProto {
            capacity: Some(linear_expression_proto(capacity.into())),
            intervals: intervals.iter().map(|interval| interval.0).collect(),
            demands: demands
                .iter()
                .map(|&demand| linear_expression_proto(demand.into()))
                .collect(),
        };
        self.append_constraint(constraint_proto::Constraint::Cumulative(cumulative))
    }

    /// Sets the objective to minimizing `expr`, replacing any previous
    /// objective.
    pub fn minimize(&mut self, expr: impl Into<LinearExpr>) {
        let (vars, coeffs, constant) = expr.into().into_parts();
        self.proto.objective = Some(CpObjectiveProto {
            vars,
            coeffs,
            offset: constant as f64,
            ..Default::default()
        });
    }

    /// Sets the objective to maximizing `expr`, replacing any previous
    /// objective.
    pub fn maximize(&mut self, expr: impl Into<LinearExpr>) {
        // The proto always minimizes; maximization is encoded by negating the
        // terms and flagging the sign flip in scaling_factor.
        let (vars, coeffs, constant) = expr.into().into_parts();
        self.proto.objective = Some(CpObjectiveProto {
            vars,
            coeffs: coeffs.into_iter().map(|coeff| -coeff).collect(),
            offset: -(constant as f64),
            scaling_factor: -1.0,
            ..Default::default()
        });
    }

    /// The model as its wire representation.
    pub fn proto(&self) -> &CpModelProto {
        &self.proto
    }

    /// Consumes the builder, returning the wire representation.
    pub fn into_proto(self) -> CpModelProto {
        self.proto
    }

    fn append_variable(&mut self, domain: Vec<i64>, name: String) -> i32 {
        let index = self.proto.variables.len() as i32;
        self.proto
            .variables
            .push(IntegerVariableProto { name, domain });
        index
    }

    fn append_interval(
        &mut self,
        start: impl Into<LinearExpr>,
        size: impl Into<LinearExpr>,
        end: impl Into<LinearExpr>,
        presence: Option<BoolVar>,
    ) -> IntervalVar {
        let interval = IntervalConstraintProto {
            start: Some(linear_expression_proto(start.into())),
            size: Some(linear_expression_proto(size.into())),
            end: Some(linear_expression_proto(end.into())),
        };
        let index = self.proto.constraints.len() as i32;
        let constraint = self.append_constraint(constraint_proto::Constraint::Interval(interval));
        if let Some(presence) = presence {
            constraint.only_enforce_if([presence]);
        }
        IntervalVar(index)
    }

    fn append_constraint(&mut self, constraint: constraint_proto::Constraint) -> Constraint<'_> {
        self.proto.constraints.push(ConstraintProto {
            constraint: Some(constraint),
            ..Default::default()
        });
        Constraint(self.proto.constraints.last_mut().expect("just pushed"))
    }
}

/// A constraint freshly added to a [`CpModelBuilder`], open for refinement.
/// Dropping it unchanged is the common case, not a mistake.
pub struct Constraint<'model>(&'model mut ConstraintProto);

impl Constraint<'_> {
    /// Makes the constraint apply only when every literal is true
    /// (half-reified). Callable multiple times; literals accumulate.
    pub fn only_enforce_if(self, literals: impl IntoIterator<Item = BoolVar>) -> Self {
        self.0
            .enforcement_literal
            .extend(literals.into_iter().map(BoolVar::literal_index));
        self
    }

    /// Names the constraint for logs and debugging output.
    pub fn with_name(self, name: impl Into<String>) -> Self {
        self.0.name = name.into();
        self
    }
}

fn linear_expression_proto(expr: LinearExpr) -> LinearExpressionProto {
    let (vars, coeffs, offset) = expr.into_parts();
    LinearExpressionProto {
        vars,
        coeffs,
        offset,
    }
}

fn bool_argument_proto(literals: impl IntoIterator<Item = BoolVar>) -> BoolArgumentProto {
    BoolArgumentProto {
        literals: literals.into_iter().map(BoolVar::literal_index).collect(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn linear_constraint_folds_constant_into_domain() {
        let mut model = CpModelBuilder::new();
        let x = model.new_int_var(0..=10);
        model.add_less_or_equal(x + 4, 7);

        let constraint = &model.proto().constraints[0];
        let Some(constraint_proto::Constraint::Linear(linear)) = &constraint.constraint else {
            panic!("expected a linear constraint");
        };
        // x + 4 - 7 <= 0  ⇒  x in [MIN, 3].
        assert_eq!(linear.vars, vec![0]);
        assert_eq!(linear.coeffs, vec![1]);
        assert_eq!(linear.domain, vec![i64::MIN, 3]);
    }

    #[test]
    fn exactly_one_encodes_negated_literals() {
        let mut model = CpModelBuilder::new();
        let a = model.new_bool_var();
        let b = model.new_bool_var();
        model.add_exactly_one([a, b.not()]);

        let Some(constraint_proto::Constraint::ExactlyOne(arg)) =
            &model.proto().constraints[0].constraint
        else {
            panic!("expected an exactly_one constraint");
        };
        assert_eq!(arg.literals, vec![0, -2]);
    }

    #[test]
    fn only_enforce_if_accumulates_enforcement_literals() {
        let mut model = CpModelBuilder::new();
        let a = model.new_bool_var();
        let b = model.new_bool_var();
        let x = model.new_int_var(0..=5);
        model
            .add_equality(x, 3)
            .only_enforce_if([a])
            .only_enforce_if([b.not()]);

        assert_eq!(
            model.proto().constraints[0].enforcement_literal,
            vec![0, -2]
        );
    }

    #[test]
    fn maximize_negates_terms_and_flags_scaling() {
        let mut model = CpModelBuilder::new();
        let x = model.new_int_var(0..=10);
        model.maximize(2 * x + 5);

        let objective = model.proto().objective.as_ref().expect("objective set");
        assert_eq!(objective.coeffs, vec![-2]);
        assert_eq!(objective.offset, -5.0);
        assert_eq!(objective.scaling_factor, -1.0);
    }

    #[test]
    fn optional_interval_carries_presence_literal() {
        let mut model = CpModelBuilder::new();
        let presence = model.new_bool_var();
        let start = model.new_int_var(0..=10);
        let interval = model.new_optional_interval_var(start, 3, start + 3, presence);

        assert_eq!(interval.0, 0);
        assert_eq!(model.proto().constraints[0].enforcement_literal, vec![0]);
    }

    #[test]
    fn no_overlap_references_interval_indices() {
        let mut model = CpModelBuilder::new();
        let s0 = model.new_int_var(0..=10);
        let s1 = model.new_int_var(0..=10);
        let i0 = model.new_interval_var(s0, 2, s0 + 2);
        let i1 = model.new_interval_var(s1, 3, s1 + 3);
        model.add_no_overlap([i0, i1]);

        let Some(constraint_proto::Constraint::NoOverlap(no_overlap)) =
            &model.proto().constraints[2].constraint
        else {
            panic!("expected a no_overlap constraint");
        };
        assert_eq!(no_overlap.intervals, vec![0, 1]);
    }
}
