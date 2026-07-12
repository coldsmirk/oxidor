use std::sync::atomic::{AtomicU32, Ordering};

use oxidor_protos::sat::{
    AllDifferentConstraintProto, AutomatonConstraintProto, BoolArgumentProto,
    CircuitConstraintProto, ConstraintProto, CpModelProto, CpObjectiveProto,
    CumulativeConstraintProto, ElementConstraintProto, IntegerVariableProto,
    IntervalConstraintProto, InverseConstraintProto, LinearArgumentProto, LinearConstraintProto,
    LinearExpressionProto, NoOverlap2DConstraintProto, NoOverlapConstraintProto,
    ReservoirConstraintProto, RoutesConstraintProto, TableConstraintProto, constraint_proto,
};

use crate::domain::Domain;
use crate::expr::{BoolVar, IntVar, LinearExpr};

/// Source of the process-unique identity every builder stamps on its handles.
static NEXT_MODEL_ID: AtomicU32 = AtomicU32::new(0);

/// An interval variable of a [`CpModelBuilder`]: a named span
/// `[start, start + size)` used by scheduling constraints such as
/// [`add_no_overlap`](CpModelBuilder::add_no_overlap).
///
/// A cheap copyable handle; the interval's state lives in the model.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct IntervalVar {
    pub(crate) model: u32,
    pub(crate) index: i32,
}

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
///
/// # Handle identity
///
/// Variable, literal, and interval handles are only meaningful to the builder
/// that created them. Every constraint and objective method checks this and
/// **panics** when handed a handle from a different builder — a programmer
/// error that would otherwise silently encode the wrong model. Cloning a
/// builder preserves its identity: handles created before the clone are valid
/// for both copies.
#[derive(Debug, Clone)]
pub struct CpModelBuilder {
    id: u32,
    proto: CpModelProto,
}

impl Default for CpModelBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl CpModelBuilder {
    /// An empty model.
    pub fn new() -> Self {
        Self {
            id: NEXT_MODEL_ID.fetch_add(1, Ordering::Relaxed),
            proto: CpModelProto::default(),
        }
    }

    /// A new integer variable taking values in `domain`.
    ///
    /// ```
    /// # let mut model = oxidor_cpsat::CpModelBuilder::new();
    /// let x = model.new_int_var(0..=23);
    /// let y = model.new_int_var(oxidor_cpsat::Domain::from_values([2, 4, 8]));
    /// ```
    pub fn new_int_var(&mut self, domain: impl Into<Domain>) -> IntVar {
        IntVar {
            model: self.id,
            index: self.append_variable(domain.into().flattened(), String::new()),
        }
    }

    /// Like [`new_int_var`](Self::new_int_var), with a name for logs and
    /// debugging output.
    pub fn new_int_var_named(
        &mut self,
        domain: impl Into<Domain>,
        name: impl Into<String>,
    ) -> IntVar {
        IntVar {
            model: self.id,
            index: self.append_variable(domain.into().flattened(), name.into()),
        }
    }

    /// A new Boolean variable.
    pub fn new_bool_var(&mut self) -> BoolVar {
        BoolVar {
            model: self.id,
            index: self.append_variable(vec![0, 1], String::new()),
        }
    }

    /// Like [`new_bool_var`](Self::new_bool_var), with a name for logs and
    /// debugging output.
    pub fn new_bool_var_named(&mut self, name: impl Into<String>) -> BoolVar {
        BoolVar {
            model: self.id,
            index: self.append_variable(vec![0, 1], name.into()),
        }
    }

    /// An integer variable fixed to `value`.
    pub fn new_constant(&mut self, value: i64) -> IntVar {
        IntVar {
            model: self.id,
            index: self.append_variable(vec![value, value], String::new()),
        }
    }

    /// Constrains `expr` to take a value inside `domain`.
    pub fn add_linear_constraint(
        &mut self,
        expr: impl Into<LinearExpr>,
        domain: impl Into<Domain>,
    ) -> Constraint<'_> {
        let (vars, coeffs, constant) = self.owned_expr(expr).into_parts();
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
        let exprs = exprs
            .into_iter()
            .map(|expr| linear_expression_proto(self.owned_expr(expr)))
            .collect();
        let all_diff = AllDifferentConstraintProto { exprs };
        self.append_constraint(constraint_proto::Constraint::AllDiff(all_diff))
    }

    /// Constrains `target` to equal the maximum of the expressions.
    ///
    /// The workhorse of fairness objectives: introduce a `target` variable,
    /// bind it to the maximum of the per-worker loads, and minimize it.
    pub fn add_max_equality<T: Into<LinearExpr>>(
        &mut self,
        target: impl Into<LinearExpr>,
        exprs: impl IntoIterator<Item = T>,
    ) -> Constraint<'_> {
        let argument = self.linear_argument_proto(target, exprs, false);
        self.append_constraint(constraint_proto::Constraint::LinMax(argument))
    }

    /// Constrains `target` to equal the minimum of the expressions.
    pub fn add_min_equality<T: Into<LinearExpr>>(
        &mut self,
        target: impl Into<LinearExpr>,
        exprs: impl IntoIterator<Item = T>,
    ) -> Constraint<'_> {
        // The proto only has lin_max; min(e…) == -max(-e…), matching the C++
        // CpModelBuilder's AddMinEquality.
        let argument = self.linear_argument_proto(target, exprs, true);
        self.append_constraint(constraint_proto::Constraint::LinMax(argument))
    }

    /// Constrains `target` to equal the product of the expressions (an empty
    /// product forces `target == 1`).
    ///
    /// The solver rejects models where the product can overflow an `i64` over
    /// the initial domains.
    pub fn add_multiplication_equality<T: Into<LinearExpr>>(
        &mut self,
        target: impl Into<LinearExpr>,
        exprs: impl IntoIterator<Item = T>,
    ) -> Constraint<'_> {
        let argument = self.linear_argument_proto(target, exprs, false);
        self.append_constraint(constraint_proto::Constraint::IntProd(argument))
    }

    /// Constrains `target == numerator / denominator`, rounding toward zero
    /// (`12 / 5 == 2`, `-10 / 3 == -3`).
    ///
    /// The solver rejects models where `denominator`'s domain contains 0.
    pub fn add_division_equality(
        &mut self,
        target: impl Into<LinearExpr>,
        numerator: impl Into<LinearExpr>,
        denominator: impl Into<LinearExpr>,
    ) -> Constraint<'_> {
        let argument =
            self.linear_argument_proto(target, [numerator.into(), denominator.into()], false);
        self.append_constraint(constraint_proto::Constraint::IntDiv(argument))
    }

    /// Constrains `target == expr % modulus`; the target takes the sign of
    /// `expr`.
    ///
    /// The solver rejects models where `modulus`'s domain is not strictly
    /// positive.
    pub fn add_modulo_equality(
        &mut self,
        target: impl Into<LinearExpr>,
        expr: impl Into<LinearExpr>,
        modulus: impl Into<LinearExpr>,
    ) -> Constraint<'_> {
        let argument = self.linear_argument_proto(target, [expr.into(), modulus.into()], false);
        self.append_constraint(constraint_proto::Constraint::IntMod(argument))
    }

    /// Constrains `target` to equal the absolute value of `expr`.
    pub fn add_abs_equality(
        &mut self,
        target: impl Into<LinearExpr>,
        expr: impl Into<LinearExpr>,
    ) -> Constraint<'_> {
        // Encoded as target == max(expr, -expr), like the C++ CpModelBuilder.
        let expr = self.owned_expr(expr);
        let argument = self.linear_argument_proto(target, [expr.clone(), -expr], false);
        self.append_constraint(constraint_proto::Constraint::LinMax(argument))
    }

    /// Constrains `target` to equal `exprs[index]`, which also restricts
    /// `index` to `0..exprs.len()`.
    ///
    /// The classic lookup constraint: the expressions may be constants (a
    /// table of values indexed by a variable) or variables themselves.
    pub fn add_element<T: Into<LinearExpr>>(
        &mut self,
        index: impl Into<LinearExpr>,
        exprs: impl IntoIterator<Item = T>,
        target: impl Into<LinearExpr>,
    ) -> Constraint<'_> {
        let element = ElementConstraintProto {
            linear_index: Some(linear_expression_proto(self.owned_expr(index))),
            linear_target: Some(linear_expression_proto(self.owned_expr(target))),
            exprs: exprs
                .into_iter()
                .map(|expr| linear_expression_proto(self.owned_expr(expr)))
                .collect(),
            ..Default::default()
        };
        self.append_constraint(constraint_proto::Constraint::Element(element))
    }

    /// Constrains the tuple of expressions to equal one of the listed tuples.
    ///
    /// ```
    /// # let mut model = oxidor_cpsat::CpModelBuilder::new();
    /// let x = model.new_int_var(0..=5);
    /// let y = model.new_int_var(0..=5);
    /// // (x, y) must be one of (1, 2) or (2, 3).
    /// model.add_allowed_assignments([x, y], [[1, 2], [2, 3]]);
    /// ```
    ///
    /// # Panics
    ///
    /// Panics if a tuple's length differs from the number of expressions.
    pub fn add_allowed_assignments<T: Into<LinearExpr>>(
        &mut self,
        exprs: impl IntoIterator<Item = T>,
        tuples: impl IntoIterator<Item = impl AsRef<[i64]>>,
    ) -> Constraint<'_> {
        let table = self.table_constraint_proto(exprs, tuples, false);
        self.append_constraint(constraint_proto::Constraint::Table(table))
    }

    /// Constrains the tuple of expressions to differ from every listed tuple.
    ///
    /// # Panics
    ///
    /// Panics if a tuple's length differs from the number of expressions.
    pub fn add_forbidden_assignments<T: Into<LinearExpr>>(
        &mut self,
        exprs: impl IntoIterator<Item = T>,
        tuples: impl IntoIterator<Item = impl AsRef<[i64]>>,
    ) -> Constraint<'_> {
        let table = self.table_constraint_proto(exprs, tuples, true);
        self.append_constraint(constraint_proto::Constraint::Table(table))
    }

    /// Constrains the two variable arrays to be inverse permutations of each
    /// other: `f_direct[i] == j ⇔ f_inverse[j] == i`.
    ///
    /// # Panics
    ///
    /// Panics if the arrays have different lengths.
    pub fn add_inverse(
        &mut self,
        f_direct: impl IntoIterator<Item = IntVar>,
        f_inverse: impl IntoIterator<Item = IntVar>,
    ) -> Constraint<'_> {
        let inverse = InverseConstraintProto {
            f_direct: f_direct
                .into_iter()
                .map(|var| self.owned_var(var))
                .collect(),
            f_inverse: f_inverse
                .into_iter()
                .map(|var| self.owned_var(var))
                .collect(),
        };
        assert_eq!(
            inverse.f_direct.len(),
            inverse.f_inverse.len(),
            "add_inverse needs arrays of equal length",
        );
        self.append_constraint(constraint_proto::Constraint::Inverse(inverse))
    }

    /// Constrains the selected arcs — each `(tail, head, literal)`, present
    /// when its literal is true — to form a single circuit visiting every
    /// node with an incident arc exactly once.
    ///
    /// Self-arcs `(n, n, literal)` are allowed and mean "node n is skipped"
    /// when the literal is true.
    pub fn add_circuit(
        &mut self,
        arcs: impl IntoIterator<Item = (i32, i32, BoolVar)>,
    ) -> Constraint<'_> {
        let (tails, heads, literals) = self.arc_lists(arcs);
        let circuit = CircuitConstraintProto {
            tails,
            heads,
            literals,
        };
        self.append_constraint(constraint_proto::Constraint::Circuit(circuit))
    }

    /// Constrains the selected arcs to form a set of routes, all starting and
    /// ending at node 0 (the depot) — the CP-SAT native vehicle-routing
    /// constraint.
    ///
    /// Every node other than 0 must have exactly one incoming and one
    /// outgoing selected arc (use a self-arc to make a node optional); node 0
    /// has one per route.
    pub fn add_multiple_circuit(
        &mut self,
        arcs: impl IntoIterator<Item = (i32, i32, BoolVar)>,
    ) -> Constraint<'_> {
        let (tails, heads, literals) = self.arc_lists(arcs);
        let routes = RoutesConstraintProto {
            tails,
            heads,
            literals,
            ..Default::default()
        };
        self.append_constraint(constraint_proto::Constraint::Routes(routes))
    }

    /// Constrains the sequence of expressions to be accepted by the
    /// finite-state automaton given as `(tail_state, head_state, label)`
    /// transitions: starting from `starting_state`, step `i` follows the
    /// transition whose label equals the value of the `i`-th expression, and
    /// the final state must be one of `final_states`.
    pub fn add_automaton<T: Into<LinearExpr>>(
        &mut self,
        exprs: impl IntoIterator<Item = T>,
        starting_state: i64,
        final_states: impl IntoIterator<Item = i64>,
        transitions: impl IntoIterator<Item = (i64, i64, i64)>,
    ) -> Constraint<'_> {
        let mut automaton = AutomatonConstraintProto {
            starting_state,
            final_states: final_states.into_iter().collect(),
            exprs: exprs
                .into_iter()
                .map(|expr| linear_expression_proto(self.owned_expr(expr)))
                .collect(),
            ..Default::default()
        };
        for (tail, head, label) in transitions {
            automaton.transition_tail.push(tail);
            automaton.transition_head.push(head);
            automaton.transition_label.push(label);
        }
        self.append_constraint(constraint_proto::Constraint::Automaton(automaton))
    }

    /// Constrains at least one literal to be true.
    pub fn add_bool_or(&mut self, literals: impl IntoIterator<Item = BoolVar>) -> Constraint<'_> {
        let arg = self.bool_argument_proto(literals);
        self.append_constraint(constraint_proto::Constraint::BoolOr(arg))
    }

    /// Constrains all literals to be true.
    pub fn add_bool_and(&mut self, literals: impl IntoIterator<Item = BoolVar>) -> Constraint<'_> {
        let arg = self.bool_argument_proto(literals);
        self.append_constraint(constraint_proto::Constraint::BoolAnd(arg))
    }

    /// Constrains at most one literal to be true.
    pub fn add_at_most_one(
        &mut self,
        literals: impl IntoIterator<Item = BoolVar>,
    ) -> Constraint<'_> {
        let arg = self.bool_argument_proto(literals);
        self.append_constraint(constraint_proto::Constraint::AtMostOne(arg))
    }

    /// Constrains exactly one literal to be true.
    pub fn add_exactly_one(
        &mut self,
        literals: impl IntoIterator<Item = BoolVar>,
    ) -> Constraint<'_> {
        let arg = self.bool_argument_proto(literals);
        self.append_constraint(constraint_proto::Constraint::ExactlyOne(arg))
    }

    /// Constrains an odd number of the literals to be true.
    pub fn add_bool_xor(&mut self, literals: impl IntoIterator<Item = BoolVar>) -> Constraint<'_> {
        let arg = self.bool_argument_proto(literals);
        self.append_constraint(constraint_proto::Constraint::BoolXor(arg))
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
        let intervals = intervals
            .into_iter()
            .map(|interval| self.owned_interval(interval))
            .collect();
        let no_overlap = NoOverlapConstraintProto { intervals };
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
            intervals: intervals
                .iter()
                .map(|&interval| self.owned_interval(interval))
                .collect(),
            demands: demands
                .iter()
                .map(|&demand| linear_expression_proto(demand.into()))
                .collect(),
        };
        self.append_constraint(constraint_proto::Constraint::Cumulative(cumulative))
    }

    /// Constrains the boxes — each spanning `x_interval × y_interval` — to be
    /// pairwise non-overlapping in the plane (rectangle packing).
    ///
    /// A box is optional when one of its intervals is optional.
    pub fn add_no_overlap_2d(
        &mut self,
        boxes: impl IntoIterator<Item = (IntervalVar, IntervalVar)>,
    ) -> Constraint<'_> {
        let mut no_overlap = NoOverlap2DConstraintProto::default();
        for (x_interval, y_interval) in boxes {
            no_overlap.x_intervals.push(self.owned_interval(x_interval));
            no_overlap.y_intervals.push(self.owned_interval(y_interval));
        }
        self.append_constraint(constraint_proto::Constraint::NoOverlap2d(no_overlap))
    }

    /// Constrains a running level to stay within `min_level..=max_level`
    /// (with `min_level <= 0 <= max_level`; the level starts at 0): each
    /// `(time, level_change)` event adds its change to the level at its time.
    ///
    /// Use a fixed event to model an initial stock.
    pub fn add_reservoir<T: Into<LinearExpr>, L: Into<LinearExpr>>(
        &mut self,
        min_level: i64,
        max_level: i64,
        events: impl IntoIterator<Item = (T, L)>,
    ) -> Constraint<'_> {
        let mut reservoir = ReservoirConstraintProto {
            min_level,
            max_level,
            ..Default::default()
        };
        for (time, level_change) in events {
            reservoir
                .time_exprs
                .push(linear_expression_proto(self.owned_expr(time)));
            reservoir
                .level_changes
                .push(linear_expression_proto(self.owned_expr(level_change)));
        }
        self.append_constraint(constraint_proto::Constraint::Reservoir(reservoir))
    }

    /// Like [`add_reservoir`](Self::add_reservoir), but each
    /// `(time, level_change, active)` event only affects the level when its
    /// `active` literal is true.
    pub fn add_reservoir_with_optional_events<T: Into<LinearExpr>, L: Into<LinearExpr>>(
        &mut self,
        min_level: i64,
        max_level: i64,
        events: impl IntoIterator<Item = (T, L, BoolVar)>,
    ) -> Constraint<'_> {
        let mut reservoir = ReservoirConstraintProto {
            min_level,
            max_level,
            ..Default::default()
        };
        for (time, level_change, active) in events {
            reservoir
                .time_exprs
                .push(linear_expression_proto(self.owned_expr(time)));
            reservoir
                .level_changes
                .push(linear_expression_proto(self.owned_expr(level_change)));
            reservoir.active_literals.push(self.owned_literal(active));
        }
        self.append_constraint(constraint_proto::Constraint::Reservoir(reservoir))
    }

    /// Sets the objective to minimizing `expr`, replacing any previous
    /// objective.
    pub fn minimize(&mut self, expr: impl Into<LinearExpr>) {
        let (vars, coeffs, constant) = self.owned_expr(expr).into_parts();
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
        let (vars, coeffs, constant) = self.owned_expr(expr).into_parts();
        self.proto.objective = Some(CpObjectiveProto {
            vars,
            coeffs: coeffs.into_iter().map(|coeff| -coeff).collect(),
            offset: -(constant as f64),
            scaling_factor: -1.0,
            ..Default::default()
        });
    }

    /// Hints the search that `var` is likely `value` in a good solution.
    /// Hints guide the search; they are not constraints.
    pub fn add_hint(&mut self, var: IntVar, value: i64) {
        let index = self.owned_var(var);
        let hint = self
            .proto
            .solution_hint
            .get_or_insert_with(Default::default);
        hint.vars.push(index);
        hint.values.push(value);
    }

    /// Hints the search that `literal` is likely `value` in a good solution.
    pub fn add_bool_hint(&mut self, literal: BoolVar, value: bool) {
        let index = self.owned_literal(literal);
        // A hint targets the underlying variable: unwrap a negated literal.
        let (var, value) = if index >= 0 {
            (index, i64::from(value))
        } else {
            (-index - 1, i64::from(!value))
        };
        let hint = self
            .proto
            .solution_hint
            .get_or_insert_with(Default::default);
        hint.vars.push(var);
        hint.values.push(value);
    }

    /// Removes every hint added so far.
    pub fn clear_hints(&mut self) {
        self.proto.solution_hint = None;
    }

    /// Adds literals the solve assumes true. When that makes the model
    /// infeasible, the raw response's
    /// `sufficient_assumptions_for_infeasibility` field names a subset of
    /// them explaining the conflict.
    pub fn add_assumptions(&mut self, literals: impl IntoIterator<Item = BoolVar>) {
        for literal in literals {
            let index = self.owned_literal(literal);
            self.proto.assumptions.push(index);
        }
    }

    /// Removes every assumption added so far.
    pub fn clear_assumptions(&mut self) {
        self.proto.assumptions.clear();
    }

    /// The model as its wire representation.
    pub fn proto(&self) -> &CpModelProto {
        &self.proto
    }

    /// Consumes the builder, returning the wire representation.
    pub fn into_proto(self) -> CpModelProto {
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
            "the expression uses variables from a different CpModelBuilder",
        );
        expr
    }

    /// Checks a variable's origin, panicking on a handle from another model.
    #[track_caller]
    fn owned_var(&self, var: IntVar) -> i32 {
        assert!(
            var.model == self.id,
            "the variable belongs to a different CpModelBuilder",
        );
        var.index
    }

    /// Checks a literal's origin, panicking on a handle from another model.
    #[track_caller]
    fn owned_literal(&self, literal: BoolVar) -> i32 {
        assert!(
            literal.model == self.id,
            "the literal belongs to a different CpModelBuilder",
        );
        literal.literal_index()
    }

    /// Checks an interval's origin, panicking on a handle from another model.
    #[track_caller]
    fn owned_interval(&self, interval: IntervalVar) -> i32 {
        assert!(
            interval.model == self.id,
            "the interval belongs to a different CpModelBuilder",
        );
        interval.index
    }

    #[track_caller]
    fn bool_argument_proto(
        &self,
        literals: impl IntoIterator<Item = BoolVar>,
    ) -> BoolArgumentProto {
        BoolArgumentProto {
            literals: literals
                .into_iter()
                .map(|literal| self.owned_literal(literal))
                .collect(),
        }
    }

    #[track_caller]
    fn linear_argument_proto<T: Into<LinearExpr>>(
        &self,
        target: impl Into<LinearExpr>,
        exprs: impl IntoIterator<Item = T>,
        negate: bool,
    ) -> LinearArgumentProto {
        let sign = if negate { -1 } else { 1 };
        LinearArgumentProto {
            target: Some(linear_expression_proto(self.owned_expr(target) * sign)),
            exprs: exprs
                .into_iter()
                .map(|expr| linear_expression_proto(self.owned_expr(expr) * sign))
                .collect(),
        }
    }

    #[track_caller]
    fn table_constraint_proto<T: Into<LinearExpr>>(
        &self,
        exprs: impl IntoIterator<Item = T>,
        tuples: impl IntoIterator<Item = impl AsRef<[i64]>>,
        negated: bool,
    ) -> TableConstraintProto {
        let exprs: Vec<LinearExpressionProto> = exprs
            .into_iter()
            .map(|expr| linear_expression_proto(self.owned_expr(expr)))
            .collect();
        let mut values = Vec::new();
        for tuple in tuples {
            let tuple = tuple.as_ref();
            assert_eq!(
                tuple.len(),
                exprs.len(),
                "every tuple needs one value per expression",
            );
            values.extend_from_slice(tuple);
        }
        TableConstraintProto {
            exprs,
            values,
            negated,
            ..Default::default()
        }
    }

    /// Splits `(tail, head, literal)` arcs into the proto's parallel lists.
    #[track_caller]
    fn arc_lists(
        &self,
        arcs: impl IntoIterator<Item = (i32, i32, BoolVar)>,
    ) -> (Vec<i32>, Vec<i32>, Vec<i32>) {
        let mut tails = Vec::new();
        let mut heads = Vec::new();
        let mut literals = Vec::new();
        for (tail, head, literal) in arcs {
            tails.push(tail);
            heads.push(head);
            literals.push(self.owned_literal(literal));
        }
        (tails, heads, literals)
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
            start: Some(linear_expression_proto(self.owned_expr(start))),
            size: Some(linear_expression_proto(self.owned_expr(size))),
            end: Some(linear_expression_proto(self.owned_expr(end))),
        };
        let index = self.proto.constraints.len() as i32;
        let constraint = self.append_constraint(constraint_proto::Constraint::Interval(interval));
        if let Some(presence) = presence {
            constraint.only_enforce_if([presence]);
        }
        IntervalVar {
            model: self.id,
            index,
        }
    }

    fn append_constraint(&mut self, constraint: constraint_proto::Constraint) -> Constraint<'_> {
        self.proto.constraints.push(ConstraintProto {
            constraint: Some(constraint),
            ..Default::default()
        });
        Constraint {
            model: self.id,
            proto: self.proto.constraints.last_mut().expect("just pushed"),
        }
    }
}

/// A constraint freshly added to a [`CpModelBuilder`], open for refinement.
/// Dropping it unchanged is the common case, not a mistake.
pub struct Constraint<'model> {
    model: u32,
    proto: &'model mut ConstraintProto,
}

impl Constraint<'_> {
    /// Makes the constraint apply only when every literal is true
    /// (half-reified). Callable multiple times; literals accumulate.
    #[track_caller]
    pub fn only_enforce_if(self, literals: impl IntoIterator<Item = BoolVar>) -> Self {
        for literal in literals {
            assert!(
                literal.model == self.model,
                "the literal belongs to a different CpModelBuilder",
            );
            self.proto.enforcement_literal.push(literal.literal_index());
        }
        self
    }

    /// Names the constraint for logs and debugging output.
    pub fn with_name(self, name: impl Into<String>) -> Self {
        self.proto.name = name.into();
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
    fn minimize_keeps_terms_and_default_scaling() {
        let mut model = CpModelBuilder::new();
        let x = model.new_int_var(0..=10);
        model.minimize(2 * x + 5);

        let objective = model.proto().objective.as_ref().expect("objective set");
        assert_eq!(objective.coeffs, vec![2]);
        assert_eq!(objective.offset, 5.0);
        assert_eq!(objective.scaling_factor, 0.0);
    }

    #[test]
    fn optional_interval_carries_presence_literal() {
        let mut model = CpModelBuilder::new();
        let presence = model.new_bool_var();
        let start = model.new_int_var(0..=10);
        let interval = model.new_optional_interval_var(start, 3, start + 3, presence);

        assert_eq!(interval.index, 0);
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

    #[test]
    fn cumulative_encodes_capacity_intervals_and_demands() {
        let mut model = CpModelBuilder::new();
        let s0 = model.new_int_var(0..=10);
        let s1 = model.new_int_var(0..=10);
        let intervals = [
            model.new_interval_var(s0, 2, s0 + 2),
            model.new_interval_var(s1, 3, s1 + 3),
        ];
        model.add_cumulative(5, &intervals, &[1, 4]);

        let Some(constraint_proto::Constraint::Cumulative(cumulative)) =
            &model.proto().constraints[2].constraint
        else {
            panic!("expected a cumulative constraint");
        };
        assert_eq!(
            cumulative.capacity.as_ref().expect("capacity set").offset,
            5
        );
        assert_eq!(cumulative.intervals, vec![0, 1]);
        let demands: Vec<i64> = cumulative
            .demands
            .iter()
            .map(|demand| demand.offset)
            .collect();
        assert_eq!(demands, vec![1, 4]);
    }

    #[test]
    #[should_panic(expected = "one demand per interval")]
    fn cumulative_rejects_mismatched_lengths() {
        let mut model = CpModelBuilder::new();
        let s0 = model.new_int_var(0..=10);
        let interval = model.new_interval_var(s0, 2, s0 + 2);
        model.add_cumulative(5, &[interval], &[1, 2]);
    }

    #[test]
    fn max_equality_encodes_target_and_exprs() {
        let mut model = CpModelBuilder::new();
        let target = model.new_int_var(0..=10);
        let x = model.new_int_var(0..=10);
        let y = model.new_int_var(0..=10);
        model.add_max_equality(target, [x, y]);

        let Some(constraint_proto::Constraint::LinMax(argument)) =
            &model.proto().constraints[0].constraint
        else {
            panic!("expected a lin_max constraint");
        };
        assert_eq!(argument.target.as_ref().expect("target").vars, vec![0]);
        assert_eq!(argument.target.as_ref().expect("target").coeffs, vec![1]);
        assert_eq!(argument.exprs.len(), 2);
        assert_eq!(argument.exprs[0].vars, vec![1]);
        assert_eq!(argument.exprs[1].vars, vec![2]);
    }

    #[test]
    fn min_equality_negates_into_lin_max() {
        let mut model = CpModelBuilder::new();
        let target = model.new_int_var(0..=10);
        let x = model.new_int_var(0..=10);
        model.add_min_equality(target, [x + 1]);

        let Some(constraint_proto::Constraint::LinMax(argument)) =
            &model.proto().constraints[0].constraint
        else {
            panic!("expected a lin_max constraint");
        };
        // min(e) == -max(-e): both sides arrive negated.
        assert_eq!(argument.target.as_ref().expect("target").coeffs, vec![-1]);
        assert_eq!(argument.exprs[0].coeffs, vec![-1]);
        assert_eq!(argument.exprs[0].offset, -1);
    }

    #[test]
    #[should_panic(expected = "different CpModelBuilder")]
    fn rejects_an_expression_from_another_model() {
        let mut model_a = CpModelBuilder::new();
        let mut model_b = CpModelBuilder::new();
        let x = model_a.new_int_var(0..=10);
        model_b.add_less_or_equal(x, 3);
    }

    #[test]
    #[should_panic(expected = "different CpModelBuilder")]
    fn rejects_a_literal_from_another_model() {
        let mut model_a = CpModelBuilder::new();
        let mut model_b = CpModelBuilder::new();
        let flag = model_a.new_bool_var();
        model_b.add_exactly_one([flag]);
    }

    #[test]
    #[should_panic(expected = "different CpModelBuilder")]
    fn rejects_an_enforcement_literal_from_another_model() {
        let mut model_a = CpModelBuilder::new();
        let mut model_b = CpModelBuilder::new();
        let flag = model_a.new_bool_var();
        let x = model_b.new_int_var(0..=5);
        model_b.add_equality(x, 3).only_enforce_if([flag]);
    }

    #[test]
    fn a_cloned_builder_accepts_pre_clone_handles() {
        let mut original = CpModelBuilder::new();
        let x = original.new_int_var(0..=10);
        let mut branch = original.clone();
        branch.add_less_or_equal(x, 3);
        original.add_greater_or_equal(x, 7);
    }

    #[test]
    fn element_encodes_index_target_and_exprs() {
        let mut model = CpModelBuilder::new();
        let index = model.new_int_var(0..=2);
        let target = model.new_int_var(0..=10);
        model.add_element(index, [3, 7, 9], target);

        let Some(constraint_proto::Constraint::Element(element)) =
            &model.proto().constraints[0].constraint
        else {
            panic!("expected an element constraint");
        };
        assert_eq!(element.linear_index.as_ref().expect("index").vars, vec![0]);
        assert_eq!(
            element.linear_target.as_ref().expect("target").vars,
            vec![1]
        );
        let values: Vec<i64> = element.exprs.iter().map(|expr| expr.offset).collect();
        assert_eq!(values, vec![3, 7, 9]);
    }

    #[test]
    fn allowed_assignments_flatten_tuples() {
        let mut model = CpModelBuilder::new();
        let x = model.new_int_var(0..=5);
        let y = model.new_int_var(0..=5);
        model.add_allowed_assignments([x, y], [[1, 2], [2, 3]]);

        let Some(constraint_proto::Constraint::Table(table)) =
            &model.proto().constraints[0].constraint
        else {
            panic!("expected a table constraint");
        };
        assert_eq!(table.exprs.len(), 2);
        assert_eq!(table.values, vec![1, 2, 2, 3]);
        assert!(!table.negated);
    }

    #[test]
    fn forbidden_assignments_set_the_negated_flag() {
        let mut model = CpModelBuilder::new();
        let x = model.new_int_var(0..=5);
        model.add_forbidden_assignments([x], [[4]]);

        let Some(constraint_proto::Constraint::Table(table)) =
            &model.proto().constraints[0].constraint
        else {
            panic!("expected a table constraint");
        };
        assert!(table.negated);
    }

    #[test]
    #[should_panic(expected = "one value per expression")]
    fn table_rejects_a_ragged_tuple() {
        let mut model = CpModelBuilder::new();
        let x = model.new_int_var(0..=5);
        let y = model.new_int_var(0..=5);
        model.add_allowed_assignments([x, y], [vec![1, 2], vec![3]]);
    }

    #[test]
    fn inverse_encodes_both_directions() {
        let mut model = CpModelBuilder::new();
        let f: Vec<_> = (0..3).map(|_| model.new_int_var(0..=2)).collect();
        let g: Vec<_> = (0..3).map(|_| model.new_int_var(0..=2)).collect();
        model.add_inverse(f, g);

        let Some(constraint_proto::Constraint::Inverse(inverse)) =
            &model.proto().constraints[0].constraint
        else {
            panic!("expected an inverse constraint");
        };
        assert_eq!(inverse.f_direct, vec![0, 1, 2]);
        assert_eq!(inverse.f_inverse, vec![3, 4, 5]);
    }

    #[test]
    #[should_panic(expected = "arrays of equal length")]
    fn inverse_rejects_mismatched_lengths() {
        let mut model = CpModelBuilder::new();
        let f = model.new_int_var(0..=1);
        model.add_inverse([f], []);
    }

    #[test]
    fn circuit_encodes_arcs_with_literals() {
        let mut model = CpModelBuilder::new();
        let a = model.new_bool_var();
        let b = model.new_bool_var();
        model.add_circuit([(0, 1, a), (1, 0, b.not())]);

        let Some(constraint_proto::Constraint::Circuit(circuit)) =
            &model.proto().constraints[0].constraint
        else {
            panic!("expected a circuit constraint");
        };
        assert_eq!(circuit.tails, vec![0, 1]);
        assert_eq!(circuit.heads, vec![1, 0]);
        assert_eq!(circuit.literals, vec![0, -2]);
    }

    #[test]
    fn multiple_circuit_encodes_into_routes() {
        let mut model = CpModelBuilder::new();
        let a = model.new_bool_var();
        model.add_multiple_circuit([(0, 1, a)]);

        let Some(constraint_proto::Constraint::Routes(routes)) =
            &model.proto().constraints[0].constraint
        else {
            panic!("expected a routes constraint");
        };
        assert_eq!(routes.tails, vec![0]);
        assert_eq!(routes.heads, vec![1]);
        assert_eq!(routes.literals, vec![0]);
    }

    #[test]
    fn automaton_encodes_transitions_in_parallel_lists() {
        let mut model = CpModelBuilder::new();
        let steps: Vec<_> = (0..2).map(|_| model.new_int_var(0..=1)).collect();
        model.add_automaton(steps, 0, [2], [(0, 1, 1), (1, 2, 0)]);

        let Some(constraint_proto::Constraint::Automaton(automaton)) =
            &model.proto().constraints[0].constraint
        else {
            panic!("expected an automaton constraint");
        };
        assert_eq!(automaton.starting_state, 0);
        assert_eq!(automaton.final_states, vec![2]);
        assert_eq!(automaton.transition_tail, vec![0, 1]);
        assert_eq!(automaton.transition_head, vec![1, 2]);
        assert_eq!(automaton.transition_label, vec![1, 0]);
        assert_eq!(automaton.exprs.len(), 2);
    }

    #[test]
    fn reservoir_encodes_events() {
        let mut model = CpModelBuilder::new();
        let time = model.new_int_var(0..=10);
        model.add_reservoir(0, 5, [(LinearExpr::from(time), 3), (time + 2, -3)]);

        let Some(constraint_proto::Constraint::Reservoir(reservoir)) =
            &model.proto().constraints[0].constraint
        else {
            panic!("expected a reservoir constraint");
        };
        assert_eq!(reservoir.min_level, 0);
        assert_eq!(reservoir.max_level, 5);
        assert_eq!(reservoir.time_exprs.len(), 2);
        assert_eq!(reservoir.level_changes[0].offset, 3);
        assert_eq!(reservoir.level_changes[1].offset, -3);
        assert!(reservoir.active_literals.is_empty());
    }

    #[test]
    fn optional_reservoir_events_carry_literals() {
        let mut model = CpModelBuilder::new();
        let time = model.new_int_var(0..=10);
        let active = model.new_bool_var();
        model.add_reservoir_with_optional_events(0, 5, [(time, 2, active)]);

        let Some(constraint_proto::Constraint::Reservoir(reservoir)) =
            &model.proto().constraints[0].constraint
        else {
            panic!("expected a reservoir constraint");
        };
        assert_eq!(reservoir.active_literals, vec![1]);
    }

    #[test]
    fn no_overlap_2d_pairs_intervals() {
        let mut model = CpModelBuilder::new();
        let x = model.new_int_var(0..=5);
        let y = model.new_int_var(0..=5);
        let width = model.new_interval_var(x, 2, x + 2);
        let height = model.new_interval_var(y, 2, y + 2);
        model.add_no_overlap_2d([(width, height)]);

        let Some(constraint_proto::Constraint::NoOverlap2d(no_overlap)) =
            &model.proto().constraints[2].constraint
        else {
            panic!("expected a no_overlap_2d constraint");
        };
        assert_eq!(no_overlap.x_intervals, vec![0]);
        assert_eq!(no_overlap.y_intervals, vec![1]);
    }

    #[test]
    fn division_equality_orders_numerator_then_denominator() {
        let mut model = CpModelBuilder::new();
        let target = model.new_int_var(0..=10);
        let numerator = model.new_int_var(0..=10);
        let denominator = model.new_int_var(1..=10);
        model.add_division_equality(target, numerator, denominator);

        let Some(constraint_proto::Constraint::IntDiv(argument)) =
            &model.proto().constraints[0].constraint
        else {
            panic!("expected an int_div constraint");
        };
        assert_eq!(argument.target.as_ref().expect("target").vars, vec![0]);
        assert_eq!(argument.exprs[0].vars, vec![1]);
        assert_eq!(argument.exprs[1].vars, vec![2]);
    }

    #[test]
    fn abs_equality_encodes_max_of_expr_and_negation() {
        let mut model = CpModelBuilder::new();
        let target = model.new_int_var(0..=10);
        let x = model.new_int_var(-10..=10);
        model.add_abs_equality(target, x);

        let Some(constraint_proto::Constraint::LinMax(argument)) =
            &model.proto().constraints[0].constraint
        else {
            panic!("expected a lin_max constraint");
        };
        assert_eq!(argument.exprs[0].coeffs, vec![1]);
        assert_eq!(argument.exprs[1].coeffs, vec![-1]);
    }

    #[test]
    fn hints_accumulate_and_unwrap_negated_literals() {
        let mut model = CpModelBuilder::new();
        let x = model.new_int_var(0..=10);
        let flag = model.new_bool_var();
        model.add_hint(x, 7);
        model.add_bool_hint(flag.not(), true);

        let hint = model.proto().solution_hint.as_ref().expect("hints set");
        assert_eq!(hint.vars, vec![0, 1]);
        // Hinting "not(flag) is true" means the underlying flag is 0.
        assert_eq!(hint.values, vec![7, 0]);

        model.clear_hints();
        assert!(model.proto().solution_hint.is_none());
    }

    #[test]
    fn assumptions_store_literal_indices() {
        let mut model = CpModelBuilder::new();
        let a = model.new_bool_var();
        let b = model.new_bool_var();
        model.add_assumptions([a, b.not()]);

        assert_eq!(model.proto().assumptions, vec![0, -2]);
        model.clear_assumptions();
        assert!(model.proto().assumptions.is_empty());
    }
}
