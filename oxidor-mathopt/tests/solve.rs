//! End-to-end tests that cross the FFI boundary into MathOpt.

use oxidor_mathopt::{Model, SolveInterrupter, SolverType, TerminationReason, Variable};

/// Solver answers are floating point; compare with a tolerance.
fn assert_close(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() < 1e-6,
        "expected ≈{expected}, got {actual}",
    );
}

/// max 2x + 3y  s.t.  x + y <= 14,  0 <= x, y <= 10  →  (4, 10), objective 38.
fn small_lp() -> (Model, Variable, Variable) {
    let mut model = Model::new();
    let x = model.add_continuous_variable(0.0..=10.0);
    let y = model.add_continuous_variable(0.0..=10.0);
    model.add_less_or_equal(x + y, 14.0);
    model.maximize(2.0 * x + 3.0 * y);
    (model, x, y)
}

#[test]
fn glop_solves_an_lp_to_optimality() {
    let (model, x, y) = small_lp();

    let result = model.solve(SolverType::Glop).expect("Glop is linked");

    assert_eq!(result.reason(), TerminationReason::Optimal);
    let solution = result.primal_solution().expect("optimal has a solution");
    assert_close(solution.objective_value(), 38.0);
    assert_close(solution.value(x), 4.0);
    assert_close(solution.value(y), 10.0);
    assert_close(solution.value(2.0 * x + 3.0 * y), 38.0);
}

#[test]
fn pdlp_solves_the_same_lp() {
    let (model, _, _) = small_lp();

    let result = model.solve(SolverType::Pdlp).expect("PDLP is linked");

    assert_eq!(result.reason(), TerminationReason::Optimal);
    let solution = result.primal_solution().expect("optimal has a solution");
    assert!(
        (solution.objective_value() - 38.0).abs() < 1e-3,
        "first-order solver stays within loose tolerance, got {}",
        solution.objective_value(),
    );
}

#[test]
fn gscip_solves_a_mip_with_integrality() {
    let mut model = Model::new();
    // LP relaxation would take x = 2.5; integrality forces 2.
    let x = model.add_integer_variable(0.0..=10.0);
    model.add_less_or_equal(2.0 * x, 5.0);
    model.maximize(x);

    let result = model.solve(SolverType::Gscip).expect("SCIP is linked");

    assert_eq!(result.reason(), TerminationReason::Optimal);
    let solution = result.primal_solution().expect("optimal has a solution");
    assert_close(solution.value(x), 2.0);
}

#[test]
fn cp_sat_solves_an_integer_model() {
    let mut model = Model::new();
    let x = model.add_integer_variable(0.0..=10.0);
    let y = model.add_integer_variable(0.0..=10.0);
    model.add_less_or_equal(x + y, 7.0);
    model.maximize(2.0 * x + y);

    let result = model.solve(SolverType::CpSat).expect("CP-SAT is linked");

    assert_eq!(result.reason(), TerminationReason::Optimal);
    let solution = result.primal_solution().expect("optimal has a solution");
    assert_close(solution.value(x), 7.0);
    assert_close(solution.value(y), 0.0);
}

#[test]
fn infeasibility_is_an_outcome_not_an_error() {
    let mut model = Model::new();
    let x = model.add_continuous_variable(0.0..=1.0);
    model.add_greater_or_equal(x, 2.0);

    let result = model
        .solve(SolverType::Glop)
        .expect("the solve itself runs");

    assert_eq!(result.reason(), TerminationReason::Infeasible);
    assert!(result.primal_solution().is_none());
}

#[test]
fn an_invalid_model_is_a_solve_error() {
    let mut model = Model::new();
    let x = model.add_continuous_variable(0.0..=1.0);
    // NaN coefficients fail MathOpt's model validation with a clean status.
    model.maximize(f64::NAN * x);

    let error = model
        .solve(SolverType::Glop)
        .expect_err("validation rejects NaN");

    assert_ne!(error.code, 0);
    assert!(!error.message.is_empty());
}

#[test]
fn a_pre_triggered_interrupter_stops_the_solve_immediately() {
    let (model, _, _) = small_lp();

    let interrupter = SolveInterrupter::new();
    interrupter.interrupt();
    assert!(interrupter.is_interrupted());

    // Glop checks the interrupter between iterations; a pre-triggered one
    // ends the solve before any work happens.
    let result = model
        .solve_interruptible(SolverType::Glop, &interrupter)
        .expect("interruption is not an error");
    assert_ne!(result.reason(), TerminationReason::Infeasible);
}
