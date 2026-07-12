//! End-to-end tests that cross the FFI boundary into MathOpt.

use oxidor_mathopt::{
    LinearExpr, Model, SolveInterrupter, SolverType, TerminationReason, Variable,
};

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
    let x = model.new_continuous_variable(0.0..=10.0);
    let y = model.new_continuous_variable(0.0..=10.0);
    model.add_less_or_equal(x + y, 14.0);
    model.maximize(2.0 * x + 3.0 * y);
    (model, x, y)
}

#[test]
fn glop_solves_an_lp_to_optimality() {
    let (model, x, y) = small_lp();

    let result = model.solve(SolverType::Glop).expect("Glop is linked");

    assert_eq!(result.status(), TerminationReason::Optimal);
    let solution = result.primal_solution().expect("optimal has a solution");
    assert_close(solution.objective_value(), 38.0);
    assert_close(solution.value(x), 4.0);
    assert_close(solution.value(y), 10.0);
    assert_close(solution.value(2.0 * x + 3.0 * y), 38.0);
}

#[test]
fn glop_minimizes_with_the_correct_objective_sign() {
    let mut model = Model::new();
    let x = model.new_continuous_variable(0.0..=10.0);
    let y = model.new_continuous_variable(0.0..=10.0);
    model.add_greater_or_equal(x + y, 4.0);
    model.minimize(2.0 * x + 3.0 * y);

    let result = model.solve(SolverType::Glop).expect("Glop is linked");

    assert_eq!(result.status(), TerminationReason::Optimal);
    let solution = result.primal_solution().expect("optimal has a solution");
    // The cheaper x carries the whole requirement: x = 4, y = 0 ⇒ 8.
    assert_close(solution.objective_value(), 8.0);
    assert_close(solution.value(x), 4.0);
    assert_close(solution.value(y), 0.0);
}

#[test]
fn pdlp_solves_the_same_lp() {
    let (model, _, _) = small_lp();

    let result = model.solve(SolverType::Pdlp).expect("PDLP is linked");

    assert_eq!(result.status(), TerminationReason::Optimal);
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
    let x = model.new_integer_variable(0.0..=10.0);
    model.add_less_or_equal(2.0 * x, 5.0);
    model.maximize(x);

    let result = model.solve(SolverType::Gscip).expect("SCIP is linked");

    assert_eq!(result.status(), TerminationReason::Optimal);
    let solution = result.primal_solution().expect("optimal has a solution");
    assert_close(solution.value(x), 2.0);
}

#[test]
fn cp_sat_solves_an_integer_model() {
    let mut model = Model::new();
    let x = model.new_integer_variable(0.0..=10.0);
    let y = model.new_integer_variable(0.0..=10.0);
    model.add_less_or_equal(x + y, 7.0);
    model.maximize(2.0 * x + y);

    let result = model.solve(SolverType::CpSat).expect("CP-SAT is linked");

    assert_eq!(result.status(), TerminationReason::Optimal);
    let solution = result.primal_solution().expect("optimal has a solution");
    assert_close(solution.value(x), 7.0);
    assert_close(solution.value(y), 0.0);
}

#[test]
fn infeasibility_is_an_outcome_not_an_error() {
    let mut model = Model::new();
    let x = model.new_continuous_variable(0.0..=1.0);
    model.add_greater_or_equal(x, 2.0);

    let result = model
        .solve(SolverType::Glop)
        .expect("the solve itself runs");

    assert_eq!(result.status(), TerminationReason::Infeasible);
    assert!(result.primal_solution().is_none());
}

#[test]
fn unboundedness_is_an_outcome_not_an_error() {
    let mut model = Model::new();
    let x = model.new_continuous_variable(0.0..=f64::INFINITY);
    model.maximize(x);

    let result = model
        .solve(SolverType::Glop)
        .expect("the solve itself runs");

    assert!(
        matches!(
            result.status(),
            TerminationReason::Unbounded | TerminationReason::InfeasibleOrUnbounded
        ),
        "got {:?}",
        result.status(),
    );
}

#[test]
fn an_invalid_model_is_a_solve_error() {
    let mut model = Model::new();
    let x = model.new_continuous_variable(0.0..=1.0);
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

    // Glop checks the interrupter before iterating; were the trigger ignored,
    // this trivial LP would come back Optimal.
    let result = model
        .solve_interruptible(SolverType::Glop, &interrupter)
        .expect("interruption is not an error");
    assert_eq!(result.status(), TerminationReason::NoSolutionFound);
    assert!(result.primal_solution().is_none());
}

/// A pseudo-random coefficient in `0..100` (deterministic LCG — tests must
/// not depend on ambient randomness).
fn lcg_coefficient(state: &mut u64) -> f64 {
    *state = state
        .wrapping_mul(6364136223846793005)
        .wrapping_add(1442695040888963407);
    ((*state >> 33) % 100) as f64
}

#[test]
fn an_interrupter_stops_a_running_mip_search() {
    use std::time::{Duration, Instant};

    // A market-split (Cornuéjols–Dawande) feasibility MIP: notoriously hard
    // for branch and bound at this size — SCIP cannot settle it in seconds,
    // so only the interrupt can end the solve quickly.
    let mut model = Model::new();
    let variables: Vec<Variable> = (0..50)
        .map(|_| model.new_integer_variable(0.0..=1.0))
        .collect();
    let mut state: u64 = 0x9E37_79B9_7F4A_7C15;
    for _ in 0..6 {
        let coefficients: Vec<f64> = (0..variables.len())
            .map(|_| lcg_coefficient(&mut state))
            .collect();
        let row: LinearExpr = variables
            .iter()
            .zip(&coefficients)
            .map(|(&variable, &coefficient)| coefficient * variable)
            .sum();
        let half = (coefficients.iter().sum::<f64>() / 2.0).floor();
        model.add_equality(row, half);
    }

    let interrupter = SolveInterrupter::new();
    let trigger = interrupter.clone();
    let handle = std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(300));
        trigger.interrupt();
    });

    let started = Instant::now();
    let result = model
        .solve_interruptible(SolverType::Gscip, &interrupter)
        .expect("interruption is not an error");
    handle.join().expect("trigger thread");

    assert!(
        started.elapsed() < Duration::from_secs(20),
        "the interrupt must end the solve"
    );
    // Cut short, the search cannot have settled the instance either way.
    assert!(
        !matches!(
            result.status(),
            TerminationReason::Optimal | TerminationReason::Infeasible
        ),
        "got {:?}",
        result.status(),
    );
}

#[test]
#[should_panic(expected = "different model")]
fn evaluating_a_foreign_handle_panics() {
    let (model, _, _) = small_lp();
    let result = model.solve(SolverType::Glop).expect("Glop is linked");
    let solution = result.primal_solution().expect("optimal has a solution");

    let mut other = Model::new();
    let foreign = other.new_continuous_variable(0.0..=1.0);
    let _ = solution.value(foreign);
}
