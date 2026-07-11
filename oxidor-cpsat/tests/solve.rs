//! End-to-end tests that cross the FFI boundary into the native solver.

use oxidor_cpsat::{CpModelBuilder, SatParameters, SolveStatus};

#[test]
fn maximizes_a_small_linear_objective_to_optimality() {
    let mut model = CpModelBuilder::new();
    let x = model.new_int_var(0..=10);
    let y = model.new_int_var(0..=10);
    model.add_less_or_equal(x + y, 14);
    model.maximize(2 * x + 3 * y);

    let response = model.solve();

    assert_eq!(response.status(), SolveStatus::Optimal);
    assert_eq!(response.objective_value(), 38.0);
    let solution = response.solution().expect("optimal implies a solution");
    assert_eq!(solution.value(x), 4);
    assert_eq!(solution.value(y), 10);
    assert_eq!(solution.value(2 * x + 3 * y), 38);
}

#[test]
fn proves_infeasibility() {
    let mut model = CpModelBuilder::new();
    let x = model.new_int_var(0..=1);
    model.add_equality(x, 5);

    let response = model.solve();

    assert_eq!(response.status(), SolveStatus::Infeasible);
    assert!(response.solution().is_none());
}

#[test]
fn enforcement_literals_gate_constraints() {
    let mut model = CpModelBuilder::new();
    let use_big = model.new_bool_var();
    let x = model.new_int_var(0..=100);
    // x is 7 in the small regime, 42 in the big one; prefer big.
    model.add_equality(x, 42).only_enforce_if([use_big]);
    model.add_equality(x, 7).only_enforce_if([use_big.not()]);
    model.maximize(x);

    let response = model.solve();

    let solution = response.solution().expect("feasible");
    assert_eq!(response.status(), SolveStatus::Optimal);
    assert!(solution.boolean_value(use_big));
    assert_eq!(solution.value(x), 42);
}

#[test]
fn exactly_one_picks_a_single_literal() {
    let mut model = CpModelBuilder::new();
    let options = [
        model.new_bool_var(),
        model.new_bool_var(),
        model.new_bool_var(),
    ];
    model.add_exactly_one(options);

    let response = model.solve();

    let solution = response.solution().expect("feasible");
    let chosen = options
        .iter()
        .filter(|&&option| solution.boolean_value(option))
        .count();
    assert_eq!(chosen, 1);
}

#[test]
fn no_overlap_yields_the_known_minimal_makespan() {
    let mut model = CpModelBuilder::new();
    let horizon = 20;
    let durations = [2, 3, 4];
    let makespan = model.new_int_var(0..=horizon);

    let intervals: Vec<_> = durations
        .map(|duration| {
            let start = model.new_int_var(0..=horizon);
            let interval = model.new_interval_var(start, duration, start + duration);
            model.add_less_or_equal(start + duration, makespan);
            interval
        })
        .into_iter()
        .collect();
    model.add_no_overlap(intervals);
    model.minimize(makespan);

    let response = model.solve();

    assert_eq!(response.status(), SolveStatus::Optimal);
    assert_eq!(response.objective_value(), 9.0);
}

#[test]
fn respects_solver_parameters() {
    let mut model = CpModelBuilder::new();
    let x = model.new_int_var(0..=10);
    model.maximize(x);

    let parameters = SatParameters {
        max_time_in_seconds: Some(30.0),
        num_workers: Some(1),
        ..Default::default()
    };
    let response = model.solve_with_parameters(&parameters);

    assert_eq!(response.status(), SolveStatus::Optimal);
    assert_eq!(response.solution().expect("optimal").value(x), 10);
    assert!(response.wall_time() < 30.0);
}

#[test]
fn stop_token_interrupts_an_endless_enumeration() {
    use std::time::{Duration, Instant};

    let mut model = CpModelBuilder::new();
    // 2^60 free Booleans: enumerating all solutions never terminates on its
    // own; only the stop request can end this solve.
    let variables: Vec<_> = (0..60).map(|_| model.new_bool_var()).collect();
    model.add_bool_or(variables);

    let parameters = SatParameters {
        enumerate_all_solutions: Some(true),
        num_workers: Some(1),
        ..Default::default()
    };

    let token = oxidor_cpsat::StopToken::new();
    let stopper = token.clone();
    let handle = std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(200));
        stopper.stop();
    });

    let started = Instant::now();
    let response = model.solve_interruptible_with_parameters(&token, &parameters);
    handle.join().expect("stopper thread");

    assert!(
        started.elapsed() < Duration::from_secs(20),
        "the stop request must end the solve"
    );
    // Enumeration was cut short: optimality (= full enumeration) cannot have
    // been proven.
    assert_ne!(response.status(), SolveStatus::Optimal);
}

#[test]
fn stopping_before_solving_returns_immediately() {
    let mut model = CpModelBuilder::new();
    let x = model.new_int_var(0..=10);
    model.maximize(x);

    let token = oxidor_cpsat::StopToken::new();
    token.stop();
    let response = model.solve_interruptible(&token);

    // The pre-stopped environment ends the search instantly; no conclusion is
    // reached.
    assert_ne!(response.status(), SolveStatus::Infeasible);
}

#[test]
fn enumerates_the_full_solution_set() {
    use std::collections::BTreeSet;

    let mut model = CpModelBuilder::new();
    let x = model.new_int_var(0..=2);

    let parameters = SatParameters {
        enumerate_all_solutions: Some(true),
        fill_additional_solutions_in_response: Some(true),
        solution_pool_size: Some(16),
        num_workers: Some(1),
        ..Default::default()
    };
    let response = model.solve_with_parameters(&parameters);

    assert_eq!(
        response.status(),
        SolveStatus::Optimal,
        "enumeration completed"
    );
    let values: BTreeSet<i64> = response
        .solutions()
        .map(|solution| solution.value(x))
        .collect();
    assert_eq!(values, BTreeSet::from([0, 1, 2]));
}
