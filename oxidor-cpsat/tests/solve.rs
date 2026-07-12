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
fn minimizes_with_the_correct_objective_sign() {
    let mut model = CpModelBuilder::new();
    let x = model.new_int_var(0..=10);
    let y = model.new_int_var(0..=10);
    model.add_greater_or_equal(x + y, 4);
    model.minimize(2 * x + 3 * y);

    let response = model.solve();

    assert_eq!(response.status(), SolveStatus::Optimal);
    // The cheaper x carries the whole requirement: x = 4, y = 0 ⇒ 8.
    assert_eq!(response.objective_value(), 8.0);
    let solution = response.solution().expect("optimal implies a solution");
    assert_eq!(solution.value(2 * x + 3 * y), 8);
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
fn an_empty_model_solves_trivially() {
    let model = CpModelBuilder::new();
    let response = model.solve();
    assert_eq!(response.status(), SolveStatus::Optimal);
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
    assert!(solution.bool_value(use_big));
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
        .filter(|&&option| solution.bool_value(option))
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
fn cumulative_packs_within_capacity() {
    // Two units of capacity; two unit-demand tasks may run in parallel, but
    // the demand-2 task must run alone: minimal makespan 2 + 2 = 4.
    let mut model = CpModelBuilder::new();
    let horizon = 10;
    let makespan = model.new_int_var(0..=horizon);
    let demands = [1, 1, 2];
    let intervals: Vec<_> = demands
        .map(|_| {
            let start = model.new_int_var(0..=horizon);
            let interval = model.new_interval_var(start, 2, start + 2);
            model.add_less_or_equal(start + 2, makespan);
            interval
        })
        .into_iter()
        .collect();
    model.add_cumulative(2, &intervals, &demands);
    model.minimize(makespan);

    let response = model.solve();

    assert_eq!(response.status(), SolveStatus::Optimal);
    assert_eq!(response.objective_value(), 4.0);
}

#[test]
fn max_equality_supports_minimizing_the_peak() {
    // Two loads that must sum to 10; minimizing the peak balances them.
    let mut model = CpModelBuilder::new();
    let first = model.new_int_var(0..=10);
    let second = model.new_int_var(0..=10);
    model.add_equality(first + second, 10);
    let peak = model.new_int_var(0..=10);
    model.add_max_equality(peak, [first, second]);
    model.minimize(peak);

    let response = model.solve();

    assert_eq!(response.status(), SolveStatus::Optimal);
    assert_eq!(response.objective_value(), 5.0);
}

#[test]
fn min_equality_binds_the_smallest_expression() {
    let mut model = CpModelBuilder::new();
    let x = model.new_int_var(3..=3);
    let y = model.new_int_var(7..=7);
    let low = model.new_int_var(0..=10);
    model.add_min_equality(low, [x, y]);

    let response = model.solve();

    let solution = response.solution().expect("feasible");
    assert_eq!(solution.value(low), 3);
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
fn solve_with_time_limit_still_cracks_an_easy_model() {
    let mut model = CpModelBuilder::new();
    let x = model.new_int_var(0..=10);
    model.maximize(x);

    let response = model.solve_with_time_limit(std::time::Duration::from_secs(30));

    assert_eq!(response.status(), SolveStatus::Optimal);
    assert_eq!(response.solution().expect("optimal").value(x), 10);
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

    let mut token = oxidor_cpsat::StopToken::new();
    let stopper = token.stopper();
    let handle = std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(200));
        stopper.stop();
    });

    let started = Instant::now();
    let response = model.solve_interruptible_with_parameters(&mut token, &parameters);
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

    let mut token = oxidor_cpsat::StopToken::new();
    token.stop();
    let response = model.solve_interruptible(&mut token);

    // The pre-stopped environment ends the search before any work: were the
    // stop ignored, this trivial model would come back Optimal.
    assert_eq!(response.status(), SolveStatus::Unknown);
    assert!(response.solution().is_none());
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

#[test]
fn picks_shifts_like_the_crate_example() {
    // Mirrors the crate-level doctest so its golden value is executed in CI.
    let mut model = CpModelBuilder::new();
    let shifts = [
        model.new_bool_var(),
        model.new_bool_var(),
        model.new_bool_var(),
    ];
    let hours = [6, 4, 8];

    model.add_at_most_one([shifts[0], shifts[1]]);
    model.add_linear_constraint(shifts.into_iter().sum::<oxidor_cpsat::LinearExpr>(), 2..=2);
    model.maximize(
        shifts
            .iter()
            .zip(hours)
            .map(|(&s, h)| s * h)
            .sum::<oxidor_cpsat::LinearExpr>(),
    );

    let response = model.solve();
    let solution = response.solution().expect("feasible");
    assert_eq!(response.objective_value(), 14.0);
    assert!(solution.bool_value(shifts[2]));
}

#[test]
#[should_panic(expected = "different model")]
fn evaluating_a_foreign_handle_panics() {
    let mut solved = CpModelBuilder::new();
    let x = solved.new_int_var(0..=1);
    let response = solved.solve();
    let solution = response.solution().expect("feasible");
    let _ = solution.value(x); // same model: fine

    let mut other = CpModelBuilder::new();
    let foreign = other.new_int_var(0..=1);
    let _ = solution.value(foreign);
}

#[test]
#[should_panic(expected = "created after the solve")]
fn evaluating_a_post_solve_handle_panics() {
    let mut model = CpModelBuilder::new();
    let _x = model.new_int_var(0..=1);
    let response = model.solve();
    let late = model.new_int_var(0..=1);
    let _ = response.solution().expect("feasible").value(late);
}
