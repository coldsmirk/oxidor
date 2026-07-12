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
fn element_picks_the_indexed_value() {
    let mut model = CpModelBuilder::new();
    let index = model.new_int_var(0..=2);
    let target = model.new_int_var(0..=100);
    model.add_element(index, [3, 7, 9], target);
    model.add_equality(target, 7);

    let response = model.solve();

    let solution = response.solution().expect("feasible");
    assert_eq!(solution.value(index), 1);
}

#[test]
fn allowed_assignments_restrict_the_tuple() {
    let mut model = CpModelBuilder::new();
    let x = model.new_int_var(0..=5);
    let y = model.new_int_var(0..=5);
    model.add_allowed_assignments([x, y], [[1, 2], [2, 3]]);
    model.maximize(x + y);

    let response = model.solve();

    assert_eq!(response.status(), SolveStatus::Optimal);
    let solution = response.solution().expect("optimal");
    assert_eq!((solution.value(x), solution.value(y)), (2, 3));
}

#[test]
fn forbidden_assignments_exclude_the_tuple() {
    let mut model = CpModelBuilder::new();
    let x = model.new_int_var(0..=1);
    let y = model.new_int_var(0..=1);
    model.add_forbidden_assignments([x, y], [[1, 1]]);
    model.maximize(x + y);

    let response = model.solve();

    // Without (1, 1), the best total is 1.
    assert_eq!(response.objective_value(), 1.0);
}

#[test]
fn circuit_finds_the_cheapest_tour() {
    // Three nodes, all six arcs: the cheap direction 0→1→2→0 costs 1+2+3=6,
    // the reverse 15.
    let mut model = CpModelBuilder::new();
    let arcs = [
        (0, 1, 1),
        (1, 2, 2),
        (2, 0, 3),
        (1, 0, 4),
        (2, 1, 5),
        (0, 2, 6),
    ];
    let literals: Vec<_> = arcs.map(|_| model.new_bool_var()).into_iter().collect();
    model.add_circuit(
        arcs.iter()
            .zip(&literals)
            .map(|(&(tail, head, _), &literal)| (tail, head, literal)),
    );
    model.minimize(
        arcs.iter()
            .zip(&literals)
            .map(|(&(_, _, cost), &literal)| literal * cost)
            .sum::<oxidor_cpsat::LinearExpr>(),
    );

    let response = model.solve();

    assert_eq!(response.status(), SolveStatus::Optimal);
    // Evaluate the cost expression exactly: the reported objective_value is a
    // double and may carry floating-point noise.
    let tour_cost = response.solution().expect("optimal").value(
        arcs.iter()
            .zip(&literals)
            .map(|(&(_, _, cost), &literal)| literal * cost)
            .sum::<oxidor_cpsat::LinearExpr>(),
    );
    assert_eq!(tour_cost, 6);
}

#[test]
fn multiplication_equality_binds_the_product() {
    let mut model = CpModelBuilder::new();
    let x = model.new_int_var(1..=12);
    let y = model.new_int_var(1..=12);
    let product = model.new_int_var(0..=144);
    model.add_multiplication_equality(product, [x, y]);
    model.add_equality(product, 12);
    model.minimize(x + y);

    let response = model.solve();

    // 12 = 3 × 4 minimizes the factor sum.
    assert_eq!(response.objective_value(), 7.0);
}

#[test]
fn division_and_modulo_round_toward_zero() {
    let mut model = CpModelBuilder::new();
    let numerator = model.new_constant(7);
    let divisor = model.new_constant(3);
    let quotient = model.new_int_var(0..=10);
    let remainder = model.new_int_var(0..=10);
    model.add_division_equality(quotient, numerator, divisor);
    model.add_modulo_equality(remainder, numerator, divisor);

    let response = model.solve();

    let solution = response.solution().expect("feasible");
    assert_eq!(solution.value(quotient), 2);
    assert_eq!(solution.value(remainder), 1);
}

#[test]
fn abs_equality_reflects_a_negative_value() {
    let mut model = CpModelBuilder::new();
    let x = model.new_constant(-5);
    let magnitude = model.new_int_var(0..=10);
    model.add_abs_equality(magnitude, x);

    let response = model.solve();

    assert_eq!(response.solution().expect("feasible").value(magnitude), 5);
}

#[test]
fn bool_xor_forces_odd_parity() {
    let mut model = CpModelBuilder::new();
    let a = model.new_bool_var();
    let b = model.new_bool_var();
    let c = model.new_bool_var();
    model.add_bool_xor([a, b, c]);
    model.add_bool_and([a, b]);

    let response = model.solve();

    // With a and b true, odd parity forces c true.
    assert!(response.solution().expect("feasible").bool_value(c));
}

#[test]
fn inverse_derives_the_inverse_permutation() {
    let mut model = CpModelBuilder::new();
    let f: Vec<_> = (0..3).map(|_| model.new_int_var(0..=2)).collect();
    let g: Vec<_> = (0..3).map(|_| model.new_int_var(0..=2)).collect();
    model.add_inverse(f.clone(), g.clone());
    model.add_equality(f[0], 1);
    model.add_equality(f[1], 2);

    let response = model.solve();

    let solution = response.solution().expect("feasible");
    // f = (1, 2, 0) forces its inverse g = (2, 0, 1).
    assert_eq!(solution.value(f[2]), 0);
    assert_eq!([g[0], g[1], g[2]].map(|var| solution.value(var)), [2, 0, 1]);
}

#[test]
fn automaton_accepts_only_the_encoded_sequence() {
    // A straight-line automaton accepting exactly the label sequence 1, 0, 1.
    let mut model = CpModelBuilder::new();
    let steps: Vec<_> = (0..3).map(|_| model.new_int_var(0..=1)).collect();
    model.add_automaton(steps.clone(), 0, [3], [(0, 1, 1), (1, 2, 0), (2, 3, 1)]);

    let response = model.solve();

    let solution = response.solution().expect("feasible");
    assert_eq!(
        steps
            .iter()
            .map(|&step| solution.value(step))
            .collect::<Vec<_>>(),
        vec![1, 0, 1]
    );
}

#[test]
fn reservoir_orders_the_refill_before_the_drain() {
    let mut model = CpModelBuilder::new();
    let refill = model.new_int_var(0..=10);
    let drain = model.new_int_var(0..=10);
    model.add_not_equal(refill, drain);
    model.add_reservoir(
        0,
        3,
        [
            (oxidor_cpsat::LinearExpr::from(refill), 3),
            (oxidor_cpsat::LinearExpr::from(drain), -2),
        ],
    );

    let response = model.solve();

    let solution = response.solution().expect("feasible");
    // Draining first would push the level to -2, below the floor.
    assert!(solution.value(refill) < solution.value(drain));
}

#[test]
fn no_overlap_2d_rejects_two_large_boxes_in_a_small_area() {
    // Two 2×2 boxes cannot fit anywhere in a 3×3 area.
    let mut model = CpModelBuilder::new();
    let mut boxes = Vec::new();
    for _ in 0..2 {
        let x = model.new_int_var(0..=1);
        let y = model.new_int_var(0..=1);
        let horizontal = model.new_interval_var(x, 2, x + 2);
        let vertical = model.new_interval_var(y, 2, y + 2);
        boxes.push((horizontal, vertical));
    }
    model.add_no_overlap_2d(boxes);

    let response = model.solve();

    assert_eq!(response.status(), SolveStatus::Infeasible);
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
fn solve_model_proto_runs_a_hand_modified_model() {
    // The wire-level escape hatch: build with the builder, drop to the proto,
    // tighten a domain by hand, solve raw, read the raw solution.
    let mut model = CpModelBuilder::new();
    let x = model.new_int_var(0..=10);
    model.maximize(x);

    let mut proto = model.into_proto();
    proto.variables[0].domain = vec![0, 7];
    let response = oxidor_cpsat::solve_model_proto(&proto, &SatParameters::default());

    assert_eq!(
        response.status(),
        oxidor_cpsat::protos::sat::CpSolverStatus::Optimal
    );
    assert_eq!(response.solution, vec![7]);
}

#[test]
fn solve_model_proto_reports_an_invalid_model_as_a_status() {
    // A variable with an empty domain fails CP-SAT's validation; the raw
    // entry must surface that as MODEL_INVALID, never as a crash.
    let mut model = CpModelBuilder::new();
    let _ = model.new_int_var(0..=1);
    let mut proto = model.into_proto();
    proto.variables[0].domain = vec![];

    let response = oxidor_cpsat::solve_model_proto(&proto, &SatParameters::default());

    assert_eq!(
        response.status(),
        oxidor_cpsat::protos::sat::CpSolverStatus::ModelInvalid
    );
}

#[test]
fn solve_model_proto_interruptible_honors_a_pre_stopped_token() {
    let mut model = CpModelBuilder::new();
    let x = model.new_int_var(0..=10);
    model.maximize(x);

    let mut token = oxidor_cpsat::StopToken::new();
    token.stop();
    let response = oxidor_cpsat::solve_model_proto_interruptible(
        &model.into_proto(),
        &SatParameters::default(),
        &mut token,
    );

    assert_eq!(
        response.status(),
        oxidor_cpsat::protos::sat::CpSolverStatus::Unknown
    );
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
