//! End-to-end tests for streaming solution callbacks (the `callbacks`
//! feature): observation, early stop, panic propagation, handle branding.
#![cfg(feature = "callbacks")]

use std::collections::BTreeSet;
use std::ops::ControlFlow;

use oxidor_cpsat::{CpModelBuilder, SatParameters, SolveStatus};

fn single_worker_enumeration() -> SatParameters {
    SatParameters {
        enumerate_all_solutions: Some(true),
        num_workers: Some(1),
        ..Default::default()
    }
}

#[test]
fn streams_every_solution_of_an_enumeration() {
    let mut model = CpModelBuilder::new();
    let x = model.new_int_var(0..=2);

    let mut values = BTreeSet::new();
    let response = model.solve_with_solution_callback(&single_worker_enumeration(), |solution| {
        values.insert(solution.solution().expect("a streamed solution").value(x));
        ControlFlow::Continue(())
    });

    assert_eq!(response.status(), SolveStatus::Optimal);
    assert_eq!(values, BTreeSet::from([0, 1, 2]));
}

#[test]
fn breaking_from_the_callback_stops_the_search() {
    let mut model = CpModelBuilder::new();
    // 2^30 solutions: only the early stop can end this enumeration quickly.
    let variables: Vec<_> = (0..30).map(|_| model.new_bool_var()).collect();
    model.add_bool_or(variables);

    let mut seen = 0;
    let started = std::time::Instant::now();
    let response = model.solve_with_solution_callback(&single_worker_enumeration(), |_| {
        seen += 1;
        if seen >= 3 {
            ControlFlow::Break(())
        } else {
            ControlFlow::Continue(())
        }
    });

    assert!(seen >= 3);
    assert!(
        started.elapsed() < std::time::Duration::from_secs(20),
        "the break must end the solve"
    );
    // Enumeration was cut short: optimality (= full enumeration) cannot have
    // been proven.
    assert_ne!(response.status(), SolveStatus::Optimal);
}

#[test]
fn improving_solutions_stream_during_optimization() {
    let mut model = CpModelBuilder::new();
    let x = model.new_int_var(0..=10);
    model.maximize(x);

    let mut last_seen = None;
    let response = model.solve_with_solution_callback(&SatParameters::default(), |solution| {
        last_seen = Some(solution.solution().expect("a streamed solution").value(x));
        ControlFlow::Continue(())
    });

    assert_eq!(response.status(), SolveStatus::Optimal);
    // The final streamed solution is the optimum the response reports.
    assert_eq!(last_seen, Some(10));
    assert_eq!(response.solution().expect("optimal").value(x), 10);
}

#[test]
fn a_callback_panic_resumes_on_the_calling_thread() {
    let mut model = CpModelBuilder::new();
    let x = model.new_int_var(0..=10);
    model.maximize(x);

    let outcome = std::panic::catch_unwind(|| {
        model.solve_with_solution_callback(&SatParameters::default(), |_| {
            panic!("boom from the observer");
        })
    });

    let payload = outcome.expect_err("the callback panic must propagate");
    let message = payload
        .downcast_ref::<&str>()
        .copied()
        .expect("the original panic payload");
    assert_eq!(message, "boom from the observer");
}

#[test]
fn streamed_solutions_carry_the_model_identity() {
    let mut model = CpModelBuilder::new();
    let x = model.new_int_var(0..=0);

    let mut other = CpModelBuilder::new();
    let foreign = other.new_int_var(0..=0);

    let mut checked = false;
    model.solve_with_solution_callback(&SatParameters::default(), |solution| {
        let solution = solution.solution().expect("a streamed solution");
        assert_eq!(solution.value(x), 0);
        // A foreign handle must still be rejected inside the callback.
        let foreign_read = std::panic::catch_unwind(|| solution.value(foreign));
        assert!(foreign_read.is_err(), "foreign handles must panic");
        checked = true;
        ControlFlow::Continue(())
    });
    assert!(checked, "the observer ran at least once");
}
