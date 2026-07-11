//! End-to-end tests through the C++ shim into the routing library.

use oxidor_routing::protos::prost_types::Duration;
use oxidor_routing::{RoutingProblem, RoutingSearchParameters, RoutingStatus};

/// The classic 4-city instance; the optimal tour 0→1→3→2→0 costs 80.
fn four_city_matrix() -> Vec<Vec<i64>> {
    vec![
        vec![0, 10, 15, 20],
        vec![10, 0, 35, 25],
        vec![15, 35, 0, 30],
        vec![20, 25, 30, 0],
    ]
}

#[test]
fn solves_a_small_tsp_to_the_known_optimum() {
    let solution = RoutingProblem::from_matrix(four_city_matrix())
        .expect("square matrix")
        .solve()
        .expect("solve runs");

    assert_eq!(solution.status(), RoutingStatus::Success);
    assert!(solution.has_solution());
    assert_eq!(solution.objective(), 80);

    let route = &solution.routes()[0];
    assert_eq!(route.len(), 3, "three cities besides the depot");
    // The optimal tour visits 1, 3, 2 in one direction or the other.
    assert!(route == &vec![1, 3, 2] || route == &vec![2, 3, 1]);
}

#[test]
fn solves_a_capacitated_vrp_visiting_every_customer() {
    // Depot 0 plus four unit-demand customers; two vehicles of capacity 2.
    let matrix = vec![
        vec![0, 4, 4, 6, 6],
        vec![4, 0, 2, 8, 8],
        vec![4, 2, 0, 8, 8],
        vec![6, 8, 8, 0, 2],
        vec![6, 8, 8, 2, 0],
    ];
    let solution = RoutingProblem::from_matrix(matrix)
        .expect("square matrix")
        .with_vehicles(2)
        .with_capacities(vec![0, 1, 1, 1, 1], vec![2, 2])
        .solve()
        .expect("solve runs");

    assert_eq!(solution.status(), RoutingStatus::Success);
    let mut visited: Vec<usize> = solution.routes().iter().flatten().copied().collect();
    visited.sort_unstable();
    assert_eq!(visited, vec![1, 2, 3, 4], "every customer exactly once");
    for route in solution.routes() {
        assert!(route.len() <= 2, "capacity 2 per vehicle: {route:?}");
    }
    // Pairing nearby customers (1,2) and (3,4) is optimal: 4+2+4 + 6+2+6 = 24.
    assert_eq!(solution.objective(), 24);
}

#[test]
fn merges_search_parameters_over_defaults() {
    let parameters = RoutingSearchParameters {
        time_limit: Some(Duration {
            seconds: 10,
            nanos: 0,
        }),
        ..Default::default()
    };
    let solution = RoutingProblem::from_matrix(four_city_matrix())
        .expect("square matrix")
        .solve_with_parameters(&parameters)
        .expect("solve runs");

    assert_eq!(solution.status(), RoutingStatus::Success);
    assert_eq!(solution.objective(), 80);
}

#[test]
fn infeasible_capacities_report_no_solution() {
    // Total demand 4 exceeds the single vehicle's capacity 1.
    let matrix = vec![
        vec![0, 1, 1, 1, 1],
        vec![1, 0, 1, 1, 1],
        vec![1, 1, 0, 1, 1],
        vec![1, 1, 1, 0, 1],
        vec![1, 1, 1, 1, 0],
    ];
    let solution = RoutingProblem::from_matrix(matrix)
        .expect("square matrix")
        .with_capacities(vec![0, 1, 1, 1, 1], vec![1])
        .solve()
        .expect("the solve itself runs");

    assert!(!solution.has_solution());
    assert_ne!(solution.status(), RoutingStatus::Success);
}
