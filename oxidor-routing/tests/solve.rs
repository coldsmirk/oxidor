//! End-to-end tests through the C++ shim into the routing library.

use oxidor_routing::protos::prost_types::Duration;
use oxidor_routing::{RoutingProblem, RoutingSearchParameters, RoutingStatus, TimeDimension};

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
    let response = RoutingProblem::from_matrix(four_city_matrix())
        .expect("square matrix")
        .solve()
        .expect("solve runs");

    assert_eq!(response.status(), RoutingStatus::Success);
    let solution = response.solution().expect("a tour was found");
    assert_eq!(solution.objective_value(), 80);

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
    let response = RoutingProblem::from_matrix(matrix)
        .expect("square matrix")
        .with_vehicles(2)
        .with_capacities(vec![0, 1, 1, 1, 1], vec![2, 2])
        .solve()
        .expect("solve runs");

    assert_eq!(response.status(), RoutingStatus::Success);
    let solution = response.solution().expect("routes were found");
    let mut visited: Vec<usize> = solution.routes().iter().flatten().copied().collect();
    visited.sort_unstable();
    assert_eq!(visited, vec![1, 2, 3, 4], "every customer exactly once");
    for route in solution.routes() {
        assert!(route.len() <= 2, "capacity 2 per vehicle: {route:?}");
    }
    // Pairing nearby customers (1,2) and (3,4) is optimal: 4+2+4 + 6+2+6 = 24.
    assert_eq!(solution.objective_value(), 24);
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
    let response = RoutingProblem::from_matrix(four_city_matrix())
        .expect("square matrix")
        .solve_with_parameters(&parameters)
        .expect("solve runs");

    assert_eq!(response.status(), RoutingStatus::Success);
    let solution = response.solution().expect("a tour was found");
    assert_eq!(solution.objective_value(), 80);
}

#[test]
fn solve_with_time_limit_still_finds_the_optimum() {
    let response = RoutingProblem::from_matrix(four_city_matrix())
        .expect("square matrix")
        .solve_with_time_limit(std::time::Duration::from_secs(10))
        .expect("solve runs");

    assert_eq!(response.status(), RoutingStatus::Success);
    let solution = response.solution().expect("a tour was found");
    assert_eq!(solution.objective_value(), 80);
}

#[test]
fn time_windows_force_the_visit_order_and_report_arrivals() {
    // Depot 0 and customers 1, 2, all 5 apart. Customer 2's window closes at
    // 10 and customer 1's opens at 20: the only feasible order is 2 then 1,
    // waiting at 1 for its window.
    let matrix = vec![vec![0, 5, 5], vec![5, 0, 5], vec![5, 5, 0]];
    let time = TimeDimension::from_matrix(matrix.clone(), 40)
        .expect("square matrix")
        .with_max_waiting_time(30)
        .with_window(1, 20..=25)
        .with_window(2, 5..=10);
    let response = RoutingProblem::from_matrix(matrix)
        .expect("square matrix")
        .with_time_dimension(time)
        .solve()
        .expect("solve runs");

    assert_eq!(response.status(), RoutingStatus::Success);
    let solution = response.solution().expect("a tour was found");
    assert_eq!(solution.routes()[0], vec![2, 1]);
    let arrivals = solution.arrival_times().expect("time dimension present");
    assert_eq!(arrivals[0], vec![5, 20], "earliest feasible arrivals");
    assert_eq!(solution.objective_value(), 15);
}

#[test]
fn without_a_time_dimension_there_are_no_arrival_times() {
    let response = RoutingProblem::from_matrix(four_city_matrix())
        .expect("square matrix")
        .solve()
        .expect("solve runs");

    assert!(
        response
            .solution()
            .expect("a tour was found")
            .arrival_times()
            .is_none()
    );
}

#[test]
fn service_times_delay_later_arrivals() {
    // 0→1→2 in a line, 5 apart; 7 units of service at customer 1 push the
    // arrival at 2 from 10 to 17.
    let matrix = vec![vec![0, 5, 10], vec![5, 0, 5], vec![10, 5, 0]];
    let time = TimeDimension::from_matrix(matrix.clone(), 60)
        .expect("square matrix")
        .with_service_times(vec![0, 7, 0])
        // Pin the order with windows so the golden arrivals are stable.
        .with_window(1, 0..=6)
        .with_window(2, 0..=30)
        .with_max_waiting_time(0);
    let response = RoutingProblem::from_matrix(matrix)
        .expect("square matrix")
        .with_time_dimension(time)
        .solve()
        .expect("solve runs");

    let solution = response.solution().expect("a tour was found");
    assert_eq!(solution.routes()[0], vec![1, 2]);
    assert_eq!(
        solution.arrival_times().expect("time dimension")[0],
        vec![5, 17],
    );
}

#[test]
fn pickup_before_delivery_on_the_same_vehicle() {
    // Depot plus four nodes; node 3 is picked up and delivered to node 1.
    let matrix = vec![
        vec![0, 4, 4, 6, 6],
        vec![4, 0, 2, 8, 8],
        vec![4, 2, 0, 8, 8],
        vec![6, 8, 8, 0, 2],
        vec![6, 8, 8, 2, 0],
    ];
    let response = RoutingProblem::from_matrix(matrix)
        .expect("square matrix")
        .with_vehicles(2)
        .with_pickup_deliveries([(3, 1)])
        .solve()
        .expect("solve runs");

    assert_eq!(response.status(), RoutingStatus::Success);
    let solution = response.solution().expect("routes were found");
    let carrier: &Vec<usize> = solution
        .routes()
        .iter()
        .find(|route| route.contains(&3))
        .expect("some vehicle serves the pickup");
    let pickup_position = carrier.iter().position(|&node| node == 3).expect("pickup");
    let delivery_position = carrier
        .iter()
        .position(|&node| node == 1)
        .expect("the same vehicle must serve the delivery");
    assert!(
        pickup_position < delivery_position,
        "pickup must precede delivery: {carrier:?}",
    );
}

#[test]
fn vehicle_fixed_costs_consolidate_routes() {
    // Two cheap-to-visit customers and two vehicles: a 1000-per-vehicle
    // fixed cost makes one combined tour beat two direct round trips.
    let matrix = vec![vec![0, 5, 5], vec![5, 0, 5], vec![5, 5, 0]];
    let response = RoutingProblem::from_matrix(matrix)
        .expect("square matrix")
        .with_vehicles(2)
        .with_vehicle_fixed_costs(vec![1000, 1000])
        .solve()
        .expect("solve runs");

    let solution = response.solution().expect("routes were found");
    let used_vehicles = solution
        .routes()
        .iter()
        .filter(|route| !route.is_empty())
        .count();
    assert_eq!(used_vehicles, 1, "{:?}", solution.routes());
    // One tour of length 15 plus a single fixed cost.
    assert_eq!(solution.objective_value(), 1015);
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
    let response = RoutingProblem::from_matrix(matrix)
        .expect("square matrix")
        .with_capacities(vec![0, 1, 1, 1, 1], vec![1])
        .solve()
        .expect("the solve itself runs");

    assert!(response.solution().is_none());
    assert_ne!(response.status(), RoutingStatus::Success);
}

#[test]
fn a_single_node_problem_yields_an_empty_route() {
    let response = RoutingProblem::from_matrix(vec![vec![0]])
        .expect("square matrix")
        .solve()
        .expect("solve runs");

    // The trivial instance is even proven optimal.
    assert_eq!(response.status(), RoutingStatus::Optimal);
    let solution = response.solution().expect("the trivial tour exists");
    assert_eq!(solution.objective_value(), 0);
    assert_eq!(solution.routes(), &[Vec::<usize>::new()]);
}
