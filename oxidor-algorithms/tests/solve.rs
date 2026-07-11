//! End-to-end tests through the C++ shim into the algorithm classes.

use oxidor_algorithms::{
    MaxFlow, MinCostFlow, MinCostFlowStatus, solve_knapsack, solve_knapsack_multidimensional,
};

#[test]
fn knapsack_finds_the_classic_optimum() {
    let solution = solve_knapsack(&[60, 100, 120], &[10, 20, 30], 50).expect("solves");
    assert_eq!(solution.total_value(), 220);
    assert_eq!(solution.selected_items(), vec![1, 2]);
    assert!(!solution.is_selected(0));
}

#[test]
fn multidimensional_knapsack_respects_every_dimension() {
    // Item 2 is valuable but too heavy in the second dimension.
    let solution =
        solve_knapsack_multidimensional(&[10, 10, 30], &[vec![1, 1, 1], vec![1, 1, 10]], &[2, 5])
            .expect("solves");
    assert_eq!(solution.total_value(), 20);
    assert_eq!(solution.selected_items(), vec![0, 1]);
}

#[test]
fn knapsack_rejects_mismatched_lengths() {
    let error = solve_knapsack_multidimensional(&[1, 2], &[vec![1, 2, 3]], &[5]).unwrap_err();
    assert!(error.message.contains("entries"));
}

#[test]
fn max_flow_reaches_the_known_maximum() {
    let mut graph = MaxFlow::new();
    let source_direct = graph.add_arc(0, 1, 3);
    graph.add_arc(0, 2, 2);
    graph.add_arc(1, 2, 1);
    graph.add_arc(1, 3, 2);
    graph.add_arc(2, 3, 3);

    let solution = graph.solve(0, 3).expect("solves");
    assert_eq!(solution.maximum_flow(), 5);
    assert_eq!(solution.flow(source_direct), 3);
}

#[test]
fn min_cost_flow_prefers_the_cheap_arc() {
    let mut graph = MinCostFlow::new();
    let cheap = graph.add_arc(0, 1, 4, 1);
    let expensive = graph.add_arc(0, 1, 10, 5);
    graph.set_node_supply(0, 6);
    graph.set_node_supply(1, -6);

    let solution = graph.solve().expect("runs");
    assert!(solution.is_optimal());
    // 4 units over the cheap arc, 2 over the expensive one: 4*1 + 2*5 = 14.
    assert_eq!(solution.total_cost(), 14);
    assert_eq!(solution.flow(cheap), 4);
    assert_eq!(solution.flow(expensive), 2);
}

#[test]
fn unbalanced_supplies_are_an_outcome() {
    let mut graph = MinCostFlow::new();
    graph.add_arc(0, 1, 10, 1);
    graph.set_node_supply(0, 5);
    graph.set_node_supply(1, -3);

    let solution = graph.solve().expect("runs");
    assert_eq!(solution.status(), MinCostFlowStatus::Unbalanced);
    assert!(!solution.is_optimal());
}
