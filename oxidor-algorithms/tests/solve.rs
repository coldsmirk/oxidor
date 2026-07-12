//! End-to-end tests through the C++ shim into the algorithm classes.

use oxidor_algorithms::{
    AlgorithmError, MaxFlow, MinCostFlow, MinCostFlowStatus, solve_knapsack,
    solve_knapsack_multidimensional,
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
fn knapsack_with_no_items_packs_nothing() {
    let solution = solve_knapsack(&[], &[], 10).expect("solves");
    assert_eq!(solution.total_value(), 0);
    assert!(solution.selected_items().is_empty());
}

#[test]
fn knapsack_rejects_mismatched_lengths() {
    let error = solve_knapsack_multidimensional(&[1, 2], &[vec![1, 2, 3]], &[5]).unwrap_err();
    assert!(matches!(error, AlgorithmError::InvalidInput(_)));
}

#[test]
fn knapsack_rejects_negative_weights() {
    let error = solve_knapsack(&[10, 20], &[-5, 10], 8).unwrap_err();
    assert!(matches!(error, AlgorithmError::InvalidInput(_)));
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
fn max_flow_to_an_unreachable_sink_is_zero() {
    let mut graph = MaxFlow::new();
    graph.add_arc(0, 1, 3);
    graph.add_arc(2, 3, 3);

    let solution = graph.solve(0, 3).expect("solves");
    assert_eq!(solution.maximum_flow(), 0);
}

#[test]
fn max_flow_rejects_a_negative_capacity() {
    // Upstream requires non-negative capacities; without the check it would
    // report a meaningless negative "maximum" flow as success.
    let mut graph = MaxFlow::new();
    graph.add_arc(0, 1, -3);

    let error = graph.solve(0, 1).unwrap_err();
    assert!(matches!(error, AlgorithmError::InvalidInput(_)));
}

#[test]
fn max_flow_rejects_an_oversized_node_index() {
    let mut graph = MaxFlow::new();
    graph.add_arc(3_000_000_000, 1, 5);

    let error = graph.solve(0, 1).unwrap_err();
    assert!(matches!(error, AlgorithmError::InvalidInput(_)));
}

#[test]
fn min_cost_flow_prefers_the_cheap_arc() {
    let mut graph = MinCostFlow::new();
    let cheap = graph.add_arc(0, 1, 4, 1);
    let expensive = graph.add_arc(0, 1, 10, 5);
    graph.set_node_supply(0, 6);
    graph.set_node_supply(1, -6);

    let response = graph.solve().expect("runs");
    assert_eq!(response.status(), MinCostFlowStatus::Optimal);
    let solution = response.solution().expect("optimal");
    // 4 units over the cheap arc, 2 over the expensive one: 4*1 + 2*5 = 14.
    assert_eq!(solution.total_cost(), 14);
    assert_eq!(solution.flow(cheap), 4);
    assert_eq!(solution.flow(expensive), 2);
}

#[test]
fn min_cost_flow_matches_the_doc_example() {
    let mut graph = MinCostFlow::new();
    let arc = graph.add_arc(0, 1, 10, 3);
    graph.set_node_supply(0, 5);
    graph.set_node_supply(1, -5);

    let response = graph.solve().expect("runs");
    let solution = response.solution().expect("optimal");
    assert_eq!(solution.total_cost(), 15);
    assert_eq!(solution.flow(arc), 5);
}

#[test]
fn unbalanced_supplies_are_an_outcome() {
    let mut graph = MinCostFlow::new();
    graph.add_arc(0, 1, 10, 1);
    graph.set_node_supply(0, 5);
    graph.set_node_supply(1, -3);

    let response = graph.solve().expect("runs");
    assert_eq!(response.status(), MinCostFlowStatus::Unbalanced);
    assert!(response.solution().is_none());
}

#[test]
fn insufficient_capacity_is_infeasible() {
    let mut graph = MinCostFlow::new();
    graph.add_arc(0, 1, 3, 1);
    graph.set_node_supply(0, 5);
    graph.set_node_supply(1, -5);

    let response = graph.solve().expect("runs");
    assert_eq!(response.status(), MinCostFlowStatus::Infeasible);
    assert!(response.solution().is_none());
}

#[test]
fn min_cost_flow_rejects_a_negative_capacity() {
    let mut graph = MinCostFlow::new();
    graph.add_arc(0, 1, -4, 1);
    graph.set_node_supply(0, 1);
    graph.set_node_supply(1, -1);

    let error = graph.solve().unwrap_err();
    assert!(matches!(error, AlgorithmError::InvalidInput(_)));
}
