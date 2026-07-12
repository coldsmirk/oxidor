/// A failure to define or run a routing solve.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum RoutingError {
    /// The problem definition is inconsistent (non-square matrix, depot out
    /// of range, mismatched demand/capacity lengths, negative costs, …).
    InvalidProblem(String),
    /// The native layer reported an error (message from the C++ side).
    Native(String),
}

impl std::fmt::Display for RoutingError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidProblem(message) => {
                write!(formatter, "invalid routing problem: {message}")
            }
            Self::Native(message) => write!(formatter, "routing solve failed natively: {message}"),
        }
    }
}

impl std::error::Error for RoutingError {}

/// A vehicle routing problem over a dense arc-cost matrix: TSP with one
/// vehicle, capacitated VRP with several.
///
/// ```no_run
/// use oxidor_routing::RoutingProblem;
///
/// let matrix = vec![
///     vec![0, 10, 15, 20],
///     vec![10, 0, 35, 25],
///     vec![15, 35, 0, 30],
///     vec![20, 25, 30, 0],
/// ];
/// let response = RoutingProblem::from_matrix(matrix)?.solve()?;
/// let tour = response.solution().expect("a tour was found");
/// println!("tour cost {}: {:?}", tour.objective_value(), tour.routes()[0]);
/// # Ok::<(), oxidor_routing::RoutingError>(())
/// ```
#[derive(Debug, Clone)]
// Some fields are only read by the solve module (`solve` feature).
#[cfg_attr(not(feature = "solve"), allow(dead_code))]
pub struct RoutingProblem {
    pub(crate) matrix: Vec<i64>,
    pub(crate) num_nodes: usize,
    pub(crate) num_vehicles: usize,
    pub(crate) depot: usize,
    pub(crate) demands: Option<Vec<i64>>,
    pub(crate) vehicle_capacities: Option<Vec<i64>>,
    pub(crate) vehicle_fixed_costs: Option<Vec<i64>>,
    pub(crate) time: Option<TimeDimension>,
    pub(crate) pickup_deliveries: Vec<(usize, usize)>,
}

/// A travel-time dimension for a [`RoutingProblem`]: per-arc travel times,
/// optional per-node service times and time windows, and a horizon bounding
/// every route's total time.
///
/// ```
/// use oxidor_routing::TimeDimension;
///
/// let travel = vec![vec![0, 2, 4], vec![2, 0, 3], vec![4, 3, 0]];
/// let time = TimeDimension::from_matrix(travel, 30)?
///     .with_service_times(vec![0, 5, 5])
///     .with_max_waiting_time(10)
///     .with_window(1, 0..=10)
///     .with_window(2, 12..=20);
/// # Ok::<(), oxidor_routing::RoutingError>(())
/// ```
#[derive(Debug, Clone)]
// Some fields are only read by the solve module (`solve` feature).
#[cfg_attr(not(feature = "solve"), allow(dead_code))]
pub struct TimeDimension {
    pub(crate) travel_times: Vec<i64>,
    pub(crate) num_nodes: usize,
    pub(crate) service_times: Option<Vec<i64>>,
    pub(crate) windows: Vec<(usize, i64, i64)>,
    pub(crate) horizon: i64,
    pub(crate) max_waiting_time: i64,
}

impl TimeDimension {
    /// A time dimension over the given square travel-time matrix
    /// (`travel_times[from][to]`, non-negative). `horizon` bounds every
    /// route's cumulative time — arrivals, waiting, and service included.
    pub fn from_matrix(travel_times: Vec<Vec<i64>>, horizon: i64) -> Result<Self, RoutingError> {
        let num_nodes = travel_times.len();
        let mut flat = Vec::with_capacity(num_nodes * num_nodes);
        for (index, row) in travel_times.iter().enumerate() {
            if row.len() != num_nodes {
                return Err(RoutingError::InvalidProblem(format!(
                    "travel-time row {index} has {} entries, expected {num_nodes}",
                    row.len(),
                )));
            }
            if let Some(time) = row.iter().find(|&&time| time < 0) {
                return Err(RoutingError::InvalidProblem(format!(
                    "travel-time row {index} contains the negative time {time}",
                )));
            }
            flat.extend_from_slice(row);
        }
        if horizon < 0 {
            return Err(RoutingError::InvalidProblem(format!(
                "negative time horizon {horizon}",
            )));
        }
        Ok(Self {
            travel_times: flat,
            num_nodes,
            service_times: None,
            windows: Vec::new(),
            horizon,
            max_waiting_time: 0,
        })
    }

    /// Sets the per-node service durations, added to the clock when leaving
    /// each node (default zero).
    pub fn with_service_times(mut self, service_times: Vec<i64>) -> Self {
        self.service_times = Some(service_times);
        self
    }

    /// Allows vehicles to wait up to `max_waiting_time` at each node for a
    /// time window to open (default 0 — no waiting).
    pub fn with_max_waiting_time(mut self, max_waiting_time: i64) -> Self {
        self.max_waiting_time = max_waiting_time;
        self
    }

    /// Constrains the arrival at `node` to the given window. A window on the
    /// depot constrains when each vehicle may leave it. Nodes without a
    /// window accept arrivals anywhere in `0..=horizon`.
    pub fn with_window(mut self, node: usize, window: std::ops::RangeInclusive<i64>) -> Self {
        self.windows.push((node, *window.start(), *window.end()));
        self
    }

    /// Validated against the owning problem's node count.
    pub(crate) fn validate(&self, num_nodes: usize) -> Result<(), RoutingError> {
        if self.num_nodes != num_nodes {
            return Err(RoutingError::InvalidProblem(format!(
                "the travel-time matrix covers {} nodes, the problem has {num_nodes}",
                self.num_nodes,
            )));
        }
        if let Some(service_times) = &self.service_times {
            if service_times.len() != num_nodes {
                return Err(RoutingError::InvalidProblem(format!(
                    "{} service times for {num_nodes} nodes",
                    service_times.len(),
                )));
            }
            if let Some(time) = service_times.iter().find(|&&time| time < 0) {
                return Err(RoutingError::InvalidProblem(format!(
                    "negative service time {time}",
                )));
            }
        }
        if self.max_waiting_time < 0 {
            return Err(RoutingError::InvalidProblem(format!(
                "negative max waiting time {}",
                self.max_waiting_time,
            )));
        }
        for &(node, start, end) in &self.windows {
            if node >= num_nodes {
                return Err(RoutingError::InvalidProblem(format!(
                    "time window on node {node}, out of range for {num_nodes} nodes",
                )));
            }
            if start < 0 || start > end {
                return Err(RoutingError::InvalidProblem(format!(
                    "invalid time window [{start}, {end}] on node {node}",
                )));
            }
        }
        Ok(())
    }
}

impl RoutingProblem {
    /// A problem over the given square arc-cost matrix (`matrix[from][to]`,
    /// non-negative costs), with one vehicle based at node 0 until configured
    /// otherwise.
    pub fn from_matrix(matrix: Vec<Vec<i64>>) -> Result<Self, RoutingError> {
        let num_nodes = matrix.len();
        if num_nodes == 0 {
            return Err(RoutingError::InvalidProblem("the matrix is empty".into()));
        }
        let mut flat = Vec::with_capacity(num_nodes * num_nodes);
        for (index, row) in matrix.iter().enumerate() {
            if row.len() != num_nodes {
                return Err(RoutingError::InvalidProblem(format!(
                    "matrix row {index} has {} entries, expected {num_nodes}",
                    row.len(),
                )));
            }
            if let Some(cost) = row.iter().find(|&&cost| cost < 0) {
                return Err(RoutingError::InvalidProblem(format!(
                    "matrix row {index} contains the negative cost {cost}",
                )));
            }
            flat.extend_from_slice(row);
        }
        Ok(Self {
            matrix: flat,
            num_nodes,
            num_vehicles: 1,
            depot: 0,
            demands: None,
            vehicle_capacities: None,
            vehicle_fixed_costs: None,
            time: None,
            pickup_deliveries: Vec::new(),
        })
    }

    /// Sets the vehicle count (default 1).
    pub fn with_vehicles(mut self, num_vehicles: usize) -> Self {
        self.num_vehicles = num_vehicles;
        self
    }

    /// Sets the depot node every route starts and ends at (default 0).
    pub fn with_depot(mut self, depot: usize) -> Self {
        self.depot = depot;
        self
    }

    /// Adds a capacity dimension: `demands[node]` units are picked up at each
    /// visited node, and vehicle `v` carries at most `vehicle_capacities[v]`.
    /// Both must be non-negative.
    pub fn with_capacities(mut self, demands: Vec<i64>, vehicle_capacities: Vec<i64>) -> Self {
        self.demands = Some(demands);
        self.vehicle_capacities = Some(vehicle_capacities);
        self
    }

    /// Adds a fixed cost to the objective for each vehicle that leaves the
    /// depot (one non-negative entry per vehicle) — the lever for "use as few
    /// vehicles as possible".
    pub fn with_vehicle_fixed_costs(mut self, costs: Vec<i64>) -> Self {
        self.vehicle_fixed_costs = Some(costs);
        self
    }

    /// Adds a travel-time dimension (see [`TimeDimension`]); with it, the
    /// solution reports per-node arrival times.
    pub fn with_time_dimension(mut self, time: TimeDimension) -> Self {
        self.time = Some(time);
        self
    }

    /// Adds pickup-and-delivery pairs: each pair's two nodes are served by
    /// the same vehicle, pickup first. A node may appear in at most one
    /// pair, and the depot in none.
    pub fn with_pickup_deliveries(
        mut self,
        pairs: impl IntoIterator<Item = (usize, usize)>,
    ) -> Self {
        self.pickup_deliveries.extend(pairs);
        self
    }

    // Called by the solve module (`solve` feature) and by unit tests.
    #[cfg_attr(not(any(feature = "solve", test)), allow(dead_code))]
    pub(crate) fn validate(&self) -> Result<(), RoutingError> {
        if self.num_vehicles == 0 {
            return Err(RoutingError::InvalidProblem("no vehicles".into()));
        }
        if self.depot >= self.num_nodes {
            return Err(RoutingError::InvalidProblem(format!(
                "depot {} out of range for {} nodes",
                self.depot, self.num_nodes,
            )));
        }
        if let Some(demands) = &self.demands {
            if demands.len() != self.num_nodes {
                return Err(RoutingError::InvalidProblem(format!(
                    "{} demands for {} nodes",
                    demands.len(),
                    self.num_nodes,
                )));
            }
            if let Some(demand) = demands.iter().find(|&&demand| demand < 0) {
                return Err(RoutingError::InvalidProblem(format!(
                    "negative demand {demand}",
                )));
            }
            let capacities = self
                .vehicle_capacities
                .as_ref()
                .expect("with_capacities sets both");
            if capacities.len() != self.num_vehicles {
                return Err(RoutingError::InvalidProblem(format!(
                    "{} vehicle capacities for {} vehicles",
                    capacities.len(),
                    self.num_vehicles,
                )));
            }
            // Upstream CHECK-aborts the whole process on a negative capacity;
            // reject it here, where it can be an ordinary error.
            if let Some(capacity) = capacities.iter().find(|&&capacity| capacity < 0) {
                return Err(RoutingError::InvalidProblem(format!(
                    "negative vehicle capacity {capacity}",
                )));
            }
        }
        if let Some(costs) = &self.vehicle_fixed_costs {
            if costs.len() != self.num_vehicles {
                return Err(RoutingError::InvalidProblem(format!(
                    "{} vehicle fixed costs for {} vehicles",
                    costs.len(),
                    self.num_vehicles,
                )));
            }
            // Upstream CHECK-aborts the whole process on a negative fixed
            // cost; reject it here, where it can be an ordinary error.
            if let Some(cost) = costs.iter().find(|&&cost| cost < 0) {
                return Err(RoutingError::InvalidProblem(format!(
                    "negative vehicle fixed cost {cost}",
                )));
            }
        }
        if let Some(time) = &self.time {
            time.validate(self.num_nodes)?;
        }
        if !self.pickup_deliveries.is_empty() {
            let mut seen = std::collections::HashSet::new();
            for &(pickup, delivery) in &self.pickup_deliveries {
                for node in [pickup, delivery] {
                    if node >= self.num_nodes {
                        return Err(RoutingError::InvalidProblem(format!(
                            "pickup-delivery node {node} out of range for {} nodes",
                            self.num_nodes,
                        )));
                    }
                    if node == self.depot {
                        return Err(RoutingError::InvalidProblem(
                            "the depot cannot be a pickup or delivery node".into(),
                        ));
                    }
                    if !seen.insert(node) {
                        return Err(RoutingError::InvalidProblem(format!(
                            "node {node} appears in more than one pickup-delivery role",
                        )));
                    }
                }
            }
            if self.time.is_none() {
                // Without a time dimension the shim derives the ordering
                // dimension's horizon by summing the cost matrix; that sum
                // must not overflow.
                let mut total: i64 = 0;
                for &cost in &self.matrix {
                    total = total.checked_add(cost).ok_or_else(|| {
                        RoutingError::InvalidProblem(
                            "the cost matrix sum overflows i64; add a time dimension to order pickups"
                                .into(),
                        )
                    })?;
                }
            }
        }
        let node_count = i32::try_from(self.num_nodes);
        let vehicle_count = i32::try_from(self.num_vehicles);
        if node_count.is_err() || vehicle_count.is_err() {
            return Err(RoutingError::InvalidProblem(
                "node or vehicle count exceeds i32".into(),
            ));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_a_ragged_matrix() {
        let error = RoutingProblem::from_matrix(vec![vec![0, 1], vec![1]]).unwrap_err();
        assert!(matches!(error, RoutingError::InvalidProblem(_)));
    }

    #[test]
    fn rejects_an_empty_matrix() {
        let error = RoutingProblem::from_matrix(vec![]).unwrap_err();
        assert!(matches!(error, RoutingError::InvalidProblem(_)));
    }

    #[test]
    fn rejects_negative_costs() {
        let error = RoutingProblem::from_matrix(vec![vec![0, -5], vec![1, 0]]).unwrap_err();
        assert!(matches!(error, RoutingError::InvalidProblem(_)));
    }

    #[test]
    fn rejects_an_out_of_range_depot() {
        let problem = RoutingProblem::from_matrix(vec![vec![0, 1], vec![1, 0]])
            .unwrap()
            .with_depot(5);
        assert!(matches!(
            problem.validate(),
            Err(RoutingError::InvalidProblem(_))
        ));
    }

    #[test]
    fn rejects_zero_vehicles() {
        let problem = RoutingProblem::from_matrix(vec![vec![0, 1], vec![1, 0]])
            .unwrap()
            .with_vehicles(0);
        assert!(matches!(
            problem.validate(),
            Err(RoutingError::InvalidProblem(_))
        ));
    }

    #[test]
    fn rejects_mismatched_capacity_lengths() {
        let problem = RoutingProblem::from_matrix(vec![vec![0, 1], vec![1, 0]])
            .unwrap()
            .with_capacities(vec![1], vec![10]);
        assert!(matches!(
            problem.validate(),
            Err(RoutingError::InvalidProblem(_))
        ));
    }

    #[test]
    fn rejects_a_negative_vehicle_capacity() {
        // Upstream would CHECK-abort the process on this; it must be caught
        // before the FFI boundary.
        let problem = RoutingProblem::from_matrix(vec![vec![0, 1], vec![1, 0]])
            .unwrap()
            .with_capacities(vec![0, 1], vec![-2]);
        assert!(matches!(
            problem.validate(),
            Err(RoutingError::InvalidProblem(_))
        ));
    }

    #[test]
    fn rejects_a_negative_demand() {
        let problem = RoutingProblem::from_matrix(vec![vec![0, 1], vec![1, 0]])
            .unwrap()
            .with_capacities(vec![0, -1], vec![2]);
        assert!(matches!(
            problem.validate(),
            Err(RoutingError::InvalidProblem(_))
        ));
    }

    fn two_node_problem() -> RoutingProblem {
        RoutingProblem::from_matrix(vec![vec![0, 1], vec![1, 0]]).unwrap()
    }

    #[test]
    fn rejects_mismatched_fixed_cost_lengths() {
        let problem = two_node_problem().with_vehicle_fixed_costs(vec![5, 5]);
        assert!(matches!(
            problem.validate(),
            Err(RoutingError::InvalidProblem(_))
        ));
    }

    #[test]
    fn rejects_a_negative_fixed_cost() {
        // Upstream would CHECK-abort the process on this; it must be caught
        // before the FFI boundary.
        let problem = two_node_problem().with_vehicle_fixed_costs(vec![-1]);
        assert!(matches!(
            problem.validate(),
            Err(RoutingError::InvalidProblem(_))
        ));
    }

    #[test]
    fn time_dimension_rejects_a_ragged_matrix() {
        assert!(matches!(
            TimeDimension::from_matrix(vec![vec![0, 1], vec![1]], 10),
            Err(RoutingError::InvalidProblem(_))
        ));
    }

    #[test]
    fn time_dimension_rejects_negative_times() {
        assert!(matches!(
            TimeDimension::from_matrix(vec![vec![0, -1], vec![1, 0]], 10),
            Err(RoutingError::InvalidProblem(_))
        ));
    }

    #[test]
    fn time_dimension_rejects_a_negative_horizon() {
        assert!(matches!(
            TimeDimension::from_matrix(vec![vec![0]], -1),
            Err(RoutingError::InvalidProblem(_))
        ));
    }

    #[test]
    fn rejects_a_time_matrix_of_the_wrong_size() {
        let time = TimeDimension::from_matrix(vec![vec![0]], 10).unwrap();
        let problem = two_node_problem().with_time_dimension(time);
        assert!(matches!(
            problem.validate(),
            Err(RoutingError::InvalidProblem(_))
        ));
    }

    #[test]
    fn rejects_an_out_of_range_time_window() {
        let time = TimeDimension::from_matrix(vec![vec![0, 1], vec![1, 0]], 10)
            .unwrap()
            .with_window(5, 0..=3);
        let problem = two_node_problem().with_time_dimension(time);
        assert!(matches!(
            problem.validate(),
            Err(RoutingError::InvalidProblem(_))
        ));
    }

    #[test]
    fn rejects_an_inverted_time_window() {
        #[allow(clippy::reversed_empty_ranges)]
        let inverted = 8..=3;
        let time = TimeDimension::from_matrix(vec![vec![0, 1], vec![1, 0]], 10)
            .unwrap()
            .with_window(1, inverted);
        let problem = two_node_problem().with_time_dimension(time);
        assert!(matches!(
            problem.validate(),
            Err(RoutingError::InvalidProblem(_))
        ));
    }

    #[test]
    fn rejects_mismatched_service_time_lengths() {
        let time = TimeDimension::from_matrix(vec![vec![0, 1], vec![1, 0]], 10)
            .unwrap()
            .with_service_times(vec![0]);
        let problem = two_node_problem().with_time_dimension(time);
        assert!(matches!(
            problem.validate(),
            Err(RoutingError::InvalidProblem(_))
        ));
    }

    #[test]
    fn rejects_the_depot_in_a_pickup_pair() {
        let problem = two_node_problem().with_pickup_deliveries([(0, 1)]);
        assert!(matches!(
            problem.validate(),
            Err(RoutingError::InvalidProblem(_))
        ));
    }

    #[test]
    fn rejects_a_node_in_two_pickup_roles() {
        let matrix = vec![vec![0; 4]; 4];
        let problem = RoutingProblem::from_matrix(matrix)
            .unwrap()
            .with_pickup_deliveries([(1, 2), (2, 3)]);
        assert!(matches!(
            problem.validate(),
            Err(RoutingError::InvalidProblem(_))
        ));
    }
}
