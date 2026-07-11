/// A failure to define or run a routing solve.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RoutingError {
    /// The problem definition is inconsistent (non-square matrix, depot out
    /// of range, mismatched demand/capacity lengths, …).
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
/// let solution = RoutingProblem::from_matrix(matrix)?.solve()?;
/// println!("tour cost {}: {:?}", solution.objective(), solution.routes());
/// # Ok::<(), oxidor_routing::RoutingError>(())
/// ```
#[derive(Debug, Clone)]
pub struct RoutingProblem {
    pub(crate) matrix: Vec<i64>,
    pub(crate) num_nodes: usize,
    pub(crate) num_vehicles: usize,
    pub(crate) depot: usize,
    pub(crate) demands: Option<Vec<i64>>,
    pub(crate) vehicle_capacities: Option<Vec<i64>>,
}

impl RoutingProblem {
    /// A problem over the given square arc-cost matrix (`matrix[from][to]`),
    /// with one vehicle based at node 0 until configured otherwise.
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
            flat.extend_from_slice(row);
        }
        Ok(Self {
            matrix: flat,
            num_nodes,
            num_vehicles: 1,
            depot: 0,
            demands: None,
            vehicle_capacities: None,
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
    pub fn with_capacities(mut self, demands: Vec<i64>, vehicle_capacities: Vec<i64>) -> Self {
        self.demands = Some(demands);
        self.vehicle_capacities = Some(vehicle_capacities);
        self
    }

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
    fn rejects_mismatched_capacity_lengths() {
        let problem = RoutingProblem::from_matrix(vec![vec![0, 1], vec![1, 0]])
            .unwrap()
            .with_capacities(vec![1], vec![10]);
        assert!(matches!(
            problem.validate(),
            Err(RoutingError::InvalidProblem(_))
        ));
    }
}
