use core::ffi::c_char;

use crate::error::{AlgorithmError, take_error_message};

/// An arc handle returned by [`MaxFlow::add_arc`] /
/// [`MinCostFlow::add_arc`], used to read the arc's flow from a solution.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Arc(pub(crate) usize);

/// A maximum-flow problem on a directed graph with arc capacities.
///
/// ```no_run
/// use oxidor_algorithms::MaxFlow;
///
/// let mut graph = MaxFlow::new();
/// let direct = graph.add_arc(0, 1, 3);
/// graph.add_arc(0, 2, 2);
/// graph.add_arc(1, 2, 1);
/// graph.add_arc(1, 3, 2);
/// graph.add_arc(2, 3, 3);
///
/// let solution = graph.solve(0, 3)?;
/// assert_eq!(solution.maximum_flow(), 5);
/// assert_eq!(solution.flow(direct), 3);
/// # Ok::<(), oxidor_algorithms::AlgorithmError>(())
/// ```
#[derive(Debug, Clone, Default)]
pub struct MaxFlow {
    tails: Vec<i32>,
    heads: Vec<i32>,
    capacities: Vec<i64>,
}

impl MaxFlow {
    /// An empty graph.
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds a directed arc with the given capacity; nodes are arbitrary
    /// non-negative indices.
    pub fn add_arc(&mut self, tail: u32, head: u32, capacity: i64) -> Arc {
        self.tails.push(tail as i32);
        self.heads.push(head as i32);
        self.capacities.push(capacity);
        Arc(self.tails.len() - 1)
    }

    /// Computes the maximum flow from `source` to `sink`.
    pub fn solve(&self, source: u32, sink: u32) -> Result<MaxFlowSolution, AlgorithmError> {
        let num_arcs = self.tails.len();
        let mut flows = vec![0i64; num_arcs];
        let mut maximum_flow: i64 = 0;
        let mut error_message: *mut c_char = core::ptr::null_mut();
        // SAFETY: the arc arrays share one length; output buffers are
        // writable for the stated sizes.
        let status = unsafe {
            oxidor_sys::OxidorMaxFlowSolve(
                self.tails.as_ptr(),
                self.heads.as_ptr(),
                self.capacities.as_ptr(),
                num_arcs as i32,
                source as i32,
                sink as i32,
                flows.as_mut_ptr(),
                &mut maximum_flow,
                &mut error_message,
            )
        };
        match status {
            // SimpleMaxFlow::OPTIMAL — max flow always has an optimum, so
            // everything else is a genuine error.
            0 => Ok(MaxFlowSolution {
                maximum_flow,
                flows,
            }),
            // SAFETY: -1 means the shim caught an exception and set the message.
            -1 => Err(AlgorithmError::new(unsafe {
                take_error_message(error_message)
            })),
            1 => Err(AlgorithmError::new("flow exceeds the int64 range")),
            _ => Err(AlgorithmError::new(format!(
                "bad input or result (SimpleMaxFlow status {status})",
            ))),
        }
    }
}

/// The result of a [`MaxFlow`] solve.
#[derive(Debug, Clone)]
pub struct MaxFlowSolution {
    maximum_flow: i64,
    flows: Vec<i64>,
}

impl MaxFlowSolution {
    /// The value of the maximum flow (0 when the sink is unreachable).
    pub fn maximum_flow(&self) -> i64 {
        self.maximum_flow
    }

    /// The flow assigned to an arc.
    pub fn flow(&self, arc: Arc) -> i64 {
        self.flows[arc.0]
    }
}

/// A minimum-cost-flow problem: route flow from supply nodes to demand nodes
/// at minimal total cost.
///
/// ```no_run
/// use oxidor_algorithms::MinCostFlow;
///
/// let mut graph = MinCostFlow::new();
/// let arc = graph.add_arc(0, 1, 10, 3);
/// graph.set_node_supply(0, 5);
/// graph.set_node_supply(1, -5);
///
/// let solution = graph.solve()?;
/// assert!(solution.is_optimal());
/// assert_eq!(solution.total_cost(), 15);
/// assert_eq!(solution.flow(arc), 5);
/// # Ok::<(), oxidor_algorithms::AlgorithmError>(())
/// ```
#[derive(Debug, Clone, Default)]
pub struct MinCostFlow {
    tails: Vec<i32>,
    heads: Vec<i32>,
    capacities: Vec<i64>,
    unit_costs: Vec<i64>,
    supplies: Vec<i64>,
}

impl MinCostFlow {
    /// An empty graph.
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds a directed arc with a capacity and a per-unit cost.
    pub fn add_arc(&mut self, tail: u32, head: u32, capacity: i64, unit_cost: i64) -> Arc {
        self.tails.push(tail as i32);
        self.heads.push(head as i32);
        self.capacities.push(capacity);
        self.unit_costs.push(unit_cost);
        let highest = tail.max(head) as usize;
        if self.supplies.len() <= highest {
            self.supplies.resize(highest + 1, 0);
        }
        Arc(self.tails.len() - 1)
    }

    /// Sets a node's supply: positive to inject flow, negative to demand it.
    pub fn set_node_supply(&mut self, node: u32, supply: i64) {
        let node = node as usize;
        if self.supplies.len() <= node {
            self.supplies.resize(node + 1, 0);
        }
        self.supplies[node] = supply;
    }

    /// Solves the problem. Infeasibility and unbalanced supplies are
    /// outcomes on the returned solution, not errors.
    pub fn solve(&self) -> Result<MinCostFlowSolution, AlgorithmError> {
        let num_arcs = self.tails.len();
        let mut flows = vec![0i64; num_arcs];
        let mut total_cost: i64 = 0;
        let mut error_message: *mut c_char = core::ptr::null_mut();
        // SAFETY: the arc arrays share one length; supplies covers every node
        // referenced by an arc (maintained by add_arc/set_node_supply).
        let status = unsafe {
            oxidor_sys::OxidorMinCostFlowSolve(
                self.tails.as_ptr(),
                self.heads.as_ptr(),
                self.capacities.as_ptr(),
                self.unit_costs.as_ptr(),
                num_arcs as i32,
                self.supplies.as_ptr(),
                self.supplies.len() as i32,
                flows.as_mut_ptr(),
                &mut total_cost,
                &mut error_message,
            )
        };
        let status = match status {
            1 => MinCostFlowStatus::Optimal,
            3 => MinCostFlowStatus::Infeasible,
            4 => MinCostFlowStatus::Unbalanced,
            // SAFETY: -1 means the shim caught an exception and set the message.
            -1 => {
                return Err(AlgorithmError::new(unsafe {
                    take_error_message(error_message)
                }));
            }
            other => {
                return Err(AlgorithmError::new(format!(
                    "SimpleMinCostFlow failed (status {other})",
                )));
            }
        };
        Ok(MinCostFlowSolution {
            status,
            total_cost,
            flows,
        })
    }
}

/// How a [`MinCostFlow`] solve ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum MinCostFlowStatus {
    /// A minimum-cost flow satisfying every supply and demand was found.
    Optimal,
    /// Supplies and demands cannot be satisfied with the given capacities.
    Infeasible,
    /// Total supply does not equal total demand.
    Unbalanced,
}

/// The result of a [`MinCostFlow`] solve.
#[derive(Debug, Clone)]
pub struct MinCostFlowSolution {
    status: MinCostFlowStatus,
    total_cost: i64,
    flows: Vec<i64>,
}

impl MinCostFlowSolution {
    /// How the solve ended.
    pub fn status(&self) -> MinCostFlowStatus {
        self.status
    }

    /// Whether an optimal flow was found.
    pub fn is_optimal(&self) -> bool {
        self.status == MinCostFlowStatus::Optimal
    }

    /// The total cost of the flow (0 unless [`is_optimal`](Self::is_optimal)).
    pub fn total_cost(&self) -> i64 {
        self.total_cost
    }

    /// The flow assigned to an arc (0 unless [`is_optimal`](Self::is_optimal)).
    pub fn flow(&self, arc: Arc) -> i64 {
        self.flows[arc.0]
    }
}
