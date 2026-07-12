use core::ffi::c_char;

use crate::error::{AlgorithmError, take_error_message};

/// An arc handle returned by [`MaxFlow::add_arc`] /
/// [`MinCostFlow::add_arc`], used to read the arc's flow from a solution.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ArcId(pub(crate) usize);

/// Checks the upstream flow contracts (`max_flow.h` / `min_cost_flow.h`):
/// non-negative node indices that fit an `i32`, non-negative capacities, and
/// an arc count within `i32`.
fn validate_arcs(
    tails: &[u32],
    heads: &[u32],
    capacities: &[i64],
) -> Result<(Vec<i32>, Vec<i32>), AlgorithmError> {
    if i32::try_from(tails.len()).is_err() {
        return Err(AlgorithmError::InvalidInput(format!(
            "{} arcs exceed the i32 range",
            tails.len(),
        )));
    }
    let node = |value: u32| -> Result<i32, AlgorithmError> {
        i32::try_from(value)
            .map_err(|_| AlgorithmError::InvalidInput(format!("node index {value} exceeds i32")))
    };
    let tails = tails
        .iter()
        .map(|&tail| node(tail))
        .collect::<Result<_, _>>()?;
    let heads = heads
        .iter()
        .map(|&head| node(head))
        .collect::<Result<_, _>>()?;
    if let Some(capacity) = capacities.iter().find(|&&capacity| capacity < 0) {
        return Err(AlgorithmError::InvalidInput(format!(
            "negative arc capacity {capacity}",
        )));
    }
    Ok((tails, heads))
}

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
    tails: Vec<u32>,
    heads: Vec<u32>,
    capacities: Vec<i64>,
}

impl MaxFlow {
    /// An empty graph.
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds a directed arc with the given capacity. Node indices must fit an
    /// `i32` and the capacity must be non-negative — both checked by
    /// [`solve`](Self::solve).
    pub fn add_arc(&mut self, tail: u32, head: u32, capacity: i64) -> ArcId {
        self.tails.push(tail);
        self.heads.push(head);
        self.capacities.push(capacity);
        ArcId(self.tails.len() - 1)
    }

    /// Computes the maximum flow from `source` to `sink`.
    pub fn solve(&self, source: u32, sink: u32) -> Result<MaxFlowSolution, AlgorithmError> {
        let (tails, heads) = validate_arcs(&self.tails, &self.heads, &self.capacities)?;
        if i32::try_from(source).is_err() || i32::try_from(sink).is_err() {
            return Err(AlgorithmError::InvalidInput(
                "source or sink index exceeds i32".into(),
            ));
        }
        let num_arcs = tails.len();
        let mut flows = vec![0i64; num_arcs];
        let mut maximum_flow: i64 = 0;
        let mut error_message: *mut c_char = core::ptr::null_mut();
        // SAFETY: the arc arrays share one length; output buffers are
        // writable for the stated sizes.
        let status = unsafe {
            oxidor_sys::OxidorMaxFlowSolve(
                tails.as_ptr(),
                heads.as_ptr(),
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
            -1 => Err(AlgorithmError::Native(unsafe {
                take_error_message(error_message)
            })),
            1 => Err(AlgorithmError::Native(
                "flow exceeds the int64 range".into(),
            )),
            _ => Err(AlgorithmError::Native(format!(
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
    pub fn flow(&self, arc: ArcId) -> i64 {
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
/// let response = graph.solve()?;
/// let solution = response.solution().expect("feasible and balanced");
/// assert_eq!(solution.total_cost(), 15);
/// assert_eq!(solution.flow(arc), 5);
/// # Ok::<(), oxidor_algorithms::AlgorithmError>(())
/// ```
#[derive(Debug, Clone, Default)]
pub struct MinCostFlow {
    tails: Vec<u32>,
    heads: Vec<u32>,
    capacities: Vec<i64>,
    unit_costs: Vec<i64>,
    supplies: Vec<i64>,
}

impl MinCostFlow {
    /// An empty graph.
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds a directed arc with a capacity and a per-unit cost. Node indices
    /// must fit an `i32` and the capacity must be non-negative — both checked
    /// by [`solve`](Self::solve); the unit cost may be negative (upstream
    /// allows it).
    pub fn add_arc(&mut self, tail: u32, head: u32, capacity: i64, unit_cost: i64) -> ArcId {
        self.tails.push(tail);
        self.heads.push(head);
        self.capacities.push(capacity);
        self.unit_costs.push(unit_cost);
        let highest = tail.max(head) as usize;
        if self.supplies.len() <= highest {
            self.supplies.resize(highest + 1, 0);
        }
        ArcId(self.tails.len() - 1)
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
    /// outcomes on the returned response, not errors.
    pub fn solve(&self) -> Result<MinCostFlowResponse, AlgorithmError> {
        let (tails, heads) = validate_arcs(&self.tails, &self.heads, &self.capacities)?;
        if i32::try_from(self.supplies.len()).is_err() {
            return Err(AlgorithmError::InvalidInput(format!(
                "{} nodes exceed the i32 range",
                self.supplies.len(),
            )));
        }
        let num_arcs = tails.len();
        let mut flows = vec![0i64; num_arcs];
        let mut total_cost: i64 = 0;
        let mut error_message: *mut c_char = core::ptr::null_mut();
        // SAFETY: the arc arrays share one length; supplies covers every node
        // referenced by an arc (maintained by add_arc/set_node_supply).
        let status = unsafe {
            oxidor_sys::OxidorMinCostFlowSolve(
                tails.as_ptr(),
                heads.as_ptr(),
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
                return Err(AlgorithmError::Native(unsafe {
                    take_error_message(error_message)
                }));
            }
            other => {
                return Err(AlgorithmError::Native(format!(
                    "SimpleMinCostFlow failed (status {other})",
                )));
            }
        };
        Ok(MinCostFlowResponse {
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

/// The outcome of a [`MinCostFlow`] solve: a status and — when optimal — a
/// [`MinCostFlowSolution`].
#[derive(Debug, Clone)]
pub struct MinCostFlowResponse {
    status: MinCostFlowStatus,
    total_cost: i64,
    flows: Vec<i64>,
}

impl MinCostFlowResponse {
    /// How the solve ended.
    pub fn status(&self) -> MinCostFlowStatus {
        self.status
    }

    /// The optimal flow, when one was found.
    pub fn solution(&self) -> Option<MinCostFlowSolution<'_>> {
        (self.status == MinCostFlowStatus::Optimal).then_some(MinCostFlowSolution {
            total_cost: self.total_cost,
            flows: &self.flows,
        })
    }
}

/// An optimal flow, borrowed from a [`MinCostFlowResponse`].
#[derive(Debug, Clone, Copy)]
pub struct MinCostFlowSolution<'response> {
    total_cost: i64,
    flows: &'response [i64],
}

impl MinCostFlowSolution<'_> {
    /// The total cost of the flow.
    pub fn total_cost(&self) -> i64 {
        self.total_cost
    }

    /// The flow assigned to an arc.
    pub fn flow(&self, arc: ArcId) -> i64 {
        self.flows[arc.0]
    }
}
