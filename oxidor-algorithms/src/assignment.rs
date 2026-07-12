use core::ffi::c_char;

use crate::error::{AlgorithmError, take_error_message};

/// A linear sum assignment problem: match `n` left nodes to `n` right nodes
/// through cost-carrying arcs, minimizing the total cost of a perfect
/// matching.
///
/// Node indices on each side run from 0; a perfect matching needs every
/// index in `0..n` covered on both sides. Costs may be negative.
///
/// ```no_run
/// use oxidor_algorithms::LinearSumAssignment;
///
/// let mut assignment = LinearSumAssignment::new();
/// let costs = [[4, 1, 3], [2, 0, 5], [3, 2, 2]];
/// for (left, row) in costs.iter().enumerate() {
///     for (right, &cost) in row.iter().enumerate() {
///         assignment.add_arc_with_cost(left as u32, right as u32, cost);
///     }
/// }
///
/// let response = assignment.solve()?;
/// let solution = response.solution().expect("a perfect matching exists");
/// assert_eq!(solution.total_cost(), 5);
/// assert_eq!(solution.right_mate(0), 1);
/// # Ok::<(), oxidor_algorithms::AlgorithmError>(())
/// ```
#[derive(Debug, Clone, Default)]
pub struct LinearSumAssignment {
    left_nodes: Vec<u32>,
    right_nodes: Vec<u32>,
    costs: Vec<i64>,
}

impl LinearSumAssignment {
    /// An empty problem.
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds an arc assigning `left` to `right` at the given cost (any sign).
    /// Node indices must fit an `i32` — checked by [`solve`](Self::solve).
    pub fn add_arc_with_cost(&mut self, left: u32, right: u32, cost: i64) {
        self.left_nodes.push(left);
        self.right_nodes.push(right);
        self.costs.push(cost);
    }

    /// Solves the problem. A missing perfect matching or a cost overflow is
    /// an outcome on the returned response, not an error.
    pub fn solve(&self) -> Result<AssignmentResponse, AlgorithmError> {
        if i32::try_from(self.left_nodes.len()).is_err() {
            return Err(AlgorithmError::InvalidInput(format!(
                "{} arcs exceed the i32 range",
                self.left_nodes.len(),
            )));
        }
        let node = |value: u32| -> Result<i32, AlgorithmError> {
            i32::try_from(value).map_err(|_| {
                AlgorithmError::InvalidInput(format!("node index {value} exceeds i32"))
            })
        };
        let left_nodes: Vec<i32> = self
            .left_nodes
            .iter()
            .map(|&index| node(index))
            .collect::<Result<_, _>>()?;
        let right_nodes: Vec<i32> = self
            .right_nodes
            .iter()
            .map(|&index| node(index))
            .collect::<Result<_, _>>()?;

        // Upstream's NumNodes(): one greater than the largest index seen.
        let num_nodes = left_nodes
            .iter()
            .chain(&right_nodes)
            .map(|&index| index as usize + 1)
            .max()
            .unwrap_or(0);
        let mut right_mates = vec![0i32; num_nodes];
        let mut optimal_cost: i64 = 0;
        let mut error_message: *mut c_char = core::ptr::null_mut();
        // SAFETY: the arc arrays share one length; right_mates covers every
        // node index by construction; output locations are writable.
        let status = unsafe {
            oxidor_sys::OxidorAssignmentSolve(
                left_nodes.as_ptr(),
                right_nodes.as_ptr(),
                self.costs.as_ptr(),
                left_nodes.len() as i32,
                &mut optimal_cost,
                right_mates.as_mut_ptr(),
                &mut error_message,
            )
        };
        let status = match status {
            0 => AssignmentStatus::Optimal,
            1 => AssignmentStatus::Infeasible,
            2 => AssignmentStatus::PossibleOverflow,
            // SAFETY: -1 means the shim caught an exception and set the message.
            -1 => {
                return Err(AlgorithmError::Native(unsafe {
                    take_error_message(error_message)
                }));
            }
            other => {
                return Err(AlgorithmError::Native(format!(
                    "SimpleLinearSumAssignment failed (status {other})",
                )));
            }
        };
        Ok(AssignmentResponse {
            status,
            optimal_cost,
            right_mates,
        })
    }
}

/// How a [`LinearSumAssignment`] solve ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum AssignmentStatus {
    /// A minimum-cost perfect matching was found.
    Optimal,
    /// The arcs admit no perfect matching.
    Infeasible,
    /// A cost magnitude is too large for the algorithm's arithmetic.
    PossibleOverflow,
}

/// The outcome of a [`LinearSumAssignment`] solve: a status and — when
/// optimal — an [`AssignmentSolution`].
#[derive(Debug, Clone)]
pub struct AssignmentResponse {
    status: AssignmentStatus,
    optimal_cost: i64,
    right_mates: Vec<i32>,
}

impl AssignmentResponse {
    /// How the solve ended.
    pub fn status(&self) -> AssignmentStatus {
        self.status
    }

    /// The minimum-cost matching, when one was found.
    pub fn solution(&self) -> Option<AssignmentSolution<'_>> {
        (self.status == AssignmentStatus::Optimal).then_some(AssignmentSolution {
            optimal_cost: self.optimal_cost,
            right_mates: &self.right_mates,
        })
    }
}

/// A minimum-cost perfect matching, borrowed from an [`AssignmentResponse`].
#[derive(Debug, Clone, Copy)]
pub struct AssignmentSolution<'response> {
    optimal_cost: i64,
    right_mates: &'response [i32],
}

impl AssignmentSolution<'_> {
    /// The total cost of the matching.
    pub fn total_cost(&self) -> i64 {
        self.optimal_cost
    }

    /// The right node matched to `left_node`.
    ///
    /// # Panics
    ///
    /// Panics if `left_node` is out of range — a programmer error.
    pub fn right_mate(&self, left_node: u32) -> u32 {
        self.right_mates[left_node as usize] as u32
    }

    /// Every `(left, right)` pair of the matching, in left-node order.
    pub fn assignments(&self) -> impl Iterator<Item = (u32, u32)> + '_ {
        self.right_mates
            .iter()
            .enumerate()
            .map(|(left, &right)| (left as u32, right as u32))
    }
}
