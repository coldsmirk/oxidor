use core::ffi::{CStr, c_char};

use oxidor_protos::operations_research::RoutingSearchParameters;
use oxidor_protos::operations_research::routing_search_status::Value as StatusProto;
use oxidor_protos::prost::Message;

use crate::problem::{RoutingError, RoutingProblem};

impl RoutingProblem {
    /// Solves with default search parameters.
    pub fn solve(&self) -> Result<RoutingSolution, RoutingError> {
        self.solve_with_parameters(&RoutingSearchParameters::default())
    }

    /// Solves with the given `RoutingSearchParameters` (time limits, first
    /// solution strategy, metaheuristic, …), merged over OR-Tools' defaults.
    ///
    /// ```no_run
    /// use oxidor_routing::{RoutingProblem, RoutingSearchParameters};
    /// use oxidor_routing::protos::prost_types::Duration;
    ///
    /// # let problem = RoutingProblem::from_matrix(vec![vec![0]])?;
    /// let parameters = RoutingSearchParameters {
    ///     time_limit: Some(Duration { seconds: 10, nanos: 0 }),
    ///     ..Default::default()
    /// };
    /// let solution = problem.solve_with_parameters(&parameters)?;
    /// # Ok::<(), oxidor_routing::RoutingError>(())
    /// ```
    pub fn solve_with_parameters(
        &self,
        parameters: &RoutingSearchParameters,
    ) -> Result<RoutingSolution, RoutingError> {
        self.validate()?;
        let parameter_bytes = parameters.encode_to_vec();

        let mut out_len: i32 = 0;
        let mut error_message: *mut c_char = core::ptr::null_mut();
        // SAFETY: validate() guarantees the array lengths the shim expects;
        // demands/capacities are either both present or both null; output
        // locations are valid for writes.
        let buffer_pointer = unsafe {
            oxidor_sys::OxidorRoutingSolveMatrix(
                self.num_nodes as i32,
                self.num_vehicles as i32,
                self.depot as i32,
                self.matrix.as_ptr(),
                self.demands
                    .as_ref()
                    .map_or(core::ptr::null(), |demands| demands.as_ptr()),
                self.vehicle_capacities
                    .as_ref()
                    .map_or(core::ptr::null(), |capacities| capacities.as_ptr()),
                parameter_bytes.as_ptr().cast(),
                parameter_bytes.len() as i32,
                &mut out_len,
                &mut error_message,
            )
        };

        if buffer_pointer.is_null() {
            // SAFETY: on failure the shim hands us a malloc'd message (or
            // null); copy it out and release it with the C allocator.
            let message = unsafe {
                let message = if error_message.is_null() {
                    "no error message".to_string()
                } else {
                    CStr::from_ptr(error_message).to_string_lossy().into_owned()
                };
                libc::free(error_message.cast());
                message
            };
            return Err(RoutingError::Native(message));
        }

        // SAFETY: on success the shim hands us a malloc'd buffer of exactly
        // out_len i64 entries; copy it out and release it.
        let buffer =
            unsafe { core::slice::from_raw_parts(buffer_pointer, out_len as usize) }.to_vec();
        unsafe { libc::free(buffer_pointer.cast()) };

        Ok(parse_solution_buffer(&buffer))
    }
}

/// Decodes the shim's flat buffer:
/// `[status, objective, num_routes, route_len, nodes…, route_len, …]`.
fn parse_solution_buffer(buffer: &[i64]) -> RoutingSolution {
    let status = match StatusProto::try_from(buffer[0] as i32) {
        Ok(StatusProto::RoutingNotSolved) => RoutingStatus::NotSolved,
        Ok(StatusProto::RoutingSuccess) => RoutingStatus::Success,
        Ok(StatusProto::RoutingPartialSuccessLocalOptimumNotReached) => {
            RoutingStatus::LocalOptimumNotReached
        }
        Ok(StatusProto::RoutingFail) => RoutingStatus::Fail,
        Ok(StatusProto::RoutingFailTimeout) => RoutingStatus::FailTimeout,
        Ok(StatusProto::RoutingInvalid) => RoutingStatus::Invalid,
        Ok(StatusProto::RoutingInfeasible) => RoutingStatus::Infeasible,
        Ok(StatusProto::RoutingOptimal) => RoutingStatus::Optimal,
        Err(_) => RoutingStatus::NotSolved,
    };
    let objective = buffer[1];
    let num_routes = buffer[2] as usize;
    let mut routes = Vec::with_capacity(num_routes);
    let mut cursor = 3;
    for _ in 0..num_routes {
        let length = buffer[cursor] as usize;
        cursor += 1;
        routes.push(
            buffer[cursor..cursor + length]
                .iter()
                .map(|&node| node as usize)
                .collect(),
        );
        cursor += length;
    }
    RoutingSolution {
        status,
        objective,
        routes,
    }
}

/// The outcome of a routing solve.
#[derive(Debug, Clone)]
pub struct RoutingSolution {
    status: RoutingStatus,
    objective: i64,
    routes: Vec<Vec<usize>>,
}

impl RoutingSolution {
    /// How the search ended.
    pub fn status(&self) -> RoutingStatus {
        self.status
    }

    /// Whether a set of routes was found.
    pub fn has_solution(&self) -> bool {
        !self.routes.is_empty()
    }

    /// The total cost of the routes (0 when no solution was found).
    pub fn objective(&self) -> i64 {
        self.objective
    }

    /// One route per vehicle: the nodes visited in order, excluding the
    /// depot at both ends (upstream `AssignmentToRoutes` semantics). Unused
    /// vehicles yield empty routes; empty when no solution was found.
    pub fn routes(&self) -> &[Vec<usize>] {
        &self.routes
    }
}

/// How a routing search ended. Unfavorable statuses are outcomes, not errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum RoutingStatus {
    /// The problem was not solved.
    NotSolved,
    /// A solution was found (local search completed within its limits).
    Success,
    /// A solution was found, but local search stopped before its optimum.
    LocalOptimumNotReached,
    /// No solution was found.
    Fail,
    /// A limit hit before any solution was found.
    FailTimeout,
    /// The model is invalid.
    Invalid,
    /// The problem was proven infeasible.
    Infeasible,
    /// The solution is provably optimal.
    Optimal,
}
