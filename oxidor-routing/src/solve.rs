use core::ffi::{CStr, c_char};
use std::time::Duration;

use oxidor_protos::operations_research::RoutingSearchParameters;
use oxidor_protos::operations_research::routing_search_status::Value as StatusProto;
use oxidor_protos::prost::Message;

use crate::problem::{RoutingError, RoutingProblem};

impl RoutingProblem {
    /// Solves with default search parameters.
    pub fn solve(&self) -> Result<RoutingResponse, RoutingError> {
        self.solve_with_parameters(&RoutingSearchParameters::default())
    }

    /// Solves with the local search capped at `limit`, returning the best
    /// routes found by then.
    ///
    /// ```no_run
    /// use std::time::Duration;
    /// # let problem = oxidor_routing::RoutingProblem::from_matrix(vec![vec![0]])?;
    /// let response = problem.solve_with_time_limit(Duration::from_secs(10))?;
    /// # Ok::<(), oxidor_routing::RoutingError>(())
    /// ```
    ///
    /// For any other tuning knob, reach for
    /// [`solve_with_parameters`](Self::solve_with_parameters).
    pub fn solve_with_time_limit(&self, limit: Duration) -> Result<RoutingResponse, RoutingError> {
        self.solve_with_parameters(&RoutingSearchParameters {
            time_limit: Some(oxidor_protos::prost_types::Duration {
                seconds: limit.as_secs() as i64,
                nanos: limit.subsec_nanos() as i32,
            }),
            ..Default::default()
        })
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
    /// let response = problem.solve_with_parameters(&parameters)?;
    /// # Ok::<(), oxidor_routing::RoutingError>(())
    /// ```
    pub fn solve_with_parameters(
        &self,
        parameters: &RoutingSearchParameters,
    ) -> Result<RoutingResponse, RoutingError> {
        self.validate()?;
        let parameter_bytes = parameters.encode_to_vec();
        let parameter_length = i32::try_from(parameter_bytes.len()).map_err(|_| {
            RoutingError::InvalidProblem("serialized search parameters exceed 2 GiB".into())
        })?;

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
                parameter_length,
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

        Ok(parse_response_buffer(&buffer))
    }
}

/// Decodes the shim's flat buffer:
/// `[status, objective, num_routes, route_len, nodes…, route_len, …]`.
fn parse_response_buffer(buffer: &[i64]) -> RoutingResponse {
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
        // A status this crate predates; the safe reading is "nothing solved".
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
    RoutingResponse {
        status,
        objective,
        routes,
    }
}

/// The outcome of a routing solve: a status and — when the search found
/// routes — a [`RoutingSolution`].
#[derive(Debug, Clone)]
pub struct RoutingResponse {
    status: RoutingStatus,
    objective: i64,
    routes: Vec<Vec<usize>>,
}

impl RoutingResponse {
    /// How the search ended.
    pub fn status(&self) -> RoutingStatus {
        self.status
    }

    /// The routes, when the search found any.
    pub fn solution(&self) -> Option<RoutingSolution<'_>> {
        (!self.routes.is_empty()).then_some(RoutingSolution {
            objective: self.objective,
            routes: &self.routes,
        })
    }
}

/// A set of routes found by a solve, borrowed from a [`RoutingResponse`].
#[derive(Debug, Clone, Copy)]
pub struct RoutingSolution<'response> {
    objective: i64,
    routes: &'response [Vec<usize>],
}

impl RoutingSolution<'_> {
    /// The total cost of the routes.
    pub fn objective_value(&self) -> i64 {
        self.objective
    }

    /// One route per vehicle: the nodes visited in order, excluding the
    /// depot at both ends (upstream `AssignmentToRoutes` semantics). Unused
    /// vehicles yield empty routes.
    pub fn routes(&self) -> &[Vec<usize>] {
        self.routes
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
