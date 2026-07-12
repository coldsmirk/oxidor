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

        fn slice_or_null(slice: Option<&[i64]>) -> *const i64 {
            slice.map_or(core::ptr::null(), <[i64]>::as_ptr)
        }

        // Materialize sparse time windows into the per-node arrays the shim
        // applies wholesale; unset nodes stay free within the horizon. A
        // later window overrides an earlier one on the same node.
        let window_arrays = self.time.as_ref().and_then(|time| {
            if time.windows.is_empty() {
                return None;
            }
            let mut starts = vec![0i64; self.num_nodes];
            let mut ends = vec![time.horizon; self.num_nodes];
            for &(node, start, end) in &time.windows {
                starts[node] = start;
                ends[node] = end;
            }
            Some((starts, ends))
        });
        let (pickups, deliveries): (Vec<i32>, Vec<i32>) = self
            .pickup_deliveries
            .iter()
            .map(|&(pickup, delivery)| (pickup as i32, delivery as i32))
            .unzip();

        let problem = oxidor_sys::OxidorRoutingProblem {
            num_nodes: self.num_nodes as i32,
            num_vehicles: self.num_vehicles as i32,
            depot: self.depot as i32,
            cost_matrix: self.matrix.as_ptr(),
            demands: slice_or_null(self.demands.as_deref()),
            vehicle_capacities: slice_or_null(self.vehicle_capacities.as_deref()),
            vehicle_fixed_costs: slice_or_null(self.vehicle_fixed_costs.as_deref()),
            travel_times: slice_or_null(self.time.as_ref().map(|time| &*time.travel_times)),
            service_times: slice_or_null(
                self.time
                    .as_ref()
                    .and_then(|time| time.service_times.as_deref()),
            ),
            time_window_starts: slice_or_null(window_arrays.as_ref().map(|(starts, _)| &**starts)),
            time_window_ends: slice_or_null(window_arrays.as_ref().map(|(_, ends)| &**ends)),
            time_horizon: self.time.as_ref().map_or(0, |time| time.horizon),
            max_waiting_time: self.time.as_ref().map_or(0, |time| time.max_waiting_time),
            pickups: if pickups.is_empty() {
                core::ptr::null()
            } else {
                pickups.as_ptr()
            },
            deliveries: if deliveries.is_empty() {
                core::ptr::null()
            } else {
                deliveries.as_ptr()
            },
            num_pickup_pairs: pickups.len() as i32,
            params_bytes: parameter_bytes.as_ptr().cast(),
            params_len: parameter_length,
        };

        let mut out_len: i32 = 0;
        let mut error_message: *mut c_char = core::ptr::null_mut();
        // SAFETY: validate() guarantees every length contract the struct
        // documents; all backing buffers (matrix, dimension arrays,
        // window_arrays, pickups/deliveries, parameter_bytes) are locals or
        // fields alive across the call; output locations are valid for
        // writes.
        let buffer_pointer = unsafe {
            oxidor_sys::OxidorRoutingSolveProblem(&problem, &mut out_len, &mut error_message)
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

/// Decodes the shim's flat buffer: `[status, objective, num_routes,
/// has_times,` then per route `route_len, nodes…, arrival_times…]`
/// (`route_len` arrival entries, only when `has_times` is 1).
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
    let has_times = buffer[3] != 0;
    let mut routes = Vec::with_capacity(num_routes);
    let mut arrival_times = has_times.then(|| Vec::with_capacity(num_routes));
    let mut cursor = 4;
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
        if let Some(arrival_times) = &mut arrival_times {
            arrival_times.push(buffer[cursor..cursor + length].to_vec());
            cursor += length;
        }
    }
    RoutingResponse {
        status,
        objective,
        routes,
        arrival_times,
    }
}

/// The outcome of a routing solve: a status and — when the search found
/// routes — a [`RoutingSolution`].
#[derive(Debug, Clone)]
pub struct RoutingResponse {
    status: RoutingStatus,
    objective: i64,
    routes: Vec<Vec<usize>>,
    arrival_times: Option<Vec<Vec<i64>>>,
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
            arrival_times: self.arrival_times.as_deref(),
        })
    }
}

/// A set of routes found by a solve, borrowed from a [`RoutingResponse`].
#[derive(Debug, Clone, Copy)]
pub struct RoutingSolution<'response> {
    objective: i64,
    routes: &'response [Vec<usize>],
    arrival_times: Option<&'response [Vec<i64>]>,
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

    /// The earliest feasible arrival time at each visited node, aligned with
    /// [`routes`](Self::routes) — present when the problem has a
    /// [`TimeDimension`](crate::TimeDimension).
    pub fn arrival_times(&self) -> Option<&[Vec<i64>]> {
        self.arrival_times
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
