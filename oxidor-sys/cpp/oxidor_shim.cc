// Oxidor's own C shim over OR-Tools C++ APIs that ship no upstream C API
// (routing, the algorithms, CP-SAT solution observers; more as the bindings
// grow).
//
// Contract, mirrored by the declarations in src/lib.rs:
//   * plain C ABI; only POD scalars, arrays, and serialized protos cross it;
//   * every entry point catches C++ exceptions — they must never unwind into
//     Rust (that aborts the process);
//   * error strings and result buffers are malloc-allocated and owned by the
//     caller, who releases them with the C allocator's free().

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <string>
#include <vector>

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"
#include "ortools/constraint_solver/routing_parameters.pb.h"

namespace {

char* DuplicateMessage(const char* message) {
  const std::size_t length = std::strlen(message) + 1;
  char* copy = static_cast<char*>(std::malloc(length));
  if (copy != nullptr) std::memcpy(copy, message, length);
  return copy;
}

int64_t* CopyToMallocBuffer(const std::vector<int64_t>& buffer,
                            int32_t* out_len, char** error_message) {
  int64_t* result =
      static_cast<int64_t*>(std::malloc(buffer.size() * sizeof(int64_t)));
  if (result == nullptr) {
    *error_message = DuplicateMessage("out of memory");
    return nullptr;
  }
  std::memcpy(result, buffer.data(), buffer.size() * sizeof(int64_t));
  *out_len = static_cast<int32_t>(buffer.size());
  return result;
}

}  // namespace

extern "C" {

// Solves a vehicle routing problem over a dense arc-cost matrix.
//
// Inputs: `matrix` is row-major with num_nodes^2 entries. `demands` (length
// num_nodes) and `vehicle_capacities` (length num_vehicles) are either both
// non-null (adding a capacity dimension) or both null. `params_bytes` is a
// serialized RoutingSearchParameters merged over the defaults, or null.
//
// On success returns a malloc'd int64 buffer of `*out_len` entries laid out
// as [status, objective, num_routes, route_len, nodes..., route_len, ...]
// where routes list visited nodes excluding the depot endpoints. On failure
// returns null and sets `*error_message` (malloc'd, caller frees).
int64_t* OxidorRoutingSolveMatrix(int32_t num_nodes, int32_t num_vehicles,
                                  int32_t depot, const int64_t* matrix,
                                  const int64_t* demands,
                                  const int64_t* vehicle_capacities,
                                  const void* params_bytes,
                                  int32_t params_len, int32_t* out_len,
                                  char** error_message) {
  *error_message = nullptr;
  *out_len = 0;
  try {
    operations_research::RoutingIndexManager manager(
        num_nodes, num_vehicles,
        operations_research::RoutingIndexManager::NodeIndex(depot));
    operations_research::RoutingModel model(manager);

    const int transit_index = model.RegisterTransitCallback(
        [&manager, matrix, num_nodes](int64_t from_index,
                                      int64_t to_index) -> int64_t {
          const int from = manager.IndexToNode(from_index).value();
          const int to = manager.IndexToNode(to_index).value();
          return matrix[static_cast<int64_t>(from) * num_nodes + to];
        });
    model.SetArcCostEvaluatorOfAllVehicles(transit_index);

    if (demands != nullptr && vehicle_capacities != nullptr) {
      const int demand_index = model.RegisterUnaryTransitCallback(
          [&manager, demands](int64_t index) -> int64_t {
            return demands[manager.IndexToNode(index).value()];
          });
      const std::vector<int64_t> capacities(
          vehicle_capacities, vehicle_capacities + num_vehicles);
      model.AddDimensionWithVehicleCapacity(demand_index, /*slack_max=*/0,
                                            capacities,
                                            /*fix_start_cumul_to_zero=*/true,
                                            "Capacity");
    }

    operations_research::RoutingSearchParameters parameters =
        operations_research::DefaultRoutingSearchParameters();
    if (params_bytes != nullptr && params_len > 0) {
      operations_research::RoutingSearchParameters overrides;
      if (!overrides.ParseFromArray(params_bytes, params_len)) {
        *error_message =
            DuplicateMessage("invalid RoutingSearchParameters bytes");
        return nullptr;
      }
      parameters.MergeFrom(overrides);
    }

    const operations_research::Assignment* solution =
        model.SolveWithParameters(parameters);

    std::vector<int64_t> buffer;
    buffer.push_back(static_cast<int64_t>(model.status()));
    if (solution == nullptr) {
      buffer.push_back(0);  // objective
      buffer.push_back(0);  // num_routes
    } else {
      buffer.push_back(solution->ObjectiveValue());
      std::vector<std::vector<int64_t>> routes;
      model.AssignmentToRoutes(*solution, &routes);
      buffer.push_back(static_cast<int64_t>(routes.size()));
      for (const std::vector<int64_t>& route : routes) {
        buffer.push_back(static_cast<int64_t>(route.size()));
        buffer.insert(buffer.end(), route.begin(), route.end());
      }
    }
    return CopyToMallocBuffer(buffer, out_len, error_message);
  } catch (const std::exception& exception) {
    *error_message = DuplicateMessage(exception.what());
    return nullptr;
  } catch (...) {
    *error_message = DuplicateMessage("unknown C++ exception");
    return nullptr;
  }
}

}  // extern "C"

#include "ortools/algorithms/knapsack_solver.h"
#include "ortools/graph/max_flow.h"
#include "ortools/graph/min_cost_flow.h"

extern "C" {

// Solves a (multi-dimensional) 0-1 knapsack with branch and bound.
//
// `weights` is row-major num_dims x num_items; `capacities` has num_dims
// entries. Writes the best value to `*out_best_value` and 0/1 into
// `out_selected` (length num_items). Returns 0 on success; on failure
// returns nonzero and sets `*error_message` (malloc'd, caller frees).
int32_t OxidorKnapsackSolve(const int64_t* profits, int32_t num_items,
                            const int64_t* weights,
                            const int64_t* capacities, int32_t num_dims,
                            int64_t* out_best_value, uint8_t* out_selected,
                            char** error_message) {
  *error_message = nullptr;
  try {
    const std::vector<int64_t> profit_vector(profits, profits + num_items);
    std::vector<std::vector<int64_t>> weight_matrix;
    weight_matrix.reserve(num_dims);
    for (int32_t dim = 0; dim < num_dims; ++dim) {
      const int64_t* row = weights + static_cast<int64_t>(dim) * num_items;
      weight_matrix.emplace_back(row, row + num_items);
    }
    const std::vector<int64_t> capacity_vector(capacities,
                                               capacities + num_dims);

    operations_research::KnapsackSolver solver(
        operations_research::KnapsackSolver::
            KNAPSACK_MULTIDIMENSION_BRANCH_AND_BOUND_SOLVER,
        "oxidor_knapsack");
    solver.Init(profit_vector, weight_matrix, capacity_vector);
    *out_best_value = solver.Solve();
    for (int32_t item = 0; item < num_items; ++item) {
      out_selected[item] = solver.BestSolutionContains(item) ? 1 : 0;
    }
    return 0;
  } catch (const std::exception& exception) {
    *error_message = DuplicateMessage(exception.what());
    return 1;
  } catch (...) {
    *error_message = DuplicateMessage("unknown C++ exception");
    return 1;
  }
}

// Computes a maximum flow. Arc arrays have num_arcs entries; per-arc flows
// are written to `out_flows` and the optimal flow to `*out_max_flow`.
// Returns the SimpleMaxFlow status (0 = OPTIMAL), or -1 on a C++ exception
// (with `*error_message` set, malloc'd).
int32_t OxidorMaxFlowSolve(const int32_t* tails, const int32_t* heads,
                           const int64_t* capacities, int32_t num_arcs,
                           int32_t source, int32_t sink, int64_t* out_flows,
                           int64_t* out_max_flow, char** error_message) {
  *error_message = nullptr;
  try {
    operations_research::SimpleMaxFlow max_flow;
    for (int32_t arc = 0; arc < num_arcs; ++arc) {
      max_flow.AddArcWithCapacity(tails[arc], heads[arc], capacities[arc]);
    }
    const auto status = max_flow.Solve(source, sink);
    if (status == operations_research::SimpleMaxFlow::OPTIMAL) {
      *out_max_flow = max_flow.OptimalFlow();
      for (int32_t arc = 0; arc < num_arcs; ++arc) {
        out_flows[arc] = max_flow.Flow(arc);
      }
    }
    return static_cast<int32_t>(status);
  } catch (const std::exception& exception) {
    *error_message = DuplicateMessage(exception.what());
    return -1;
  } catch (...) {
    *error_message = DuplicateMessage("unknown C++ exception");
    return -1;
  }
}

// Computes a minimum-cost flow. Arc arrays have num_arcs entries; `supplies`
// has num_nodes entries (positive supply, negative demand). Per-arc flows are
// written to `out_flows` and the total cost to `*out_total_cost` when
// optimal. Returns the SimpleMinCostFlow status (1 = OPTIMAL), or -1 on a
// C++ exception (with `*error_message` set, malloc'd).
int32_t OxidorMinCostFlowSolve(const int32_t* tails, const int32_t* heads,
                               const int64_t* capacities,
                               const int64_t* unit_costs, int32_t num_arcs,
                               const int64_t* supplies, int32_t num_nodes,
                               int64_t* out_flows, int64_t* out_total_cost,
                               char** error_message) {
  *error_message = nullptr;
  try {
    operations_research::SimpleMinCostFlow min_cost_flow;
    for (int32_t arc = 0; arc < num_arcs; ++arc) {
      min_cost_flow.AddArcWithCapacityAndUnitCost(
          tails[arc], heads[arc], capacities[arc], unit_costs[arc]);
    }
    for (int32_t node = 0; node < num_nodes; ++node) {
      min_cost_flow.SetNodeSupply(node, supplies[node]);
    }
    const auto status = min_cost_flow.Solve();
    if (status == operations_research::SimpleMinCostFlow::OPTIMAL) {
      *out_total_cost = min_cost_flow.OptimalCost();
      for (int32_t arc = 0; arc < num_arcs; ++arc) {
        out_flows[arc] = min_cost_flow.Flow(arc);
      }
    }
    return static_cast<int32_t>(status);
  } catch (const std::exception& exception) {
    *error_message = DuplicateMessage(exception.what());
    return -1;
  } catch (...) {
    *error_message = DuplicateMessage("unknown C++ exception");
    return -1;
  }
}

}  // extern "C"

#include <atomic>

#include "ortools/sat/cp_model.pb.h"
#include "ortools/sat/cp_model_solver.h"
#include "ortools/sat/model.h"
#include "ortools/sat/sat_parameters.pb.h"
#include "ortools/util/time_limit.h"

extern "C" {

// Invoked on each feasible solution with a serialized CpSolverResponse; a
// nonzero return asks the search to stop. Must not throw or unwind.
typedef int32_t (*OxidorCpSolutionCallback)(const void* response_bytes,
                                            int32_t response_len,
                                            void* user_data);

// Solves a serialized CpModelProto like the official C API, additionally
// invoking `callback` on every feasible solution found during the search
// (every improving solution; every solution when the parameters set
// enumerate_all_solutions). This is the same observer registration the
// official C API's interruptible env uses internally, so stopping via the
// callback return value behaves exactly like SolveCpStopSearch.
//
// On success returns 0 and hands out a malloc'd serialized CpSolverResponse
// (`*out_response` / `*out_response_len`, caller frees). On failure returns
// nonzero and sets `*error_message` (malloc'd, caller frees).
int32_t OxidorCpSatSolveWithObserver(const void* model_bytes,
                                     int32_t model_len,
                                     const void* params_bytes,
                                     int32_t params_len,
                                     OxidorCpSolutionCallback callback,
                                     void* user_data, void** out_response,
                                     int32_t* out_response_len,
                                     char** error_message) {
  *error_message = nullptr;
  *out_response = nullptr;
  *out_response_len = 0;
  try {
    operations_research::sat::CpModelProto model_proto;
    if (!model_proto.ParseFromArray(model_bytes, model_len)) {
      *error_message = DuplicateMessage("invalid CpModelProto bytes");
      return 1;
    }
    operations_research::sat::SatParameters parameters;
    if (params_bytes != nullptr && params_len > 0 &&
        !parameters.ParseFromArray(params_bytes, params_len)) {
      *error_message = DuplicateMessage("invalid SatParameters bytes");
      return 1;
    }

    operations_research::sat::Model model;
    model.Add(operations_research::sat::NewSatParameters(parameters));
    std::atomic<bool> stopped(false);
    model.GetOrCreate<operations_research::TimeLimit>()
        ->RegisterExternalBooleanAsLimit(&stopped);
    model.Add(operations_research::sat::NewFeasibleSolutionObserver(
        [callback, user_data, &stopped](
            const operations_research::sat::CpSolverResponse& solution) {
          std::string bytes;
          solution.SerializeToString(&bytes);
          if (callback(bytes.data(), static_cast<int32_t>(bytes.size()),
                       user_data) != 0) {
            stopped.store(true);
          }
        }));

    const operations_research::sat::CpSolverResponse response =
        operations_research::sat::SolveCpModel(model_proto, &model);
    std::string bytes;
    response.SerializeToString(&bytes);
    void* buffer = std::malloc(bytes.size());
    if (buffer == nullptr && !bytes.empty()) {
      *error_message = DuplicateMessage("out of memory");
      return 1;
    }
    std::memcpy(buffer, bytes.data(), bytes.size());
    *out_response = buffer;
    *out_response_len = static_cast<int32_t>(bytes.size());
    return 0;
  } catch (const std::exception& exception) {
    *error_message = DuplicateMessage(exception.what());
    return 1;
  } catch (...) {
    *error_message = DuplicateMessage("unknown C++ exception");
    return 1;
  }
}

}  // extern "C"

#include <algorithm>

#include "absl/status/statusor.h"
#include "ortools/math_opt/core/solver.h"
#include "ortools/math_opt/core/solver_interface.h"
#include "ortools/math_opt/model.pb.h"
#include "ortools/math_opt/parameters.pb.h"
#include "ortools/math_opt/result.pb.h"
#include "ortools/util/solve_interrupter.h"

extern "C" {

// The MathOpt entry points below exist because the upstream C API
// (math_opt/core/c_api/solver.h) takes no per-solve parameters; they call the
// same core Solver::NonIncrementalSolve the C API wraps, adding a parsed
// SolveParametersProto and this shim's own interrupter.

// Returns a new, untriggered operations_research::SolveInterrupter, or null
// on allocation failure. Release with OxidorMathOptFreeInterrupter.
void* OxidorMathOptNewInterrupter() {
  return new (std::nothrow) operations_research::SolveInterrupter();
}

// Frees an interrupter; no effect on null. It must outlive every solve
// using it.
void OxidorMathOptFreeInterrupter(void* interrupter) {
  delete static_cast<operations_research::SolveInterrupter*>(interrupter);
}

// Triggers the interrupter. Thread-safe; sticky (never resets).
void OxidorMathOptInterrupt(void* interrupter) {
  static_cast<operations_research::SolveInterrupter*>(interrupter)
      ->Interrupt();
}

// Returns nonzero if triggered. Thread-safe.
int32_t OxidorMathOptIsInterrupted(const void* interrupter) {
  return static_cast<const operations_research::SolveInterrupter*>(interrupter)
                 ->IsInterrupted()
             ? 1
             : 0;
}

// Solves a serialized MathOpt ModelProto with the solver selected by
// `solver_type` (SolverTypeProto wire values) under a serialized
// SolveParametersProto (null/empty for defaults) and an optional interrupter
// from OxidorMathOptNewInterrupter.
//
// Returns 0 on success, with a malloc'd serialized SolveResultProto in
// `*out_result` / `*out_result_len` (caller frees). On failure returns the
// absl::StatusCode numeric value (or 3 = invalid argument for unparseable
// inputs, 13 = internal for caught C++ exceptions) and sets `*error_message`
// (malloc'd, caller frees).
int32_t OxidorMathOptSolveWithParameters(
    const void* model_bytes, size_t model_len, int32_t solver_type,
    const void* params_bytes, int32_t params_len, const void* interrupter,
    void** out_result, size_t* out_result_len, char** error_message) {
  *error_message = nullptr;
  *out_result = nullptr;
  *out_result_len = 0;
  try {
    operations_research::math_opt::ModelProto model;
    if (!model.ParseFromArray(model_bytes, static_cast<int>(model_len))) {
      *error_message = DuplicateMessage("invalid ModelProto bytes");
      return 3;  // absl::StatusCode::kInvalidArgument
    }
    operations_research::math_opt::Solver::SolveArgs solve_args;
    if (params_bytes != nullptr && params_len > 0 &&
        !solve_args.parameters.ParseFromArray(params_bytes, params_len)) {
      *error_message = DuplicateMessage("invalid SolveParametersProto bytes");
      return 3;
    }
    solve_args.interrupter =
        static_cast<const operations_research::SolveInterrupter*>(interrupter);

    const absl::StatusOr<operations_research::math_opt::SolveResultProto>
        result = operations_research::math_opt::Solver::NonIncrementalSolve(
            model,
            static_cast<operations_research::math_opt::SolverTypeProto>(
                solver_type),
            /*init_args=*/{}, solve_args);
    if (!result.ok()) {
      *error_message =
          DuplicateMessage(std::string(result.status().message()).c_str());
      return static_cast<int32_t>(result.status().code());
    }

    std::string bytes;
    result->SerializeToString(&bytes);
    void* buffer = std::malloc(bytes.size());
    if (buffer == nullptr && !bytes.empty()) {
      *error_message = DuplicateMessage("out of memory");
      return 8;  // absl::StatusCode::kResourceExhausted
    }
    std::memcpy(buffer, bytes.data(), bytes.size());
    *out_result = buffer;
    *out_result_len = bytes.size();
    return 0;
  } catch (const std::exception& exception) {
    *error_message = DuplicateMessage(exception.what());
    return 13;  // absl::StatusCode::kInternal
  } catch (...) {
    *error_message = DuplicateMessage("unknown C++ exception");
    return 13;
  }
}

// Lists the MathOpt solvers registered in the linked library (SolverTypeProto
// wire values). Returns a malloc'd int32 buffer of `*out_len` entries (caller
// frees), or null on failure with `*error_message` set (malloc'd).
int32_t* OxidorMathOptRegisteredSolvers(int32_t* out_len,
                                        char** error_message) {
  *error_message = nullptr;
  *out_len = 0;
  try {
    const std::vector<operations_research::math_opt::SolverTypeProto> solvers =
        operations_research::math_opt::AllSolversRegistry::Instance()
            ->RegisteredSolvers();
    int32_t* buffer = static_cast<int32_t*>(
        std::malloc(std::max<std::size_t>(1, solvers.size()) *
                    sizeof(int32_t)));
    if (buffer == nullptr) {
      *error_message = DuplicateMessage("out of memory");
      return nullptr;
    }
    for (std::size_t index = 0; index < solvers.size(); ++index) {
      buffer[index] = static_cast<int32_t>(solvers[index]);
    }
    *out_len = static_cast<int32_t>(solvers.size());
    return buffer;
  } catch (const std::exception& exception) {
    *error_message = DuplicateMessage(exception.what());
    return nullptr;
  } catch (...) {
    *error_message = DuplicateMessage("unknown C++ exception");
    return nullptr;
  }
}

}  // extern "C"
