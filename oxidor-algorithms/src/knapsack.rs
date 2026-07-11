use core::ffi::c_char;

use crate::error::{AlgorithmError, take_error_message};

/// The best packing found for a knapsack instance.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct KnapsackSolution {
    total_value: i64,
    selected: Vec<bool>,
}

impl KnapsackSolution {
    /// The value of the packed items.
    pub fn total_value(&self) -> i64 {
        self.total_value
    }

    /// Whether item `index` is packed.
    pub fn is_selected(&self, index: usize) -> bool {
        self.selected[index]
    }

    /// The indices of the packed items.
    pub fn selected_items(&self) -> Vec<usize> {
        self.selected
            .iter()
            .enumerate()
            .filter_map(|(index, &selected)| selected.then_some(index))
            .collect()
    }
}

/// Solves a 0-1 knapsack: pick the subset of items maximizing total value
/// with total weight within `capacity`.
///
/// ```no_run
/// use oxidor_algorithms::solve_knapsack;
///
/// let solution = solve_knapsack(&[60, 100, 120], &[10, 20, 30], 50)?;
/// assert_eq!(solution.total_value(), 220);
/// assert_eq!(solution.selected_items(), vec![1, 2]);
/// # Ok::<(), oxidor_algorithms::AlgorithmError>(())
/// ```
pub fn solve_knapsack(
    values: &[i64],
    weights: &[i64],
    capacity: i64,
) -> Result<KnapsackSolution, AlgorithmError> {
    solve_knapsack_multidimensional(values, &[weights.to_vec()], &[capacity])
}

/// Like [`solve_knapsack`], with several weight dimensions that must all stay
/// within their capacities (`weights_per_dimension[d][item]`).
pub fn solve_knapsack_multidimensional(
    values: &[i64],
    weights_per_dimension: &[Vec<i64>],
    capacities: &[i64],
) -> Result<KnapsackSolution, AlgorithmError> {
    let num_items = values.len();
    if weights_per_dimension.len() != capacities.len() {
        return Err(AlgorithmError::new(format!(
            "{} weight dimensions but {} capacities",
            weights_per_dimension.len(),
            capacities.len(),
        )));
    }
    if weights_per_dimension.is_empty() {
        return Err(AlgorithmError::new(
            "at least one weight dimension is required",
        ));
    }
    let mut weights = Vec::with_capacity(weights_per_dimension.len() * num_items);
    for (dimension, row) in weights_per_dimension.iter().enumerate() {
        if row.len() != num_items {
            return Err(AlgorithmError::new(format!(
                "weight dimension {dimension} has {} entries for {num_items} items",
                row.len(),
            )));
        }
        weights.extend_from_slice(row);
    }

    let mut total_value: i64 = 0;
    let mut selected = vec![0u8; num_items];
    let mut error_message: *mut c_char = core::ptr::null_mut();
    // SAFETY: array lengths match what we pass; output buffers are writable
    // for the stated sizes.
    let code = unsafe {
        oxidor_sys::OxidorKnapsackSolve(
            values.as_ptr(),
            num_items as i32,
            weights.as_ptr(),
            capacities.as_ptr(),
            capacities.len() as i32,
            &mut total_value,
            selected.as_mut_ptr(),
            &mut error_message,
        )
    };
    if code != 0 {
        // SAFETY: nonzero return means the shim set the message (or null).
        return Err(AlgorithmError::new(unsafe {
            take_error_message(error_message)
        }));
    }
    Ok(KnapsackSolution {
        total_value,
        selected: selected.into_iter().map(|flag| flag != 0).collect(),
    })
}
