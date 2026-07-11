use std::ops::RangeInclusive;

/// A set of integers represented as sorted, disjoint, closed intervals.
///
/// This mirrors the domain encoding of CP-SAT's `IntegerVariableProto`:
/// `[min_0, max_0, min_1, max_1, …]`. Construction normalizes the input —
/// intervals are sorted, and overlapping or adjacent intervals are merged.
///
/// ```
/// use oxidor_cpsat::Domain;
///
/// assert_eq!(Domain::from_intervals([(5, 9), (0, 4)]), Domain::new(0, 9));
/// assert_eq!(Domain::from_values([1, 2, 5]).flattened(), vec![1, 2, 5, 5]);
/// ```
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct Domain {
    intervals: Vec<(i64, i64)>,
}

impl Domain {
    /// The single interval `[min, max]`. Empty if `min > max`.
    pub fn new(min: i64, max: i64) -> Self {
        Self::from_intervals([(min, max)])
    }

    /// The union of the given closed intervals. Intervals with `lo > hi` are
    /// dropped; the rest are sorted and merged.
    pub fn from_intervals(intervals: impl IntoIterator<Item = (i64, i64)>) -> Self {
        let mut items: Vec<(i64, i64)> =
            intervals.into_iter().filter(|(lo, hi)| lo <= hi).collect();
        items.sort_unstable();
        let mut merged: Vec<(i64, i64)> = Vec::with_capacity(items.len());
        for (lo, hi) in items {
            match merged.last_mut() {
                Some(last) if lo <= last.1.saturating_add(1) => last.1 = last.1.max(hi),
                _ => merged.push((lo, hi)),
            }
        }
        Self { intervals: merged }
    }

    /// The set containing exactly the given values.
    pub fn from_values(values: impl IntoIterator<Item = i64>) -> Self {
        Self::from_intervals(values.into_iter().map(|v| (v, v)))
    }

    /// Whether the set contains no value (such a variable is unassignable).
    pub fn is_empty(&self) -> bool {
        self.intervals.is_empty()
    }

    /// The proto encoding: `[min_0, max_0, min_1, max_1, …]`.
    pub fn flattened(&self) -> Vec<i64> {
        self.intervals
            .iter()
            .flat_map(|&(lo, hi)| [lo, hi])
            .collect()
    }

    /// The encoding of this domain translated by `-shift` (used to fold an
    /// expression's constant into the domain). `i64::MIN` / `i64::MAX` act as
    /// CP-SAT's ±infinity sentinels and stay pinned, matching the C++ model
    /// builder; finite bounds saturate.
    pub(crate) fn flattened_shifted(&self, shift: i64) -> Vec<i64> {
        let translate = |bound: i64| match bound {
            i64::MIN | i64::MAX => bound,
            finite => finite.saturating_sub(shift),
        };
        self.intervals
            .iter()
            .flat_map(|&(lo, hi)| [translate(lo), translate(hi)])
            .collect()
    }
}

impl From<RangeInclusive<i64>> for Domain {
    fn from(range: RangeInclusive<i64>) -> Self {
        Self::new(*range.start(), *range.end())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn merges_overlapping_and_adjacent_intervals() {
        let domain = Domain::from_intervals([(4, 6), (0, 2), (3, 3), (10, 12)]);
        assert_eq!(domain.flattened(), vec![0, 6, 10, 12]);
    }

    #[test]
    fn drops_inverted_intervals() {
        assert!(Domain::from_intervals([(5, 1)]).is_empty());
    }

    #[test]
    fn from_values_collapses_runs() {
        assert_eq!(
            Domain::from_values([3, 1, 2, 7]).flattened(),
            vec![1, 3, 7, 7]
        );
    }

    #[test]
    fn shifting_saturates_at_bounds() {
        let domain = Domain::from_intervals([(i64::MIN, 0)]);
        assert_eq!(domain.flattened_shifted(5), vec![i64::MIN, -5]);
    }
}
