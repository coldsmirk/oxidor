//! Staff rostering, the classic CP-SAT introduction:
//!
//! - every shift of every day is covered by exactly one nurse,
//! - a nurse works at most one shift per day,
//! - the workload is spread evenly across nurses.
//!
//! Run with `cargo run -p oxidor-cpsat --example nurse_scheduling`.

use oxidor_cpsat::{BoolVar, CpModelBuilder, LinearExpr, SolveStatus};

const NUM_NURSES: usize = 4;
const NUM_DAYS: usize = 7;
const NUM_SHIFTS: usize = 3;
const SHIFT_LABELS: [&str; NUM_SHIFTS] = ["morning", "evening", "night"];

fn main() {
    let mut model = CpModelBuilder::new();

    // assigned[day][shift][nurse] == true ⇔ that nurse works that shift.
    let assigned: Vec<Vec<Vec<BoolVar>>> = (0..NUM_DAYS)
        .map(|day| {
            (0..NUM_SHIFTS)
                .map(|shift| {
                    (0..NUM_NURSES)
                        .map(|nurse| {
                            model.new_bool_var_named(format!(
                                "d{day}_{}_n{nurse}",
                                SHIFT_LABELS[shift]
                            ))
                        })
                        .collect()
                })
                .collect()
        })
        .collect();

    // Every shift of every day is covered by exactly one nurse.
    for day_roster in &assigned {
        for candidates in day_roster {
            model.add_exactly_one(candidates.iter().copied());
        }
    }

    // At most one shift per nurse per day.
    for day_roster in &assigned {
        for nurse in 0..NUM_NURSES {
            model.add_at_most_one(day_roster.iter().map(|candidates| candidates[nurse]));
        }
    }

    // Spread the workload evenly: with 21 shifts over 4 nurses, everyone
    // works 5 or 6 shifts.
    let total_shifts = NUM_DAYS * NUM_SHIFTS;
    let min_per_nurse = (total_shifts / NUM_NURSES) as i64;
    let max_per_nurse = min_per_nurse + i64::from(total_shifts % NUM_NURSES != 0);
    let workload_of = |nurse: usize| -> LinearExpr {
        assigned
            .iter()
            .flatten()
            .map(|candidates| candidates[nurse])
            .sum()
    };
    for nurse in 0..NUM_NURSES {
        model.add_linear_constraint(workload_of(nurse), min_per_nurse..=max_per_nurse);
    }

    let response = model.solve();
    assert_eq!(
        response.status(),
        SolveStatus::Optimal,
        "the roster is satisfiable"
    );
    let solution = response.solution().expect("optimal implies a solution");

    println!("Roster found in {:.2}s:\n", response.wall_time());
    println!(
        "day        {}",
        SHIFT_LABELS.map(|label| format!("{label:>8}")).join(" ")
    );
    for (day, day_roster) in assigned.iter().enumerate() {
        let row: Vec<String> = day_roster
            .iter()
            .map(|candidates| {
                let nurse = (0..NUM_NURSES)
                    .find(|&nurse| solution.bool_value(candidates[nurse]))
                    .expect("every shift is covered");
                format!("{:>8}", format!("nurse {nurse}"))
            })
            .collect();
        println!("day {day}      {}", row.join(" "));
    }

    println!();
    for nurse in 0..NUM_NURSES {
        println!(
            "nurse {nurse} works {} shifts",
            solution.value(workload_of(nurse))
        );
    }
}
