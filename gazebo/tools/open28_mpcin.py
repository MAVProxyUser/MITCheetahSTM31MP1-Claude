#!/usr/bin/env python3
"""Read the schedule lead at the solver's input from [MPCIN2] lines.

`$STM32MP1_MPC_IN=2` makes ConvexMPCLocomotion print, at EVERY inline solve:

  [MPCIN2] seg=<k> table0=<abcd> table1=<abcd> c=<c0> <c1> <c2> <c3>

where seg is the gait's own segment index at that tick, table0/table1 are the
first two horizon steps of the contact table handed to the QP (leg order
FR FL RR RL, 1 = stance), and c is the gait's contact state at the same
tick. For OffsetDurationGait trotting (offsets 0,5,5,0; durations 5) the
table for segment j is 1001 for j in 0..4 and 0110 for j in 5..9, so the
segment the QP is being told to solve FOR is recoverable from table0 alone,
and the lead is (that segment - seg) mod 10. This prints the distribution of
that lead over the run, and the same for table1 (which must be lead + 1).

Usage: open28_mpcin.py ctrl_0.log [more logs...]
"""
import sys, re, collections

PAT = re.compile(r"\[MPCIN2\] seg=(\d+) table0=([01]{4}) table1=([01]{4}) c=([\d.]+) ([\d.]+) ([\d.]+) ([\d.]+)")
# trot: pair A = {FR, RL} = legs 0,3 stance in segments 0..4; pair B = {FL, RR} in 5..9
SEG_OF = {"1001": range(0, 5), "0110": range(5, 10)}


def lead_of(seg, table):
    segs = SEG_OF.get(table)
    if segs is None:
        return None
    # the smallest non-negative lead that lands in this table's segment set
    for lead in range(0, 10):
        if (seg + lead) % 10 in segs:
            return lead
    return None


for path in sys.argv[1:]:
    leads0, leads1, n, unknown, cmatch = collections.Counter(), collections.Counter(), 0, 0, collections.Counter()
    for line in open(path, errors="replace"):
        m = PAT.search(line)
        if not m:
            continue
        n += 1
        seg = int(m.group(1)); t0, t1 = m.group(2), m.group(3)
        c = [float(m.group(i)) for i in range(4, 8)]
        l0, l1 = lead_of(seg, t0), lead_of(seg, t1)
        if l0 is None or l1 is None:
            unknown += 1
            continue
        leads0[l0] += 1; leads1[l1] += 1
        # does the gait's own contact state at this tick agree with table0?
        c_table = "".join("1" if v > 0 else "0" for v in c)
        cmatch["c==table0" if c_table == t0 else ("c==seg" if lead_of(seg, c_table) == 0 else "other")] += 1
    print(f"{path}: {n} solves, {unknown} with a non-trot table")
    if n:
        print("  lead of table0 over seg (smallest non-negative that fits):", dict(sorted(leads0.items())))
        print("  lead of table1 over seg:                                 ", dict(sorted(leads1.items())))
        print("  gait contact state at the solve tick vs table0:          ", dict(cmatch))
        print("  NOTE the lead is ambiguous by multiples of 5 within a pair's stance; table0 and table1 together, and\n"
              "  the c state (which should read the segment BEFORE seg at the boundary tick, see run()'s ordering), pin it.")
