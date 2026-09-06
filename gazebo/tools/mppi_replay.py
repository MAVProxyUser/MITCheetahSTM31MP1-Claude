#!/usr/bin/env python3
"""Stage 1 of the MPPI question: can SAMPLING match this MPC's own solver, and
what does it cost on the Mac GPU?

The operator asked whether an M4's GPU could prove an MPPI controller works
with this codebase. Before writing a controller, the cheap decisive test is
whether sampling can solve the problem the existing controller actually
assembles - on real states, not on a re-derivation of them. Re-deriving the
SRBD in Python to test a solver would be testing the re-derivation.

So `SolverMPC.cpp` now captures its contact-reduced problem and its own answer
($MPC_DUMP), and this replays them:

    P (nv x nv), q (nv), A (nc x nv), 0 <= A x <= u, plus the QP's solution

Three things are measured against each captured solve:

  1. COST GAP. The QP's solution is the reference optimum. How close does a
     sampler get, as a function of sample count K?
  2. THE WARM-START CONTROL. Consecutive MPC problems differ by one gait
     segment, so the PREVIOUS solve's answer is already a good iterate. If
     just reusing it scores nearly as well as the QP, then any sampler that
     warm-starts is mostly being scored on the warm start, and the sampling is
     decoration. That control has to be reported next to every sampler number.
  3. WALL TIME, MPS vs CPU, including transfers, warmed up.

What this CANNOT tell you: whether MPPI-the-controller walks. Sampling this
particular problem is a strictly worse ADMM - it is convex, and the whole
point of MPPI is non-convex costs and dynamics it cannot express. A good
result here says "sampling is affordable at this scale"; a bad one says the
idea is dead before anyone writes a controller. Neither says the robot walks.

And nothing here ports to the board: the STM32MP1 has no compute GPU.

Usage: mppi_replay.py --dump .../trot19.bin [--records 40] [--iters 8]
"""
import argparse, struct, time, sys
import numpy as np

try:
    import torch
except ImportError:
    sys.exit("needs torch (2.x). Not a runtime dependency - this is an offline study.")

ap = argparse.ArgumentParser()
ap.add_argument("--dump", required=True)
ap.add_argument("--records", type=int, default=40, help="how many captured solves to replay")
ap.add_argument("--iters", type=int, default=8, help="MPPI refinement iterations per solve")
ap.add_argument("--sigma", type=float, default=2.0, help="sampling std, in solution units")
ap.add_argument("--temp", type=float, default=0.05, help="softmax temperature")
ap.add_argument("--lam", type=float, default=1e3, help="constraint penalty weight")
ap.add_argument("--ks", default="256,1024,4096,16384,65536")
ap.add_argument("--sigmas", default="", help="if set, sweep sigma at one K instead")
a = ap.parse_args()

MAGIC = 0x4D504331


def load(path):
    out = []
    with open(path, "rb") as f:
        while True:
            h = f.read(16)
            if len(h) < 16:
                break
            magic, nv, nc, H = struct.unpack("<4i", h)
            if magic != MAGIC:
                raise SystemExit(f"bad magic {magic:#x} - stale dump?")
            g = lambda n: np.frombuffer(f.read(4 * n), dtype="<f4").astype(np.float32)
            P = g(nv * nv).reshape(nv, nv)
            q = g(nv)
            A = g(nc * nv).reshape(nc, nv)
            u = g(nc)
            s = g(nv)
            if not (np.isfinite(P).all() and np.isfinite(q).all()
                    and np.isfinite(A).all() and np.isfinite(s).all()):
                continue                      # the QP does occasionally return non-finite
            if np.abs(s).max() > 1e4:
                continue                      # ...and occasionally finite garbage
            out.append(dict(nv=nv, nc=nc, H=H, P=P, q=q, A=A, u=u, sol=s))
    return out


def cost_np(x, r):
    """Objective plus the same penalty the sampler is scored under."""
    J = 0.5 * float(x @ r["P"] @ x) + float(r["q"] @ x)
    Ax = r["A"] @ x
    v = np.maximum(Ax - r["u"], 0) + np.maximum(-Ax, 0)
    return J, float(np.max(v)), J + a.lam * float((v ** 2).sum())


def feasible_fraction(r, K, dev, mu0, sigma):
    """Of K isotropic samples around mu0, what share satisfies 0 <= Ax <= u?

    This is the question that decides whether sampling can work here at all.
    The constraints are per-foot friction cones in nv=60..120 dimensions; if
    essentially no sample lands inside them, a penalty-scored sampler will
    keep choosing its own incumbent and never move, which is exactly what the
    first cold-start run did - identical numbers at every K, because mu never
    changed."""
    A = torch.as_tensor(r["A"], device=dev)
    ub = torch.as_tensor(r["u"], device=dev)
    mu = torch.as_tensor(mu0, device=dev)
    U = mu.unsqueeze(0) + sigma * torch.randn(K, r["nv"], device=dev)
    Ax = U @ A.T
    ok = ((Ax >= -1e-4) & (Ax <= ub + 1e-4)).all(1)
    return float(ok.float().mean())


def sample_solve(r, K, dev, mu0, iters):
    P = torch.as_tensor(r["P"], device=dev)
    q = torch.as_tensor(r["q"], device=dev)
    A = torch.as_tensor(r["A"], device=dev)
    ub = torch.as_tensor(r["u"], device=dev)
    mu = torch.as_tensor(mu0, device=dev)
    for _ in range(iters):
        U = mu.unsqueeze(0) + a.sigma * torch.randn(K, r["nv"], device=dev)
        U = torch.cat([mu.unsqueeze(0), U], 0)          # keep the incumbent
        S = 0.5 * ((U @ P) * U).sum(1) + U @ q
        Ax = U @ A.T
        v = (Ax - ub).clamp(min=0) + (-Ax).clamp(min=0)
        S = S + a.lam * (v ** 2).sum(1)
        w = torch.softmax(-(S - S.min()) / a.temp, 0)
        mu = (w.unsqueeze(1) * U).sum(0)
    return mu.detach().to("cpu").numpy()


recs = load(a.dump)
if not recs:
    raise SystemExit("no usable records")
print(f"\n  {len(recs)} captured solves   nv={sorted(set(r['nv'] for r in recs))}  "
      f"nc={sorted(set(r['nc'] for r in recs))}  horizon={recs[0]['H']}")

idx = np.linspace(0, len(recs) - 1, min(a.records, len(recs))).astype(int)
Ks = [int(k) for k in a.ks.split(",")]
devs = ["cpu"] + (["mps"] if torch.backends.mps.is_available() else [])

# ---- control 1: the QP's own answer, and what the PREVIOUS solve scores ----
qp_J, warm_J, warm_ok = [], [], 0
for i in idx:
    r = recs[i]
    _, _, Jq = cost_np(r["sol"], r)
    qp_J.append(Jq)
    prev = next((recs[j]["sol"] for j in range(i - 1, -1, -1)
                 if recs[j]["nv"] == r["nv"]), None)
    if prev is not None:
        _, _, Jw = cost_np(prev, r)
        warm_J.append(Jw - Jq)
        warm_ok += 1
print(f"\n  QP objective over the replayed solves: median {np.median(qp_J):+.4f}  "
      f"range [{min(qp_J):+.4f}, {max(qp_J):+.4f}]")
if warm_J:
    print(f"  CONTROL - previous solve reused unchanged, {warm_ok} of {len(idx)}:")
    print(f"      cost excess over the QP: median {np.median(warm_J):+.4f}   "
          f"p90 {np.percentile(warm_J,90):+.4f}")

# ---- can a sample ever land in the feasible set? ----
dev0 = devs[-1]
print(f"\n  feasible fraction of isotropic samples (K=4096), by sigma:")
print(f"      {'sigma':>7} {'cold (mu=0)':>13} {'warm (mu=prev)':>16}")
for sg in (0.05, 0.1, 0.25, 0.5, 1.0, 2.0):
    fc, fw = [], []
    for i in idx[:10]:
        r = recs[i]
        fc.append(feasible_fraction(r, 4096, dev0, np.zeros(r["nv"], np.float32), sg))
        p = next((recs[j]["sol"] for j in range(i - 1, -1, -1)
                  if recs[j]["nv"] == r["nv"]), None)
        if p is not None:
            fw.append(feasible_fraction(r, 4096, dev0, p.copy(), sg))
    print(f"      {sg:>7.2f} {np.mean(fc):>13.4f} {np.mean(fw) if fw else float('nan'):>16.4f}")

if a.sigmas:
    print(f"\n  sigma sweep at K={Ks[0]}, {a.iters} iters:")
    print(f"  {'start':<6} {'sigma':>7} {'cost excess vs QP (median)':>27} {'force RMS':>10}")
    for start in ("cold", "warm"):
        for sg in [float(x) for x in a.sigmas.split(",")]:
            a.sigma = sg
            gaps, rms = [], []
            for i in idx:
                r = recs[i]
                if start == "cold":
                    mu0 = np.zeros(r["nv"], np.float32)
                else:
                    p = next((recs[j]["sol"] for j in range(i - 1, -1, -1)
                              if recs[j]["nv"] == r["nv"]), None)
                    mu0 = (p if p is not None else np.zeros(r["nv"], np.float32)).copy()
                x = sample_solve(r, Ks[0], dev0, mu0, a.iters)
                _, _, J = cost_np(x, r); _, _, Jq = cost_np(r["sol"], r)
                gaps.append(J - Jq); rms.append(float(np.sqrt(np.mean((x - r["sol"]) ** 2))))
            print(f"  {start:<6} {sg:>7.2f} {np.median(gaps):>27.4f} {np.median(rms):>10.3f}")
    sys.exit(0)

# ---- the sampler ----
print(f"\n  sampler: {a.iters} refinement iterations, sigma={a.sigma}, temp={a.temp}, "
      f"penalty={a.lam:g}")
print(f"\n  {'start':<6} {'K':>7} {'cost excess vs QP (median)':>27} {'p90':>10} "
      f"{'max viol':>10} {'force RMS':>10}   wall/solve")
for start in ("cold", "warm"):
    for K in Ks:
        gaps, viols, rms, times = [], [], [], {}
        for dev in devs:
            # warm up the backend so the first kernel launch is not in the timing
            r0 = recs[idx[0]]
            sample_solve(r0, K, dev, np.zeros(r0["nv"], np.float32), 1)
            t0 = time.perf_counter()
            for i in idx:
                r = recs[i]
                if start == "cold":
                    mu0 = np.zeros(r["nv"], np.float32)
                else:
                    p = next((recs[j]["sol"] for j in range(i - 1, -1, -1)
                              if recs[j]["nv"] == r["nv"]), None)
                    mu0 = (p if p is not None else np.zeros(r["nv"], np.float32)).copy()
                x = sample_solve(r, K, dev, mu0, a.iters)
                if dev == devs[-1]:
                    _, mv, J = cost_np(x, r)
                    _, _, Jq = cost_np(r["sol"], r)
                    gaps.append(J - Jq); viols.append(mv)
                    rms.append(float(np.sqrt(np.mean((x - r["sol"]) ** 2))))
            times[dev] = (time.perf_counter() - t0) / len(idx) * 1e3
        tstr = "  ".join(f"{d}={times[d]:.2f}ms" for d in devs)
        print(f"  {start:<6} {K:>7} {np.median(gaps):>27.4f} {np.percentile(gaps,90):>10.4f} "
              f"{np.median(viols):>10.3g} {np.median(rms):>10.3f}   {tstr}")

print("\n  Read the cost-excess column against the warm-start control above it: a "
      "\n  sampler that does not beat 'reuse the previous answer' is not solving "
      "\n  anything, it is riding the warm start.")
