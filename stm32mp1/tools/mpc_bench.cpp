/*!
 * mpc_bench - time THE operation that decides which SoC can run this port.
 *
 * The convex-MPC solve was measured at 60-105 ms on this port's Cortex-A7
 * against a 2 ms control period, and every cheaper explanation was ruled out:
 * not the QP iteration budget (nWSR 100/25/10 -> 105/103/104 ms), not the
 * horizon (10/6/4 -> 81/56/57 ms, so not O(n^3) QP work), not the WBC (58 vs
 * 85 ms either way). What dominates is the dense triple product in
 * SolverMPC.cpp:427
 *
 *     qH = 2*(B_qp.transpose() * Sd * B_qp + alpha*I)
 *
 * with B_qp being 13*horizon x 12*horizon. At horizon 10 that is a
 * 130x120 matrix and roughly 4M double-precision MACs.
 *
 * MIT's x86 UP board did the whole solve in 1-2 ms. So the question for any
 * candidate board is not "is it faster" in general - it is how fast it does
 * THIS, in doubles. Run it on the candidate and compare.
 *
 * Build (any platform):
 *     c++ -O3 -march=native -I third-party/eigen stm32mp1/tools/mpc_bench.cpp -o mpc_bench
 * On the A7 (no -march=native; matches the port's own configure):
 *     c++ -O3 -mcpu=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard \
 *         -DEIGEN_MAX_ALIGN_BYTES=0 -DEIGEN_MAX_STATIC_ALIGN_BYTES=0 \
 *         -I third-party/eigen stm32mp1/tools/mpc_bench.cpp -o mpc_bench
 */
#include <Eigen/Dense>
#include <cstdio>
#include <chrono>
#include <vector>
#include <algorithm>

using namespace Eigen;
typedef double fpt;                       // the port solves in double

int main(int argc, char** argv) {
  const int horizon = (argc > 1) ? atoi(argv[1]) : 10;
  const int reps    = (argc > 2) ? atoi(argv[2]) : 50;
  const int R = 13 * horizon, C = 12 * horizon;

  Matrix<fpt, Dynamic, Dynamic> B_qp(R, C), qH(C, C), Sd(R, R), eye(C, C);
  B_qp.setRandom(); Sd.setIdentity(); eye.setIdentity();
  const fpt alpha = 1e-5;

  // one untimed pass so first-touch page faults and any lazy init are paid for
  qH = 2 * (B_qp.transpose() * Sd * B_qp + alpha * eye);

  std::vector<double> ms;
  for (int i = 0; i < reps; ++i) {
    auto t0 = std::chrono::steady_clock::now();
    qH = 2 * (B_qp.transpose() * Sd * B_qp + alpha * eye);
    auto t1 = std::chrono::steady_clock::now();
    ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
  }
  std::sort(ms.begin(), ms.end());
  const double macs = (double)C * C * R;            // the triple product's work
  printf("mpc_bench: horizon=%d  B_qp=%dx%d  qH=%dx%d  ~%.1fM double MACs\n",
         horizon, R, C, C, C, macs / 1e6);
  printf("  min %.2f ms   median %.2f ms   max %.2f ms   (n=%d)\n",
         ms.front(), ms[ms.size()/2], ms.back(), reps);
  printf("  effective %.2f GFLOP/s (2 flops/MAC)\n", 2*macs / (ms[ms.size()/2]/1e3) / 1e9);
  printf("\n  reference points measured on this project:\n");
  printf("    Cortex-A7 (OSD32MP1), whole solve : 60-105 ms   -> cannot run inline\n");
  printf("    MIT's x86 UP board,   whole solve : 1-2 ms\n");
  printf("    async MPC needs 30-40 Hz          -> solve must be well under 25 ms\n");
  printf("  NOTE: this times the dominant PRODUCT, not the whole solve, so a\n");
  printf("  candidate wants headroom on top of the numbers above.\n");
  return 0;
}
