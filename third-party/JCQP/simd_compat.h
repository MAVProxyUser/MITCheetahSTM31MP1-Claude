// SIMD compatibility shim for JCQP's Cholesky solvers.
//
// The solvers were hand-written with x86 AVX2 intrinsics. On x86 we use the real
// <immintrin.h>. On other targets (e.g. the STM32MP1 Cortex-A7, armv7l) we provide
// scalar implementations of exactly the intrinsics JCQP uses. gcc -O3 lowers the
// float loops to NEON and the double loops to fused VFP (vfma.f64), so numerics
// match the fused-multiply-add semantics of the AVX2 originals.
#pragma once

#if defined(__x86_64__) || defined(__i386__) || defined(_M_X64) || defined(_M_IX86)

#include <immintrin.h>

#else  // ---- portable scalar emulation (ARM / other) ----

#include <cmath>

typedef struct { double v[4]; } __m256d;   // 4 x double
typedef struct { float  v[8]; } __m256;    // 8 x float

static inline __m256d _mm256_set1_pd(double a) { __m256d r; r.v[0]=r.v[1]=r.v[2]=r.v[3]=a; return r; }
static inline __m256  _mm256_set1_ps(float a)  { __m256 r; for (int i=0;i<8;++i) r.v[i]=a; return r; }

static inline __m256d _mm256_loadu_pd(const double* p) { __m256d r; for (int i=0;i<4;++i) r.v[i]=p[i]; return r; }
static inline __m256  _mm256_loadu_ps(const float*  p) { __m256 r; for (int i=0;i<8;++i) r.v[i]=p[i]; return r; }

static inline void _mm256_storeu_pd(double* p, __m256d a) { for (int i=0;i<4;++i) p[i]=a.v[i]; }
static inline void _mm256_storeu_ps(float*  p, __m256  a) { for (int i=0;i<8;++i) p[i]=a.v[i]; }

// Intel order: e3 is the highest lane, e0 the lowest (memory order v[0]=e0).
static inline __m256d _mm256_set_pd(double e3, double e2, double e1, double e0) {
  __m256d r; r.v[0]=e0; r.v[1]=e1; r.v[2]=e2; r.v[3]=e3; return r;
}

static inline __m256d _mm256_mul_pd(__m256d a, __m256d b) {
  __m256d r; for (int i=0;i<4;++i) r.v[i]=a.v[i]*b.v[i]; return r;
}

//  fmadd :  a*b + c        fnmadd : -(a*b) + c     (both fused -> single rounding)
static inline __m256d _mm256_fmadd_pd (__m256d a, __m256d b, __m256d c) { __m256d r; for (int i=0;i<4;++i) r.v[i]=std::fma( a.v[i], b.v[i], c.v[i]); return r; }
static inline __m256  _mm256_fmadd_ps (__m256  a, __m256  b, __m256  c) { __m256  r; for (int i=0;i<8;++i) r.v[i]=std::fma( a.v[i], b.v[i], c.v[i]); return r; }
static inline __m256d _mm256_fnmadd_pd(__m256d a, __m256d b, __m256d c) { __m256d r; for (int i=0;i<4;++i) r.v[i]=std::fma(-a.v[i], b.v[i], c.v[i]); return r; }
static inline __m256  _mm256_fnmadd_ps(__m256  a, __m256  b, __m256  c) { __m256  r; for (int i=0;i<8;++i) r.v[i]=std::fma(-a.v[i], b.v[i], c.v[i]); return r; }

#endif
