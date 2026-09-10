#include <cstdlib>
#include "Utilities/CtrlTuning.h"

#include "Gait.h"

// Offset - Duration Gait
OffsetDurationGait::OffsetDurationGait(int nSegment, Vec4<int> offsets, Vec4<int> durations, const std::string &name) :
  _offsets(offsets.array()),
  _durations(durations.array()),
  _nIterations(nSegment)
{

  _name = name;
  // allocate memory for MPC gait table
  _mpc_table = new int[nSegment * 4];

  _offsetsFloat = offsets.cast<float>() / (float) nSegment;
  _durationsFloat = durations.cast<float>() / (float) nSegment;

  _stance = durations[0];
  _swing = nSegment - durations[0];
}

MixedFrequncyGait::MixedFrequncyGait(int nSegment, Vec4<int> periods, float duty_cycle, const std::string &name) {
  _name = name;
  _duty_cycle = duty_cycle;
  _mpc_table = new int[nSegment * 4];
  _periods = periods;
  _nIterations = nSegment;
  _iteration = 0;
  _phase.setZero();
}

OffsetDurationGait::~OffsetDurationGait() {
  delete[] _mpc_table;
}

MixedFrequncyGait::~MixedFrequncyGait() {
  delete[] _mpc_table;
}

Vec4<float> OffsetDurationGait::getContactState() {
  Array4f progress = _phase - _offsetsFloat;

  for(int i = 0; i < 4; i++)
  {
    if(progress[i] < 0) progress[i] += 1.;
    if(progress[i] > _durationsFloat[i])
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / _durationsFloat[i];
    }
  }

  //printf("contact state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
  return progress.matrix();
}

Vec4<float> MixedFrequncyGait::getContactState() {
  Array4f progress = _phase;

  for(int i = 0; i < 4; i++) {
    if(progress[i] < 0) progress[i] += 1.;
    if(progress[i] > _duty_cycle) {
      progress[i] = 0.;
    } else {
      progress[i] = progress[i] / _duty_cycle;
    }
  }

  //printf("contact state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
  return progress.matrix();
}

Vec4<float> OffsetDurationGait::getSwingState()
{
  Array4f swing_offset = _offsetsFloat + _durationsFloat;
  for(int i = 0; i < 4; i++)
    if(swing_offset[i] > 1) swing_offset[i] -= 1.;
  Array4f swing_duration = 1. - _durationsFloat;

  Array4f progress = _phase - swing_offset;

  for(int i = 0; i < 4; i++)
  {
    if(progress[i] < 0) progress[i] += 1.f;
    if(progress[i] > swing_duration[i])
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / swing_duration[i];
    }
  }

  //printf("swing state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
  return progress.matrix();
}

Vec4<float> MixedFrequncyGait::getSwingState() {

  float swing_duration = 1.f - _duty_cycle;
  Array4f progress = _phase - _duty_cycle;
  for(int i = 0; i < 4; i++) {
    if(progress[i] < 0) {
      progress[i] = 0;
    } else {
      progress[i] = progress[i] / swing_duration;
    }
  }

  //printf("swing state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
  return progress.matrix();
}


bool OffsetDurationGait::getFlightState() {
  // Port of Unitree's OffsetDurationGait::getFlightState (0xf7880). A leg is in
  // STANCE while its wrapped progress through the cycle is still inside its
  // duration; if every leg has passed that point, all four are swinging and the
  // robot is airborne.
  int phase = _iteration % _nIterations;
  for (int i = 0; i < 4; i++) {
    int progress = phase - _offsets[i];
    if (progress < 0) progress += _nIterations;
    if ((float)progress < (float)_durations[i]) return false;   // still standing
  }
  return true;
}

int* OffsetDurationGait::getMpcTable()
{

  //printf("MPC table:\n");
  for(int i = 0; i < _nIterations; i++)
  {
    // MIT's +1 here is not a lead: upstream calls setIterations BEFORE the
    // tick's iterationCounter++, so _iteration is one segment stale at the
    // solve tick and the +1 makes step 0 the segment now starting (lead 0).
    // This port restores _iteration from the post-increment counter (the
    // async prefetch's restore) so the same +1 IS a lead here, and until
    // 2026-09-10 a pointer alias on the prefetched table added one more:
    // the shipped behaviour was a PHYSICAL lead of 3 - the stance pair's
    // MPC force cut 66 ms before its scheduled swing. THE KNOB IS ONE LESS
    // THAN THE PHYSICAL LEAD: the solver's step-0 table is table[seg + knob]
    // (verified at the solver input, $STM32MP1_MPC_IN=2), and the torque
    // reaches the legs one segment later than that index implies (measured
    // from the knee-torque cliff in the bridge dump: knob 0 -> cut 22 ms
    // before the swing, 1 -> 44, 3 -> 88; where that step lives is not yet
    // located). Measured on hp_gap20 (ISSUES.md OPEN-28, 60 + 60 interleaved
    // runs): shorter physical leads collapse MORE (1: 4/15 mid-course, 2:
    // 5/15, 3: 0/15 on the same transport) and longer ones collapse most
    // (4: 18/20 and 6/6). So the default is 2 = physical 3 = bit-for-bit what
    // every result in this tree was measured at. Setting this to 3 on
    // 2026-09-10 gave physical 4 and 6 collapses in 6 runs. Unitree's build
    // has no +1 at all.
    static const int sched_lead =
        ctrl_tuning::integer("CTRL_MPC_SCHED_LEAD", 2);
    int iter = (i + _iteration + sched_lead) % _nIterations;
    Array4i progress = iter - _offsets;
    for(int j = 0; j < 4; j++)
    {
      if(progress[j] < 0) progress[j] += _nIterations;
      if(progress[j] < _durations[j])
        _mpc_table[i*4 + j] = 1;
      else
        _mpc_table[i*4 + j] = 0;

      //printf("%d ", _mpc_table[i*4 + j]);
    }
    //printf("\n");
  }



  return _mpc_table;
}

int* MixedFrequncyGait::getMpcTable() {
  //printf("MPC table (%d):\n", _iteration);
  for(int i = 0; i < _nIterations; i++) {
    for(int j = 0; j < 4; j++) {
      int progress = (i + _iteration + 1) % _periods[j];  // progress
      if(progress < (_periods[j] * _duty_cycle)) {
        _mpc_table[i*4 + j] = 1;
      } else {
        _mpc_table[i*4 + j] = 0;
      }
      //printf("%d %d (%d %d) | ", _mpc_table[i*4 + j], progress, _periods[j], (int)(_periods[j] * _duty_cycle));
    }

    //printf("%d %d %d %d (%.3f %.3f %.3f %.3f)\n", _mpc_table[i*4], _mpc_table[i*4 + 1], _mpc_table[i*4 + ])
    //printf("\n");
  }
  return _mpc_table;
}

void OffsetDurationGait::setIterations(int iterationsPerMPC, int currentIteration)
{
  _iteration = (currentIteration / iterationsPerMPC) % _nIterations;
  _phase = (float)(currentIteration % (iterationsPerMPC * _nIterations)) / (float) (iterationsPerMPC * _nIterations);
}

void MixedFrequncyGait::setIterations(int iterationsBetweenMPC, int currentIteration) {
  _iteration = (currentIteration / iterationsBetweenMPC);// % _nIterations;
  for(int i = 0; i < 4; i++) {
    int progress_mult = currentIteration % (iterationsBetweenMPC * _periods[i]);
    _phase[i] = ((float)progress_mult) / ((float) iterationsBetweenMPC * _periods[i]);
    //_phase[i] = (float)(currentIteration % (iterationsBetweenMPC * _periods[i])) / (float) (iterationsBetweenMPC * _periods[i]);
  }

  //printf("phase: %.3f %.3f %.3f %.3f\n", _phase[0], _phase[1], _phase[2], _phase[3]);

}

int OffsetDurationGait::getCurrentGaitPhase() {
  return _iteration;
}

int MixedFrequncyGait::getCurrentGaitPhase() {
  return 0;
}

float OffsetDurationGait::getCurrentSwingTime(float dtMPC, int leg) {
  // PER-LEG, as Unitree's build of this same class does (it indexes
  // _durations[leg] at 0xf7420 rather than using a scalar). Upstream MIT throws
  // the leg away and returns one number for all four, which silently forbids
  // any gait whose legs do not share a duty cycle - MIT only supports that in
  // MixedFrequncyGait. The gaits in this port that need it are the asymmetric
  // ones. Falls back to the scalar when the per-leg table is uniform, so every
  // existing gait is bit-identical.
  if (leg < 0 || leg > 3) return dtMPC * _swing;
  return dtMPC * (float)(_nIterations - _durations[leg]);
}

float MixedFrequncyGait::getCurrentSwingTime(float dtMPC, int leg) {
  return dtMPC * (1. - _duty_cycle) * _periods[leg];
}

float OffsetDurationGait::getCurrentStanceTime(float dtMPC, int leg) {
  // PER-LEG - see getCurrentSwingTime above (Unitree indexes at 0xf7438).
  if (leg < 0 || leg > 3) return dtMPC * _stance;
  return dtMPC * (float)_durations[leg];
}

float MixedFrequncyGait::getCurrentStanceTime(float dtMPC, int leg) {
  return dtMPC * _duty_cycle * _periods[leg];
}

void OffsetDurationGait::debugPrint() {

}

void MixedFrequncyGait::debugPrint() {

}