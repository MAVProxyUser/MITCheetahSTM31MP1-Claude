#include <cstdlib>

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
    // MIT shifts the contact schedule one MPC step into the future (+1);
    // Unitree's build of this same function (0xf7758) does NOT. Plausibly MIT
    // compensating for solve latency. Made switchable rather than assumed,
    // because on this port the latency is much larger than on either of theirs.
    static const int sched_lead =
        getenv("CTRL_MPC_SCHED_LEAD") ? atoi(getenv("CTRL_MPC_SCHED_LEAD")) : 1;
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