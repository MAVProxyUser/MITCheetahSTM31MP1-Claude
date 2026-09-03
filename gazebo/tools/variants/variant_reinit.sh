# HEAD + a DELIBERATE one-shot estimator re-init when the first valid IMU packet lands - the
# thing the old code did by accident through the NaN guard, without the NaN.
python3 - <<'PY'
p="robot/src/RobotRunner.cpp"; s=open(p).read()
# the MAIN-PATH call is the 2-space-indented one (line ~158); the 6-space one
# sits inside the NaN guard and would make the re-init NaN-gated again.
anchor="\n  _stateEstimator->run();\n"
assert s.count(anchor)==1, "main-path anchor must be unique: %d" % s.count(anchor)
ins='''    // VARIANT reinit: re-create the estimators once, the first tick a REAL
    // IMU packet is present - what the NaN guard used to do by accident.
    {
      static bool reinitDone = false;
      if (!reinitDone && vectorNavData && vectorNavData->valid) {
        initializeStateEstimator(_cheaterModeEnabled);
        reinitDone = true;
      }
    }
'''
s=s.replace(anchor, "\n"+ins.rstrip("\n")+"\n"+anchor.lstrip("\n"), 1); open(p,"w").write(s); print("  reinit variant inserted before the first _stateEstimator->run()")
PY
grep -q "VARIANT reinit" robot/src/RobotRunner.cpp || { echo "variant_reinit: did not apply"; exit 1; }
