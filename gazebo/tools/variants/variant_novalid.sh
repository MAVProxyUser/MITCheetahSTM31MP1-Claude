# HEAD minus cc97788's gate: capture the yaw datum at tick 0 again (whatever the packet says).
sed -i '' 's/if(_b_first_visit && this->_stateEstimatorData.vectorNavData->valid){/if(_b_first_visit){/' common/src/Controllers/OrientationEstimator.cpp
grep -q 'if(_b_first_visit){' common/src/Controllers/OrientationEstimator.cpp || { echo "variant_novalid: sed did not apply"; exit 1; }
