# HEAD with the pre-packet accelerometer initialised to the RESTING reading (+g) instead of zero:
# zero means "free fall" to the KF until the first packet lands.
sed -i '' 's/Vec3<float> accelerometer = Vec3<float>::Zero();/Vec3<float> accelerometer = Vec3<float>(0.f, 0.f, 9.81f);/' common/include/SimUtilities/IMUTypes.h
grep -q 'accelerometer = Vec3<float>(0.f, 0.f, 9.81f);' common/include/SimUtilities/IMUTypes.h || { echo "variant_accelg: sed did not apply"; exit 1; }
