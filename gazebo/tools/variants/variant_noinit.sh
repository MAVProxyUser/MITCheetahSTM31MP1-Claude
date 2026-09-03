# HEAD minus c90e0ca's zero-init: IMU fields and the datum start uninitialised again (the pre-OPEN-6 state).
sed -i '' -e 's/Vec3<float> accelerometer = Vec3<float>::Zero();/Vec3<float> accelerometer;/' \
          -e 's/Vec3<float> gyro = Vec3<float>::Zero();/Vec3<float> gyro;/' \
          -e 's/Quat<float> quat = (Quat<float>() << 1.f, 0.f, 0.f, 0.f).finished();/Quat<float> quat;/' common/include/SimUtilities/IMUTypes.h
sed -i '' 's/Quat<T> _ori_ini_inv = (Quat<T>() << T(1), T(0), T(0), T(0)).finished();/Quat<T> _ori_ini_inv;/' common/include/Controllers/OrientationEstimator.h
grep -q 'Quat<float> quat;' common/include/SimUtilities/IMUTypes.h && grep -q 'Quat<T> _ori_ini_inv;' common/include/Controllers/OrientationEstimator.h || { echo "variant_noinit: sed did not apply"; exit 1; }
