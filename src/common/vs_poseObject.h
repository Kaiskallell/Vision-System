/**
 * @copyright Copyright (c) 2020 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * @brief Vision System pose defintion class
 *
 */

#ifndef VS_POSE_OBJECT_H
#define VS_POSE_OBJECT_H

struct VSPose
{
	// position center point
	float x = 0;
	float y = 0;
	float z = 0;

	// angles
	float roll = 0;
	float pitch = 0;
	float yaw = 0;
};

// convert from SI-Unit [m] to [mm/10] for Lachmann & Rink (used for x,y,z)
inline int32_t convertTo10thMM(float coordinateInMeter)
{
	constexpr float kConvFactorMeterTo10thMM = 10000.0f;
	return static_cast<int32_t>(coordinateInMeter * kConvFactorMeterTo10thMM);
}

// convert from SI-Unit [rad] to [°/10] for Lachmann & Rink (used for roll, pitch, yaw)
inline int32_t convertTo10thGrad(float radianAngle)
{
	float gradAngle = (radianAngle * 180.0) / M_PI;  // convert form rad to grad
	constexpr float kConvFactorGradTo10thGrad = 10.0f;
	return static_cast<int32_t>(gradAngle * kConvFactorGradTo10thGrad);  // convert form grad to grad/10
}

#endif
