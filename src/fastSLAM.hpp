/*
 *	Authored 2022, Greg Brooks.
 *
 *	Copyright (c) 2022, Signaloid.
 *
 *	Permission is hereby granted, free of charge, to any person obtaining a copy
 *	of this software and associated documentation files (the "Software"), to deal
 *	in the Software without restriction, including without limitation the rights
 *	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *	copies of the Software, and to permit persons to whom the Software is
 *	furnished to do so, subject to the following conditions:
 *
 *	The above copyright notice and this permission notice shall be included in all
 *	copies or substantial portions of the Software.
 *
 *	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *	SOFTWARE.
 */

#pragma once
#include <stddef.h>
#include <unordered_map>
#include <vector>

/**
 *	@brief Particle data structure for the SLAM particle filter.
 *
 */
typedef struct SLAMParticle
{
	double weight;

	/*
	 *	Point estimate of robot position.
	 */
	double position;

	/*
	 *	Uncertain estimates of landmark locations.
	 */
	std::unordered_map<size_t, double> map;
} SLAMParticle;

/**
 *	@brief Input measurements for a single timestep.
 *
 */
typedef struct SLAMMeasurements
{
	/*
	 *	Odometry input.
	 */
	double speed;

	/*
	 *	Measurements of landmark locations.
	 */
	std::unordered_map<size_t, double> observations;
} SLAMMeasurements;

/**
 *	@brief Data structure to group the SLAM output (pose and map estimates) and particle filter
 *	particles.
 *
 */
typedef struct SLAMState
{
	double                             pose;
	std::unordered_map<size_t, double> map;
	std::vector<SLAMParticle>          particles;
	double                             odometryStandardDeviation;
	double                             observationStandardDeviation;

	SLAMState(
		const size_t numberOfParticles,
		const double odometryStandardDeviation,
		const double observationStandardDeviation);
} SLAMState;

/**
 *	@brief Run fastSLAM over the supplied measurement data.
 *
 *	@param state : Reference to SLAMState variables to store SLAM progress and output.
 *	@param measurementsVector : Reference to measurement data input (a time series vector of
 *	measurements).
 *	@param timestep : Time duration between successive measurements.
 */
void
fastSLAM(
	SLAMState &                     state,
	std::vector<SLAMMeasurements> & measurementsVector,
	const double                    timestep);
