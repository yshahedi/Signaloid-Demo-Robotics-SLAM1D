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

#include "fastSLAM.hpp"
#include "uncertain.h"
#include <cmath>
#include <iostream>

typedef enum Constants
{
	kFirstMoment = 1,
	kSecondMoment = 2,
	kSizeZero = 0,
	kSizeOne = 1,
} Constants;

constexpr double pi = 3.14159265358979323846;

constexpr double zeroInitialiser = 0.0;

/**
 *	@brief Class to hold a cumulative sum of values, in order to calculate their mean.
 */
class CumulativeSum
{
public:
	double sum;
	size_t size;

	/**
	 *	@brief Add a value to the cumulative sum.
	 *
	 *	@param value : value to add.
	 */
	void
	add(const double value);

	double
	getMean(void) const;

	CumulativeSum(void);

	CumulativeSum(const double initialValue);
};

CumulativeSum::CumulativeSum(void) : sum(zeroInitialiser), size(kSizeZero)
{
}

CumulativeSum::CumulativeSum(const double initialValue) : sum{initialValue}, size(kSizeOne)
{
}

void
CumulativeSum::add(const double value)
{
	size++;
	sum += value;
}

double
CumulativeSum::getMean(void) const
{
	return sum / size;
}

/*
 *	Default particle importance weight.
 */
constexpr double defaultImportanceWeight = 1.0;

/*
 *	Measurement noise standard deviation for odometry measurements.
 */
static double odometryUncertaintyStandardDeviation = 1.0;

/*
 *	Measurement noise standard deviation for landmark observations.
 */
static double observationNoiseStandardDeviation = 3.0;

/*
 *	Random variable used to draw random samples from a uniform distribution.
 */
static double uniformDistribution = libUncertainDoubleUniformDist(0.0, 1.0);

/**
 *	@brief Model of uncertainty in odometry measurements.
 *	@details Uncertainty is modelled with a Gaussian distribution centred on the measurement.
 *
 *	@param measurement : Measurement value.
 *	@return double : Distribution of odometry measurement uncertainty.
 */
static double
odometryUncertaintyFunction(double measurement)
{
	return libUncertainDoubleGaussDist(measurement, odometryUncertaintyStandardDeviation);
}

/**
 *	@brief Model of uncertainty in landmark observations.
 *	@details Uncertainty is modelled with a Gaussian distribution centred on the measurand.
 *
 *	@param measurand : Measurand value.
 *	@return double : Distribution of landmark measurand uncertainty.
 */
static double
observationNoiseFunction(double measurand)
{
	return libUncertainDoubleGaussDist(measurand, observationNoiseStandardDeviation);
}

static inline double
getMean(const double uncertainVariable)
{
	return libUncertainDoubleNthMoment(uncertainVariable, kFirstMoment);
}

static inline double
getVariance(const double uncertainVariable)
{
	return libUncertainDoubleNthMoment(uncertainVariable, kSecondMoment);
}

/**
 *	@brief Calculate particle importance weight, assuming Gaussian uncertainty distributions.
 *	@todo Generalise to non Gaussian distributions, using Laplace architecture.
 *
 *	@param expectedObservation : The expected observation i.e. the mean of the prior
 *	distribution for the observation.
 *	@param observation : The actual observation measurement.
 *	@param variance : The observation variance, a combination of the variance of the
 *	prior distribution for the observation and the variance of the measurement uncertainty.
 *	@return double : Importance weight.
 */
static double
calculateWeight(const double expectedObservation, const double observation, const double variance)
{
	const double error = observation - expectedObservation;
	const double scaleFactor = 1.0 / (sqrt(2.0 * pi * variance));

	return scaleFactor * exp(-0.5 * error * error / variance);
}

/**
 *	@brief Update particle weight based on a new landmark observation.
 *
 *	@param particle : Particle to modify.
 *	@param landmarkID : Key value identifying the observed landmark.
 *	@param observationValue : Observation measurement value.
 */
static void
updateParticleWeight(
	SLAMParticle & particle,
	const size_t   landmarkID,
	const double   observationValue)
{
	const double observationNoiseVariance =
		observationNoiseStandardDeviation * observationNoiseStandardDeviation;
	const double expectedObservation = getMean(particle.map[landmarkID]) - particle.position;
	const double variance = getVariance(particle.map[landmarkID]) + observationNoiseVariance;

	particle.weight *= calculateWeight(expectedObservation, observationValue, variance);
}

/**
 *	@brief Update the landmark map for a particle, based on a new landmark observation.
 *
 *	@param particle : Particle to modify.
 *	@param landmarkID : Key value identifying the observed landmark.
 *	@param observationValue : Observation measurement value.
 *	@return double : New uncertain estimate for the landmark position (posterior distribution
 *	given the new observation).
 */
static double
updateParticleMap(SLAMParticle & particle, const size_t landmarkID, const double observationValue)
{
	const double prior = particle.map[landmarkID];
	const double evidence = observationValue + particle.position;
	const double posterior =
		libUncertainDoubleBayesLaplace(&observationNoiseFunction, prior, evidence);
	return posterior;
}

/**
 *	@brief Combine the particles in a vector to obtain a single uncertain estimate for robot
 *	pose and landmark positions.
 *
 *	@param state : Reference to SLAMState to modify.
 */
static void
averageParticles(SLAMState & state)
{
	std::vector<double>                       poseVector;
	std::unordered_map<size_t, CumulativeSum> cumulativeMap;

	poseVector.reserve(state.particles.size());

	for (const auto & particle : state.particles)
	{
		poseVector.push_back(particle.position);

		for (const auto & landmark : particle.map)
		{
			const size_t landmarkID = landmark.first;
			const double landmarkEstimate = landmark.second;
			auto         iterator = cumulativeMap.find(landmarkID);

			if (iterator == cumulativeMap.end())
			{
				/*
				 *	Landmark not in map yet, add it.
				 */
				cumulativeMap.insert(std::make_pair(
					landmarkID,
					CumulativeSum(landmarkEstimate)));
			}
			else
			{
				/*
				 *	Add to existing map entry.
				 */
				cumulativeMap[landmarkID].add(landmarkEstimate);
			}
		}
	}

	state.pose = libUncertainDoubleDistFromSamples(poseVector.data(), poseVector.size());

	for (const auto & landmark : cumulativeMap)
	{
		state.map[landmark.first] = landmark.second.getMean();
	}
}

SLAMState::SLAMState(
	const size_t numberOfParticles,
	const double odometryStandardDeviation,
	const double observationStandardDeviation)
	: odometryStandardDeviation{odometryStandardDeviation},
	  observationStandardDeviation{observationStandardDeviation}
{
	particles.reserve(numberOfParticles);

	for (size_t i = 0; i < numberOfParticles; i++)
	{
		constexpr double initialParticleWeight = 1.0;
		constexpr double initialParticlePosition = 0.0;

		SLAMParticle particle = {
			.weight = initialParticleWeight,
			.position = initialParticlePosition,
		};
		particles.push_back(particle);
	}
}

/**
 *	@brief Update a particle's state estimates based on new measurement data.
 *
 *	@param measurements : Reference to new measurement data.
 *	@param particle : Reference to particle to modify.
 */
static void
updateParticle(const SLAMMeasurements & measurements, SLAMParticle & particle)
{
	for (auto const & observation : measurements.observations)
	{
		const size_t landmarkID = observation.first;
		const double observationValue = observation.second;
		auto         iterator = particle.map.find(landmarkID);

		if (iterator == particle.map.end())
		{
			/*
			 *	Landmark not in map, create new entry.
			 */
			particle.weight *= defaultImportanceWeight;
			particle.map[landmarkID] =
				observationNoiseFunction(observationValue) + particle.position;
		}
		else
		{
			/*
			 *	Landmark has been observed previously.
			 */
			updateParticleWeight(particle, landmarkID, observationValue);
			particle.map[landmarkID] =
				updateParticleMap(particle, landmarkID, observationValue);
		}
	}
}

/**
 *	@brief Resample particles, based on their relative weights.
 *
 *	@param particles : Reference to vector of particles to modify.
 */
static void
resampleParticles(std::vector<SLAMParticle> & particles)
{
	std::vector<SLAMParticle> resampledParticles;
	std::vector<double>       cumulativeWeights;
	double                    cumulativeSum = 0;

	resampledParticles.reserve(particles.size());
	cumulativeWeights.reserve(particles.size());

	/*
	 *	Calculate cumulative weights
	 */
	for (size_t i = 0; i < particles.size(); i++)
	{
		cumulativeSum += particles.at(i).weight;
		cumulativeWeights.push_back(cumulativeSum);
	}

	/*
	 *	Normalise cumulative weights
	 */
	for (auto & weight : cumulativeWeights)
	{
		weight /= cumulativeSum;
	}

	/*
	 *	Resample
	 */
	for (size_t i = 0; i < particles.size(); i++)
	{
		size_t       index;
		const double sample = libUncertainDoubleSample(uniformDistribution);
		SLAMParticle resampledParticle;

		auto lower = std::lower_bound(
			cumulativeWeights.begin(),
			cumulativeWeights.end(),
			sample);
		index = std::distance(cumulativeWeights.begin(), lower);

		resampledParticle = particles.at(index);
		resampledParticle.weight = defaultImportanceWeight;
		resampledParticles.push_back(resampledParticle);
	}

	/*
	 *	Update particles
	 */
	particles = resampledParticles;
}

/**
 *	@brief Perform a single iteration of the fastSLAM algorithm.
 *
 *	@param particles : Particle filter particles.
 *	@param measurements : Measurements for a single timestep.
 *	@param timestep : Time step duration (time between successive measurements/SLAM iterations).
 */
static void
SLAMIteration(
	std::vector<SLAMParticle> & particles,
	SLAMMeasurements &          measurements,
	const double                timestep)
{
	const double uncertainOdometryInput = odometryUncertaintyFunction(measurements.speed);

	for (auto & particle : particles)
	{
		/*
		 *	Update position estimate using odometry.
		 */

		particle.position += timestep * libUncertainDoubleSample(uncertainOdometryInput);

		/*
		 *	Update state estimate based on measurements.
		 */
		updateParticle(measurements, particle);
	}

	resampleParticles(particles);
}

void
fastSLAM(
	SLAMState &                     state,
	std::vector<SLAMMeasurements> & measurementsVector,
	const double                    timestep)
{
	state.pose = 0;
	state.map.clear();

	odometryUncertaintyStandardDeviation = state.odometryStandardDeviation;
	observationNoiseStandardDeviation = state.observationStandardDeviation;

	for (size_t i = 0; i < measurementsVector.size(); i++)
	{
		SLAMIteration(state.particles, measurementsVector.at(i), timestep);

		averageParticles(state);

		std::cout << "Iteration " << i + 1 << ":" << std::endl;
		std::cout << "Robot position: " << state.pose << std::endl;
		std::cout << "Landmark positions:" << std::endl;

		for (auto const & landmarkEntry : state.map)
		{
			std::cout << "	Landmark " << landmarkEntry.first << ": "
				  << landmarkEntry.second << std::endl;
		}

		std::cout << std::endl;
	}
}
