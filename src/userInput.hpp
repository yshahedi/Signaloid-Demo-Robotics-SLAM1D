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

#include "fastSLAM.hpp"
#include <string>
#include <vector>

/**
 *	@brief Structure to hold user input data.
 *
 */
typedef struct UserParameters
{
	std::string                   inputFileName;
	std::vector<SLAMMeasurements> measurements;
	double                        timestep;
	size_t                        numberOfParticles;
	double                        odometryStandardDeviation;
	double                        observationStandardDeviation;

	UserParameters(
		const std::string & inputFileName,
		const double        timestep,
		const size_t        numberOfParticles,
		const double        odometryStandardDeviation,
		const double        observationStandardDeviation);
} UserParameters;

/**
 *	@brief Parse command line inputs.
 *
 *	@param parameters : Reference to location to store user input data.
 *	@param argc : Argument count.
 *	@param argv : Argument vector.
 *	@return int : 1 if program should exit (e.g. user only wanted to see the help message),
 * 	0 if program should continue to run.
 */
int
getUserParameters(UserParameters & parameters, int argc, char * argv[]);
