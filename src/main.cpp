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
#include "userInput.hpp"
#include <iostream>

int
main(int argc, char * argv[])
{
	constexpr char   defaultInputFileName[] = "input.csv";
	constexpr double defaultTimestep = 0.1;
	constexpr size_t defaultNumberOfParticles = 50;
	constexpr double defaultOdometryStandardDeviation = 1.0;
	constexpr double defaultObservationStandardDeviation = 3.0;

	UserParameters parameters(
		defaultInputFileName,
		defaultTimestep,
		defaultNumberOfParticles,
		defaultOdometryStandardDeviation,
		defaultObservationStandardDeviation);

	if (getUserParameters(parameters, argc, argv))
	{
		return EXIT_FAILURE;
	}

	SLAMState state(
		parameters.numberOfParticles,
		parameters.odometryStandardDeviation,
		parameters.observationStandardDeviation);

	std::cout << "Running " << parameters.measurements.size() << " SLAM iterations with "
		  << parameters.numberOfParticles << " particles..." << std::endl;

	fastSLAM(state, parameters.measurements, parameters.timestep);

	return EXIT_SUCCESS;
}
