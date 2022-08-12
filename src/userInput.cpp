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

#include "userInput.hpp"
#include <fstream>
#include <inttypes.h>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string.h>
#include <unistd.h>

typedef enum
{
	kIntegerConversionBase = 10,
} Constants;

class ConversionError : public std::runtime_error
{
public:
	ConversionError(const char * c = "") : std::runtime_error(c)
	{
	}
};

/**
 *	@brief Print program usage.
 *
 *	@param stream : Stream to print to e.g., stdout, stderr...
 */
static void
printUsage(std::ostream & stream)
{
	stream << "1D SLAM implementation.\n\n"
		  "	slam-1D [-i file] [-t timestep] [-n number of particles] [-d odometry "
		  "uncertainty standard deviation] [-b observation uncertainty standard deviation] "
		  "[-h help]\n"
		  "	\n"
		  "	-i file: path to CSV file containing input measurements (odometry and "
		  "landmark observations. (Default: \"input.csv\")\n"
		  "	\n"
		  "	-t timestep: time between successive measurements in the input file. "
		  "(Default: 0.1)\n"
		  "	\n"
		  "	-n number of particles: number of particles to use in the particle filter. "
		  "(Default: 50)\n"
		  "	\n"
		  "	-d odometry uncertainty standard deviation: standard deviation of the "
		  "measurement noise for the robot odometry measurements. (Default: 1.0)\n"
		  "	\n"
		  "	-b observation uncertainty standard deviation: standard deviation of the "
		  "measurement noise for the robot's landmark observation measurements. (Default: "
	          "3.0)\n"
		  "	\n"
		  "	-h help: display this help message.\n"
		  "	\n";
}

static double
parseDouble(const char * const c)
{
	double d;
	char * pEnd;

	errno = 0;

	d = strtod(c, &pEnd);

	if (c == pEnd)
	{
		/*
		 *	Conversion was not performed
		 */
		throw ConversionError("Failed to convert string to double.\n");
	}

	if (errno != 0)
	{
		throw std::runtime_error(
			std::string("Failed to convert string \" ") + std::string(c) +
			std::string("\" to a double"));
	}

	return d;
}

static uintmax_t
parseUintMax(const char * const c, const int base)
{
	uintmax_t u;
	char *    pEnd;

	errno = 0;

	u = strtoumax(c, &pEnd, base);

	if (c == pEnd)
	{
		/*
		 *	Conversion was not performed
		 */
		throw ConversionError("Failed to convert string to uintmax_t.\n");
	}

	if (errno != 0)
	{
		throw std::runtime_error(
			std::string("Failed to convert string \" ") + std::string(c) +
			std::string("\" to a uintmax_t"));
	}

	return u;
}

/**
 *	@brief Parse an input data CSV file.
 *
 * 	@param parameters : reference to struct to store parsed information.
 */
static void
readFromInputFile(UserParameters & parameters)
{
	std::string line;

	errno = 0;

	std::ifstream fIn(parameters.inputFileName);

	if (!fIn)
	{
		throw std::runtime_error("Could not open input file.");
	}

	/*
	 *	Discard first line
	 */
	if (!std::getline(fIn, line))
	{
		throw std::runtime_error("Input CSV file is malformed.");
	}

	while (std::getline(fIn, line))
	{
		SLAMMeasurements  newMeasurement;
		std::string       stringBuffer;
		std::stringstream stringStreamLine(line);
		size_t            landmarkID;

		if (!std::getline(stringStreamLine, stringBuffer, ','))
		{
			throw std::runtime_error("Input CSV file is malformed.");
		}

		try
		{
			newMeasurement.speed = parseDouble(stringBuffer.c_str());
		}
		catch (const ConversionError & e)
		{
			throw std::runtime_error(
				std::string("Failed to read speed value from string: \"") +
				stringBuffer + std::string("\"\n"));
		}

		landmarkID = 0;
		while (std::getline(stringStreamLine, stringBuffer, ','))
		{
			try
			{
				const double newObservation = parseDouble(stringBuffer.c_str());
				newMeasurement.observations[landmarkID] = newObservation;
			}
			catch (const ConversionError & e)
			{
			}

			landmarkID++;
			if (landmarkID == 0)
			{
				throw std::overflow_error("Too many landmarks in input data.\n");
			}
		}

		parameters.measurements.push_back(std::move(newMeasurement));
	}
}

UserParameters::UserParameters(
	const std::string & inputFileName,
	const double        timestep,
	const size_t        numberOfParticles,
	const double        odometryStandardDeviation,
	const double        observationStandardDeviation)
	: inputFileName{inputFileName},
	  timestep{timestep},
	  numberOfParticles{numberOfParticles},
	  odometryStandardDeviation{odometryStandardDeviation},
	  observationStandardDeviation{observationStandardDeviation}
{
}

int
getUserParameters(UserParameters & parameters, int argc, char * argv[])
{

	int opt;

	while ((opt = getopt(argc, argv, ":i:t:n:d:b:h")) != -1)
	{
		switch (opt)
		{
		case 'i':
			parameters.inputFileName = std::string(optarg);
			break;
		case 't':
			try
			{
				parameters.timestep = parseDouble(optarg);
			}
			catch (const ConversionError & e)
			{
				throw std::runtime_error(
					std::string(
						"Failed to read timestep value from string: \"") +
					std::string(optarg) + std::string("\"\n"));
			}
			break;
		case 'n':
			try
			{
				parameters.numberOfParticles =
					parseUintMax(optarg, kIntegerConversionBase);
			}
			catch (const ConversionError & e)
			{
				throw std::runtime_error(
					std::string("Failed to read number of particles from "
				                    "string: \"") +
					std::string(optarg) + std::string("\"\n"));
			}
			break;
		case 'd':
			try
			{
				parameters.odometryStandardDeviation = parseDouble(optarg);
			}
			catch (const ConversionError & e)
			{
				throw std::runtime_error(
					std::string("Failed to read odometry uncertainty standard "
				                    "deviation value from string: \"") +
					std::string(optarg) + std::string("\"\n"));
			}
			break;
		case 'b':
			try
			{
				parameters.observationStandardDeviation = parseDouble(optarg);
			}
			catch (const ConversionError & e)
			{
				throw std::runtime_error(
					std::string("Failed to read observation uncertainty "
				                    "standard deviation value from string: \"") +
					std::string(optarg) + std::string("\"\n"));
			}
			break;
		case 'h':
			printUsage(std::cout);
			return 1;
		default:
			std::cerr << "Invalid option -" << static_cast<char>(opt) << std::endl;
			printUsage(std::cerr);
			throw std::invalid_argument("");
		}
	}

	readFromInputFile(parameters);
	return 0;
}
