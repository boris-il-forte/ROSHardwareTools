/*
 * imu_calibration, Calibrates an IMU
 *
 * Copyright (C) 2015 Davide Tateo
 * Versione 1.0
 *
 * This file is part of SerialToRos.
 *
 * SerialToRos is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SerialToRos is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with SerialToRos.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "CalibrationLogic.h"

#include <armadillo>
#include <nlopt.hpp>

#include "Calibrator.h"

CalibrationLogic::CalibrationLogic(ros::NodeHandle& n) :
			n(n)
{
	imuSubscriber = n.subscribe("imu_raw", 1000,
				&CalibrationLogic::imuMsgHandler, this);
}

void CalibrationLogic::imuMsgHandler(const hardware_tools_msgs::ImuRaw& msg)
{
	dataSet.push_back(msg);

}

void CalibrationLogic::calibrateAccelerometer()
{
	// load data
	arma::mat data(3, dataSet.size());

	unsigned int t = 0;
	for (auto msg : dataSet)
	{
		for (int i = 0; i < 3; i++)
			data(i, t) = msg.data[i];
		t++;
	}

	// init optimization
	double bias_a_guess = .5;
	double gain_a_guess = 9.81 / 1000.0;

	arma::vec x0 =
	{ bias_a_guess, bias_a_guess, bias_a_guess, gain_a_guess, gain_a_guess,
	gain_a_guess };
	arma::vec scale = 1 / x0;
	AccMagCalibrator calibrator(data, 9.81, scale);

	// setup optimization algorithm
	nlopt::opt optimizator(nlopt::algorithm::LN_COBYLA, 6);
	optimizator.set_min_objective(AccMagCalibrator::wrapper, &calibrator);
	optimizator.set_xtol_rel(1e-21);
	optimizator.set_ftol_rel(1e-21);
	optimizator.set_maxeval(10000);

	std::vector<double> xv = arma::conv_to<std::vector<double>>::from(x0);

	// optimize function
	double minf;
	if (optimizator.optimize(xv, minf) < 0)
	{
		std::cout << "nlopt failed!" << std::endl;
	}
	else
	{
		std::cout << "found minimum = " << minf << std::endl;
	}

}

void CalibrationLogic::calibrateMagnetometer()
{
	// load data
	arma::mat data(3, dataSet.size());

	unsigned int t = 0;
	for (auto msg : dataSet)
	{
		for (int i = 6; i < 9; i++)
			data(i, t) = msg.data[i];
		t++;
	}

	// init optimization
	double bias_m_guess = -0.1;
	double gain_m_guess = 1.0 / (1000.0 * 0.35);

	arma::vec x0 =
	{ bias_m_guess, bias_m_guess, bias_m_guess, gain_m_guess, gain_m_guess,
	gain_m_guess };
	arma::vec scale = 1 / x0;
	AccMagCalibrator calibrator(data, 1, scale);

	// setup optimization algorithm
	nlopt::opt optimizator(nlopt::algorithm::LN_COBYLA, 6);
	optimizator.set_min_objective(AccMagCalibrator::wrapper, &calibrator);
	optimizator.set_xtol_rel(1e-21);
	optimizator.set_ftol_rel(1e-21);
	optimizator.set_maxeval(10000);

	std::vector<double> xv = arma::conv_to<std::vector<double>>::from(x0);

	// optimize function
	double minf;
	if (optimizator.optimize(xv, minf) < 0)
	{
		std::cout << "nlopt failed!" << std::endl;
	}
	else
	{
		std::cout << "found minimum = " << minf << std::endl;
	}

}

void CalibrationLogic::calibrateGyroscope()
{
	// load data
	arma::mat data(3, dataSet.size());

	unsigned int t = 0;
	for (auto msg : dataSet)
	{
		for (int i = 3; i < 6; i++)
			data(i, t) = msg.data[i];
		t++;
	}

	// compute bias
	arma::mat bias_g(3, data.n_cols);
	arma::vec gain_g(3);

	for (unsigned i = 0; i < 3; i++)
	{
		arma::vec sensMeas = data.row(i).t();
		arma::vec tmp = (arma::linspace(1.0, sensMeas.n_elem + 0.0, sensMeas.n_elem) / sensMeas.n_elem)* (sensMeas(sensMeas.n_elem - 1) - sensMeas(1));
		bias_g.row(i) = sensMeas(0) + tmp;
		sensMeas = sensMeas - bias_g.row(i);

		// init optimization
		double gain_g_guess = 1.0 / (1000.0 * 0.35);

		arma::vec x0 = {gain_g_guess};
		double scale = 1 / gain_g_guess;
		GyroCalibrator calibrator(sensMeas, 1.0/100.0, M_PI, scale);

		// setup optimization algorithm
		nlopt::opt optimizator(nlopt::algorithm::LN_COBYLA, 6);
		optimizator.set_min_objective(GyroCalibrator::wrapper, &calibrator);
		optimizator.set_xtol_rel(1e-21);
		optimizator.set_ftol_rel(1e-21);
		optimizator.set_maxeval(10000);

		std::vector<double> xv = arma::conv_to<std::vector<double>>::from(x0);

		// optimize function
		double minf;
		if (optimizator.optimize(xv, minf) < 0)
		{
			std::cout << "nlopt failed!" << std::endl;
		}
		else
		{
			std::cout << "found minimum = " << minf << std::endl;
		}

		gain_g(i) = xv[0];

	}

}
