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

#include "Calibrator.h"


double Calibrator::wrapper(unsigned int n, const double* x, double* grad, void* o)
{
	arma::vec df;
	arma::vec parV(const_cast<double*>(x), n, true);
	double value = static_cast<Calibrator*>(o)->objFun(parV, df);

	//Save gradient
	if (grad)
	{
		for (int i = 0; i < df.n_elem; ++i)
		{
			grad[i] = df[i];
		}
	}

	return value;
}

Calibrator::~Calibrator()
{

}

AccMagCalibrator::AccMagCalibrator(const arma::mat& data, double magnitude, const arma::vec& scale)
	: data(data), magnitude(magnitude), scale(scale)
{

}

double AccMagCalibrator::objFun(const arma::vec& x, arma::vec& df)
{
    arma::vec xs = x / scale;
    arma::vec bias = xs.rows(0, 2);
    arma::vec gain = xs.rows(3,5);

    arma::mat M = arma::diagmat(gain)*data;
    M.each_col() -= bias;

	arma::vec error(M.n_cols);

	for(unsigned int i = 0; i < M.n_cols; i++)
		error(i) = magnitude - arma::norm(M.col(i), 1);

    return arma::as_scalar(error.t() * error);

}

GyroCalibrator::GyroCalibrator(const arma::vec& data, double samplePeriod, double target, double scale)
	: data(data), samplePeriod(samplePeriod), target(target), scale(scale)
{

}

double GyroCalibrator::objFun(const arma::vec& x, arma::vec& df)
{
	arma::vec xs = x / scale;
    double gain = xs(0);
    arma::mat omega = gain * data;
    double angle = 0;
    for (unsigned int t = 1; t < data.n_cols; t++)
        angle = angle + omega(t) * samplePeriod;

    return std::pow(angle - target, 2);
}



