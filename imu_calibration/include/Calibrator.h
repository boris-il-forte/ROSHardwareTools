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

#ifndef INCLUDE_CALIBRATOR_H_
#define INCLUDE_CALIBRATOR_H_

#include <armadillo>

class Calibrator
{
public:
	virtual double objFun(const arma::vec& x, arma::vec& df) = 0;
	static double wrapper(unsigned int n, const double* x, double* grad, void* o);

	virtual ~Calibrator();

};

class AccMagCalibrator : public Calibrator
{
public:
	AccMagCalibrator(const arma::mat& data, double magnitude,const arma::vec& scale);
	virtual double objFun(const arma::vec& x, arma::vec& df) override;

private:
	const arma::mat& data;
	const arma::vec& scale;
	double magnitude;
};

class GyroCalibrator : public Calibrator
{
public:
	GyroCalibrator(const arma::vec& data, double samplePeriod, double target, double scale);
	virtual double objFun(const arma::vec& x, arma::vec& df) override;

private:
	arma::vec data;
	double samplePeriod;
	double target;
	double scale;
};



#endif /* INCLUDE_CALIBRATOR_H_ */
