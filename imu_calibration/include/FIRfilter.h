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

#ifndef INCLUDE_FIRFILTER_H_
#define INCLUDE_FIRFILTER_H_

#include <armadillo>

class FIRfilter
{
public:
	arma::mat filtfilt();
	void add(arma::vec& input);


private:
	std::vector<arma::vec> history;

private:
	const arma::vec b =
	{ 5.6328e-05, -1.4631e-19, -0.00012722, -0.00028444, -0.00037983,
	-0.0003054, 5.6758e-19, 0.00048756, 0.00097437, 0.0011889, 0.00088714,
	-1.3267e-18, -0.0012592, -0.0023999, -0.0028086, -0.0020196, 2.4352e-18,
	0.002693, 0.0049988, 0.0057134, 0.0040228, -3.8087e-18, -0.0051795,
	-0.0094801, -0.010709, -0.0074694, 5.2675e-18, 0.0095115, 0.017389,
	0.019688, 0.01382, -6.5708e-18, -0.018107, -0.03394, -0.039799, -0.029341,
	7.4747e-18, 0.045535, 0.09941, 0.15038, 0.18683, 0.20005, 0.18683, 0.15038,
	0.09941, 0.045535, 7.4747e-18, -0.029341, -0.039799, -0.03394, -0.018107,
	-6.5708e-18, 0.01382, 0.019688, 0.017389, 0.0095115, 5.2675e-18, -0.0074694,
	-0.010709, -0.0094801, -0.0051795, -3.8087e-18, 0.0040228, 0.0057134,
	0.0049988, 0.002693, 2.4352e-18, -0.0020196, -0.0028086, -0.0023999,
	-0.0012592, -1.3267e-18, 0.00088714, 0.0011889, 0.00097437, 0.00048756,
	5.6758e-19, -0.0003054, -0.00037983, -0.00028444, -0.00012722, -1.4631e-19,
	5.6328e-05

	};
};

#endif /* INCLUDE_FIRFILTER_H_ */
