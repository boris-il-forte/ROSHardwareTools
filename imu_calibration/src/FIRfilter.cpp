/*
 * FIRfilter.cpp
 *
 *  Created on: 12 dic 2015
 *      Author: dave
 */

#include "FIRfilter.h"

void FIRfilter::add(arma::vec& input)
{
	history.push_back(input);

	if(history.size() == 7000)
		filtfilt();
}

arma::mat FIRfilter::filtfilt()
{
	size_t maxT = history.size();
	size_t elemN = history.back().size();

	arma::mat result(maxT, elemN);

	arma::mat inputMat(maxT, elemN);

	for(unsigned int i = 0; i < maxT; i++)
		inputMat.row(i) = history[i];

	arma::mat forward(maxT, elemN);

	//filter initial condition
	unsigned int nb = b.n_elem;
	unsigned int na = 1;
	unsigned int nfilt = std::max(nb, na);
	unsigned int nfact = std::max(1u, 3 * (nfilt - 1)); //length of edge transients

	// Zero pad shorter coefficient vector as needed
	arma::vec a = arma::zeros(nfilt, 1);
	a(0) = 1;

	arma::vec zi;
	if (nfilt > 1)
	{
		//create location mat
		arma::uvec v1 = arma::linspace<arma::uvec>(0, nfilt - 2, nfilt - 1);
		arma::uvec v2 = arma::linspace<arma::uvec>(1, nfilt - 2, nfilt - 2);
		arma::uvec v3 = arma::linspace<arma::uvec>(0, nfilt - 3, nfilt - 2);

		arma::umat rows = arma::join_horiz(arma::join_horiz(v1.t(), v2.t()),
					v3.t());
		arma::umat cols = arma::join_horiz(arma::ones<arma::umat>(1, nfilt - 1),
					arma::join_horiz(v2, v2));

		arma::umat locations = arma::join_vert(rows, cols);

		//create value vector
		arma::vec vals = arma::join_vert(arma::vec({1 + a(2)}), a.rows(3, nfilt).t());
		vals = arma::join_vert(vals, arma::ones(1, nfilt - 2));
		vals = arma::join_vert(vals, -arma::ones(1, nfilt - 2));

		//create the system to find initial state
		arma::mat rhs = b.rows(1, nfilt-1) - b(0) * a.rows(1, nfilt-1);
		arma::sp_mat lhs(locations, vals);

		//solve the system
		zi = arma::spsolve(lhs, rhs);
	}
	else
		zi = arma::zeros(0, 1);

	std::cout << zi << std::endl;

	//forward computation
	for (unsigned int i = 0; i < elemN; i++)
	{
		arma::vec extended = arma::join_vert(zi*inputMat(0, 1), inputMat.col(i));
		arma::vec extendedFiltered = arma::conv(extended, b);
		forward.col(i) = extendedFiltered.rows(zi.n_elem +1, extendedFiltered.n_elem -1);
	}

	//backward computation
	forward = arma::flipud(forward);
	for (unsigned int i = 0; i < elemN; i++)
		result.col(i) = arma::conv(forward.col(i), b);

	result = arma::flipud(result);

	return result;

}

