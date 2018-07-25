/*
 * global.h
 *
 *  Created on: Apr 5, 2013
 *      Authors: vgomez, Sep Thijssen
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#include <vector>
#include <math.h>
#include <iostream>

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>

using namespace std;

typedef vector<double> vec;
typedef vector<vec> vvec;

template<typename G>
ostream& operator<<(ostream& os, const vector<G>& v)
{
	typename vector<G>::const_iterator it;
	for (it=v.begin(); it!=v.end(); it++)
		os << *it << " ";
	os << endl;
	return os;
}

#endif /* GLOBAL_H_ */
