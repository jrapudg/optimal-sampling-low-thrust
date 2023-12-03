#ifndef JACOBIANS_PENDELUM_HPP
#define JACOBIANS_PENDELUM_HPP

#include "pendelum.hpp"

void GetAc_pendelum(Pendelum::MatrixA& Ac, const Pendelum::State& x, const Pendelum::Control& u);

void GetBc_pendelum(Pendelum::MatrixB& Bc, const Pendelum::State& x, const Pendelum::Control& u);

#endif 