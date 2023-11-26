#ifndef JACOBIANS_HPP
#define JACOBIANS_HPP

#include "astrodynamics.hpp"

// Do not use the namespace Astrodynamics

//CR3BP continuous dynamics jacobians for the Earth-Moon system
void GetJacobiansCR3BP(Astrodynamics::MatrixA& Ac, Astrodynamics::MatrixB& Bc, const Astrodynamics::State& x, const Astrodynamics::Control& u);

void GetAc_CR3BP(Astrodynamics::MatrixA& Ac, const Astrodynamics::State& x, const Astrodynamics::Control& u);
void GetBc_CR3BP(Astrodynamics::MatrixB& Bc, const Astrodynamics::State& x, const Astrodynamics::Control& u);


// 2 body problem continuous dynamics jacobians
void GetJacobians2BP(Astrodynamics::MatrixA& Ac, Astrodynamics::MatrixB& Bc, const Astrodynamics::State& x, const Astrodynamics::Control& u);

void GetAc_2BP(Astrodynamics::MatrixA& Ac, const Astrodynamics::State& x, const Astrodynamics::Control& u);
void GetBc_2BP(Astrodynamics::MatrixB& Bc, const Astrodynamics::State& x, const Astrodynamics::Control& u);





#endif 