#ifndef JACOBIANS_HPP
#define JACOBIANS_HPP

#include "astrodynamics.hpp"

// Do not use the namespace Astrodynamics

// Nonlinear relative dynamics in LVLH frame
void GetJacobiansNRKD(Astrodynamics::MatrixA& Ac, Astrodynamics::MatrixB& Bc, const Astrodynamics::State& x, const Astrodynamics::Control& u, const double sma);

void GetA_NRKD(Astrodynamics::MatrixA& A, const Astrodynamics::State& x, const double sma, const double m);
void GetB_NRKD(Astrodynamics::MatrixB& B, const double m);


//CR3BP continuous dynamics jacobians for the Earth-Moon system
void GetJacobiansCR3BP(Astrodynamics::MatrixA& Ac, Astrodynamics::MatrixB& Bc, const Astrodynamics::State& x, const Astrodynamics::Control& u);

void GetAc_CR3BP(Astrodynamics::MatrixA& Ac, const Astrodynamics::State& x, const Astrodynamics::Control& u);
void GetBc_CR3BP(Astrodynamics::MatrixB& Bc, const Astrodynamics::State& x, const Astrodynamics::Control& u);


// 2 body problem continuous dynamics jacobians
void GetJacobians2BP(Astrodynamics::MatrixA& Ac, Astrodynamics::MatrixB& Bc, const Astrodynamics::State& x, const Astrodynamics::Control& u);

void GetAc_2BP(Astrodynamics::MatrixA& Ac, const Astrodynamics::State& x, const Astrodynamics::Control& u);
void GetBc_2BP(Astrodynamics::MatrixB& Bc, const Astrodynamics::State& x, const Astrodynamics::Control& u);





#endif 