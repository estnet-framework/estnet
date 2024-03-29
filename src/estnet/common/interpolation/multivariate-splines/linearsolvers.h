/*
 * This file is part of the Multivariate Splines library.
 * Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com)
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/


#ifndef MS_LINEARSOLVER_H
#define MS_LINEARSOLVER_H

#include "generaldefinitions.h"
#include "estnet/common/matrix/eigen/IterativeLinearSolvers"
#include "estnet/common/matrix/eigen/SparseQR"

namespace MultivariateSplines
{

template<class lhs, class rhs>
class LinearSolver
{
public:
    bool solve(const lhs &A, const rhs &b, rhs &x) const
    {
        if (!consistentData(A, b))
        {
            throw Exception("LinearSolver::solve: Inconsistent matrix dimensions!");
        }

        bool success = doSolve(A, b, x);

        if (!(success && validSolution(A, b, x)))
        {
            throw Exception("LinearSolver::solve: Solver did not converge to acceptable tolerance!");
        }
        return true;
    }

private:
    double tol = 1e-12; // Relative error tolerance

    virtual bool doSolve(const lhs &A, const rhs &b, rhs &x) const = 0;

    bool consistentData(const lhs &A, const rhs &b) const
    {
        return A.rows() == b.rows();
    }

    bool validSolution(const lhs &A, const rhs &b, const rhs &x) const
    {
        //return b.isApprox(A*x);
        double err = (A*x - b).norm() / b.norm();

        return (err <= tol);
    }
};

class DenseQR : public LinearSolver<DenseMatrix, DenseMatrix>
{
private:
    bool doSolve(const DenseMatrix &A, const DenseMatrix &b, DenseMatrix &x) const
    {
        //x = A.colPivHouseholderQr().solve(b);

        // Solve linear system
        Eigen::ColPivHouseholderQR<DenseMatrix> qr(A);
        // Note: qr.info() always returns true
        if (qr.info() == Eigen::Success)
        {
            x = qr.solve(b);

            return true;
        }
        return false;
    }
};

class SparseBiCG : public LinearSolver<SparseMatrix,DenseMatrix>
{
private:
    bool doSolve(const SparseMatrix &A, const DenseMatrix &b, DenseMatrix &x) const
    {
        // Init BiCGSTAB solver (requires square matrices)
        Eigen::BiCGSTAB<SparseMatrix> sparseSolver(A);

        if (sparseSolver.info() == Eigen::Success)
        {
            // Solve LSE
            x = sparseSolver.solve(b);

            return sparseSolver.info() == Eigen::Success;
        }

        return false;
    }
};

class SparseLU : public LinearSolver<SparseMatrix, DenseMatrix>
{
private:
    bool doSolve(const SparseMatrix &A, const DenseMatrix &b, DenseMatrix &x) const
    {
        // Init SparseLU solver (requires square matrices)
        Eigen::SparseLU<SparseMatrix > sparseSolver;
        // Compute the ordering permutation vector from the structural pattern of A
        sparseSolver.analyzePattern(A);
        // Compute the numerical factorization
        sparseSolver.factorize(A);

        if (sparseSolver.info() == Eigen::Success)
        {
            // Solve LSE
            x = sparseSolver.solve(b);

            return sparseSolver.info() == Eigen::Success;
        }

        return false;
    }
};

class SparseQR : public LinearSolver<SparseMatrix, DenseMatrix>
{
private:
    bool doSolve(const SparseMatrix &A, const DenseMatrix &b, DenseMatrix &x) const
    {
        // Init SparseQR solver (works with rectangular matrices)
        Eigen::SparseQR<SparseMatrix, Eigen::COLAMDOrdering<int>> sparseSolver;
        sparseSolver.analyzePattern(A);
        sparseSolver.factorize(A);

        if (sparseSolver.info() == Eigen::Success)
        {
            // Solve LSE
            x = sparseSolver.solve(b);

            return sparseSolver.info() == Eigen::Success;
        }

        return false;
    }
};

} // namespace MultivariateSplines

#endif // MS_LINEARSOLVER_H
