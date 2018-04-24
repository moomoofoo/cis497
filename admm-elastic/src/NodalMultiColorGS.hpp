// Copyright (c) 2017, University of Minnesota
// 
// ADMM-Elastic Uses the BSD 2-Clause License (http://www.opensource.org/licenses/BSD-2-Clause)
// Redistribution and use in source and binary forms, with or without modification, are
// permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright notice, this list of
//    conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice, this list
//    of conditions and the following disclaimer in the documentation and/or other materials
//    provided with the distribution.
// THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE UNIVERSITY OF MINNESOTA, DULUTH OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
// OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
// IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef ADMM_NCMCGSLINEARSOLVER_H
#define ADMM_NCMCGSLINEARSOLVER_H

#include "LinearSolver.hpp"
#include <iostream>
#include "MCL/GraphColor.hpp"
#include "ConstraintSet.hpp"

namespace admm {

//
// Nodal-Constrained Multi-Color Gauss-Seidel
//
class NodalMultiColorGS : public LinearSolver {
public:
	typedef Eigen::Matrix<double,3,1> Vec3;
	typedef Eigen::Matrix<double,Eigen::Dynamic,1> VecX;
	typedef Eigen::SparseMatrix<double,Eigen::RowMajor> SparseMat;
	typedef std::unordered_map<int,Vec3>::const_iterator PinIter;
	typedef Eigen::Matrix<double,3,2> Mat32;

	int max_iters;
	double m_tol; // convergence tol
	double m_omega; // relaxation paramater (0<w<2)

	NodalMultiColorGS( std::shared_ptr<ConstraintSet> constraints_ ) :
		max_iters(30), m_tol(1e-10), m_omega(1.9), constraints(constraints_) {}

	NodalMultiColorGS() : NodalMultiColorGS(std::make_shared<ConstraintSet>(ConstraintSet())) {}

	void update_system( const SparseMat &A_ ){
		int dof = A_.rows();
		if( dof != A_.cols() || dof == 0 ){
			throw std::runtime_error("**NodalMultiColorGS Error: Bad dimensions in A");
		}
		A = A_; // Storing a copy of A seems to help with performance
		A.makeCompressed();
		mcl::graphcolor::color_matrix<double>( A, A_colors, 3 );
	}

	int solve( VecX &x, const VecX &b0 ){
		logger.reset();

		// If we don't have an initial guess for x, use zeros
		const int dof = A.cols();
		if( x.rows() != dof ){ x = VecX::Zero(dof); }

		// Create the constraint matrix C and return it.
		// It seems to run a bit faster operating on a local copy
		constraints->make_matrix(dof,false,true);
		const SparseMat &C = constraints->m_C;
		const VecX &c = constraints->m_c;
		bool has_collisions = C.nonZeros()>0;
		bool has_pins = constraints->pins.size()>0;

		// If there are self collisions, add penalty terms to A
		// and do a graph coloring. Otherwise, just use the original colors.
		SparseMat AplusCtC = A;
		VecX b = b0;
		std::vector< std::vector<int> > colors;
		if( has_collisions ){
			const SparseMat &Ct = constraints->m_Ct;
			AplusCtC += (SparseMat)(Ct*C);
			b += Ct*c;
			AplusCtC.makeCompressed();
			mcl::graphcolor::color_matrix<double>( AplusCtC, colors, 3 );
		} else { colors = A_colors; }

		// Runtime vars
		VecX residual; // used for convergence test (tol>0)
		double b_norm = 1.0;
		double tol2 = m_tol*m_tol;
		if( m_tol > 0 ){ b_norm = b.squaredNorm(); }
		const int max_threads = omp_get_max_threads();

		// Outer iteration loop
		int iter = 0;
		for( ; iter < max_iters; ++iter ){

			// Loop each color
			const int n_colors = colors.size();
			for( int color=0; color<n_colors; ++color ){

				const std::vector<int> &inds = colors[color];
				const int n_inds = inds.size();

				// Only use threads if we have a large number of indices in
				// the color, otherwise the threading overhead is a waste.
				const int n_threads = n_inds < max_threads ? 1 : max_threads;

				#pragma omp parallel for num_threads(n_threads)
				for( int i=0; i<n_inds; ++i ){
					int idx = inds[i];

					// First, check if the node is pinned, which has highest priority
					if( has_pins ){
						PinIter curr_pin = constraints->pins.find(idx);
						if( curr_pin != constraints->pins.end() ){
							x.segment<3>(idx*3) = curr_pin->second;
							continue;
						}
					}

					// Perform the update
					Vec3 curr_x = segment_update(idx, x, AplusCtC, b, m_omega );

					// Next, see if the node has a linear constraint.
					Vec3 n, p;
					bool hit_obs = constraints->collider->detect_passive( idx, curr_x, n, p );
					if( hit_obs ){
						curr_x = constrained_segment_update(idx, x, AplusCtC, b, m_omega, n, p );
					}

					x.segment<3>(idx*3) = curr_x;

				} // end loop inds

			} // end loop colors

			logger.add(x);
			if( m_tol > 0 ){
				residual = b - AplusCtC*x;
				double err2 = residual.squaredNorm() / b_norm;
				if( err2 < tol2 ){ break; }
			}

		} // end loop GS iters

		logger.finalize(AplusCtC,x,b);
		return iter;
	} // end gs solve

	// Creates an ortho projection matrix G (Eq. 47 in 10.1109/TVCG.2017.2730875)
	static inline Mat32 orthoG( const Vec3 &n );

protected:

	std::shared_ptr<ConstraintSet> constraints;
	std::vector< std::vector<int> > A_colors;
	SparseMat A; // copy of A

	static inline Vec3 segment_update( int idx, const VecX &x,
		const SparseMat &A, const VecX &b, double omega );

	static inline Vec3 constrained_segment_update( int idx, const VecX &x,
		const SparseMat &A, const VecX &b, double omega, const Vec3 &n, const Vec3 &p );

}; // end class multi colored gauss seidel


//
// Other functions
//

// Creates an ortho projection matrix G (Eq. 47 in 10.1109/TVCG.2017.2730875)
inline NodalMultiColorGS::Mat32 NodalMultiColorGS::orthoG( const Vec3 &n ){
	Vec3 not_n = n[0] > 0.999 ? Vec3(0,0,1) : Vec3(1,0,0);
	Vec3 u = not_n.cross(n); u.normalize();
	Vec3 v = n.cross(u); v.normalize();
	Mat32 G; G.col(0) = u; G.col(1) = v;
	return G;
}


inline NodalMultiColorGS::Vec3 NodalMultiColorGS::segment_update( int idx, const VecX &x,
	const SparseMat &A, const VecX &b, double omega ){

	int idx3 = idx*3;
	Vec3 bi = b.segment<3>(idx3);
	const Vec3 curr_x = x.segment<3>(idx3);
	Vec3 new_x = curr_x;

	for( int s=0; s<3; ++s ){
		SparseMat::InnerIterator rit(A,idx3+s);
		double LUx = 0.0;
		double aii = 0.0;
		for( ; rit; ++rit){
			// Sometimes zero values get added when multiplying/adding/etc
			if( std::abs(rit.value())<=0.0 ){ continue; }
			int c_idx = rit.col();
			if( c_idx == idx3+s ){ // diagonal element
				aii = rit.value();
				continue;
			}
			LUx += rit.value()*x[c_idx];
		}

		if( LinearSolver::is_zero(aii) ){
			std::cerr << "**NodalMultiColorGS Error: Zero on diagonal" << std::endl;
			throw std::runtime_error("Exiting...");
		}

		double x_new = (bi[s] - LUx)/aii;
		new_x[s] = (1.0-omega)*curr_x[s] + omega*x_new;

	} // end loop segment

	return new_x;

} // end segment update


inline NodalMultiColorGS::Vec3 NodalMultiColorGS::constrained_segment_update( int idx, const VecX &x,
	const SparseMat &A, const VecX &b, double omega, const Vec3 &n, const Vec3 &p ){
	typedef Eigen::Matrix<double,2,1> Vec2;
	int idx3 = idx*3;
	(void)(omega); // Don't use over-relaxation with constrained update.
	Vec3 bi = b.segment<3>(idx3);
	const Vec3 curr_x = x.segment<3>(idx3);
	Vec3 new_x = curr_x;

	Vec3 LUx(0,0,0);
	Vec3 aii(0,0,0);
	for( int s=0; s<3; ++s ){
		SparseMat::InnerIterator rit(A,idx3+s);
		for( ; rit; ++rit){
			// Sometimes zero values get added through
			// multiplication of the various matrices.
			if( std::abs(rit.value())<=0.0 ){ continue; }
			int c_idx = rit.col();
			if( c_idx == idx3+s ){ // diagonal element
				aii[s] = rit.value();
				continue;
			}
			LUx[s] += rit.value()*x[c_idx];
		}

		if( LinearSolver::is_zero(aii[s]) ){
			std::cerr << "**NodalMultiColorGS Error: Zero on diagonal" << std::endl;
			throw std::runtime_error("Exiting...");
		}
	}


	Vec3 delta_x = Vec3(
		(bi[0] - LUx[0])/aii[0],
		(bi[1] - LUx[1])/aii[1],
		(bi[2] - LUx[2])/aii[2]
	) - p;

	// Solve constrained to a plane
	Mat32 G = orthoG( n );
	Vec2 x_tan = G.transpose() * delta_x;
	new_x = G * x_tan + p;
	return new_x;

} // end constrained segment update


} // ns admm

#endif
