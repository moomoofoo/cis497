// Copyright (c) 2017 University of Minnesota
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
//
// By Matt Overby (http://www.mattoverby.net)

#include "Application.hpp"
#include "MCL/MeshIO.hpp"
#include <Eigen/Sparse>

using namespace mcl;

typedef Eigen::Matrix<double,3,1> Vec3;


int main(int argc, char **argv){
	
	mcl::TetMesh::Ptr lteeth_tet = mcl::TetMesh::create();
	std::stringstream file4;
	file4 << ADMMELASTIC_ROOT_DIR << "/cis497/data/lower_teeth_reduced";
	mcl::meshio::load_elenode( lteeth_tet.get(), file4.str() );
	mcl::XForm<float> xfr = mcl::xform::make_rot(150.f, mcl::Vec3f(0,1,0));
	lteeth_tet->apply_xform(xfr);
	lteeth_tet->flags |= binding::LINEAR;
	std::shared_ptr<admm::PassiveCollision> lteeth_collider =
		std::make_shared<admm::PassiveMesh>( admm::PassiveMesh(lteeth_tet) );
	lteeth_collider->a[1] = 0.05f;
	lteeth_collider->v[1] = -0.1f;
	lteeth_collider->c1 = 0.5;
	//lteeth_collider->w[0] = 7.5f;
	//lteeth_collider->alpha[0] = -3.75f;
	//lteeth_collider->c2 = 0.5;
	//lteeth_collider->pivot[2] = -1.2f;
	//std::cout << lteeth_collider->pivot[1];

	mcl::TriangleMesh lteeth_tri;
	std::stringstream file5;
	file5 << ADMMELASTIC_ROOT_DIR << "/cis497/data/lower_teeth_reduced.obj";
	mcl::meshio::load_obj( &lteeth_tri, file5.str() );
	lteeth_tri.apply_xform(xfr);
	std::shared_ptr<mcl::TriangleMesh> lteeth_tri_ptr = std::make_shared<mcl::TriangleMesh>(lteeth_tri);
	
	mcl::TetMesh::Ptr uteeth_tet = mcl::TetMesh::create();
	std::stringstream file2;
	file2 << ADMMELASTIC_ROOT_DIR << "/cis497/data/upper_teeth_reduced";
	mcl::meshio::load_elenode( uteeth_tet.get(), file2.str() );
	uteeth_tet->apply_xform(xfr);
	uteeth_tet->flags |= binding::LINEAR;
	std::shared_ptr<admm::PassiveCollision> uteeth_collider =
		std::make_shared<admm::PassiveMesh>( admm::PassiveMesh(uteeth_tet) );

	mcl::TriangleMesh uteeth_tri;
	std::stringstream file3;
	file3 << ADMMELASTIC_ROOT_DIR << "/cis497/data/upper_teeth_reduced.obj";
	mcl::meshio::load_obj( &uteeth_tri, file3.str() );
	uteeth_tri.apply_xform(xfr);
	std::shared_ptr<mcl::TriangleMesh> uteeth_tri_ptr = std::make_shared<mcl::TriangleMesh>(uteeth_tri);

	mcl::TetMesh::Ptr mesh = mcl::TetMesh::create();
	std::stringstream file;
	file << ADMMELASTIC_ROOT_DIR << "/cis497/data/head_reduced2";
	mcl::meshio::load_elenode( mesh.get(), file.str() );
	mesh->flags |= binding::LINEAR;
	mesh->apply_xform(xfr);

	admm::Solver::Settings settings;
	settings.linsolver = 2;
	settings.admm_iters = 10;
	settings.gravity = 0.1f;
	if( settings.parse_args( argc, argv ) ){ return EXIT_SUCCESS; }
	Application app(settings);
	admm::Lame mat = admm::Lame(101000,0.449); 
	app.add_dynamic_mesh( mesh, mat );

	app.add_obstacle( uteeth_collider, uteeth_tri_ptr );
	app.add_obstacle( lteeth_collider, lteeth_tri_ptr );

	// Add a collision floor
	/*float floor_y = 0.f;
	std::shared_ptr<admm::PassiveCollision> floor_collider = 
		std::make_shared<admm::Floor>( admm::Floor(floor_y) );

	// Add a floor renderable
	std::shared_ptr<mcl::TriangleMesh> floor = mcl::factory::make_plane(2,2);
	mcl::XForm<float> xf = mcl::xform::make_trans(0.f,floor_y,0.f) *
		mcl::xform::make_rot(-90.f,mcl::Vec3f(1,0,0)) *
		mcl::xform::make_scale(4.f,4.f,4.f);
	floor->apply_xform(xf);

	app.add_obstacle( floor_collider, floor ); */

	bool success = app.display();
	if( !success ){ return EXIT_FAILURE; }

	return EXIT_SUCCESS;
}

