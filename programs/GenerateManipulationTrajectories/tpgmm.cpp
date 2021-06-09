/*
 * demo_TPGMMProduct01.cpp
 *
 * Product of Gaussians for a Task-Parametrized GMM.
 *
 * If this code is useful for your research, please cite the related publication:
 * @article{Calinon16JIST,
 *   author="Calinon, S.",
 *   title="A Tutorial on Task-Parameterized Movement Learning and Retrieval",
 *   journal="Intelligent Service Robotics",
 *   publisher="Springer Berlin Heidelberg",
 *   doi="10.1007/s11370-015-0187-9",
 *   year="2016",
 *   volume="9",
 *   number="1",
 *   pages="1--29"
 * }
 *
 * Authors: Sylvain Calinon, Philip Abbet, Andras Kupcsik
 *
 * This file is part of PbDlib, https://www.idiap.ch/software/pbdlib/
 * 
 * PbDlib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 * 
 * PbDlib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with PbDlib. If not, see <https://www.gnu.org/licenses/>.
 */


#include <stdio.h>
#include <float.h>
#include <armadillo>

#include <gfx2.h>
#include <gfx_ui.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw_gl2.h>

#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui_internal.h>

using namespace arma;


/***************************** ALGORITHM SECTION *****************************/

typedef std::vector<vec> vector_list_t;
typedef std::vector<mat> matrix_list_t;


//-----------------------------------------------------------------------------
// Contains all the parameters used by the algorithm. Some of them are
// modifiable through the UI, others are hard-coded.
//-----------------------------------------------------------------------------
struct parameters_t {
	int	   nb_states;		// Number of components in the GMM
	int	   nb_frames;		// Number of candidate frames of reference
	int	   nb_deriv;		// Number of static and dynamic features
	int	   nb_data;			// Number of datapoints in a trajectory <==== not used for product of Gaussians!!!
	float  dt;				// Time step duration					<==== not used for product of Gaussians!!!
};

//-----------------------------------------------------------------------------
// Model trained using the algorithm
//-----------------------------------------------------------------------------
struct model_t {
	parameters_t			   parameters;	// Parameters used to train the model

	// These lists contain one element per GMM state and per frame (access them
	// by doing: mu[state][frame])
	std::vector<vector_list_t> mu;
	std::vector<matrix_list_t> sigma;

	int						   nb_var;
	mat						   pix;
	vec						   priors;
};


//-----------------------------------------------------------------------------
// Represents a coordinate system, aka a reference frame
//-----------------------------------------------------------------------------
struct coordinate_system_t {

	coordinate_system_t(const arma::vec& position, const arma::mat& orientation,
						const parameters_t& parameters) {

		this->position = zeros(2 + (parameters.nb_deriv - 1) * 2);
		this->position(span(0, 1)) = position(span(0, 1));

		this->orientation = kron(eye(parameters.nb_deriv, parameters.nb_deriv),
								 orientation(span(0, 1), span(0, 1)));
	}

	vec position;
	mat orientation;
};


//-----------------------------------------------------------------------------
// Represents a list of coordinate systems
//-----------------------------------------------------------------------------
typedef std::vector<coordinate_system_t> coordinate_system_list_t;


//-----------------------------------------------------------------------------
// Contains all the needed informations about a demonstration, like:
//	 - its reference frames
//	 - the trajectory originally drawn by the user
//	 - the resampled trajectory
//	 - the trajectory expressed in each reference frame
//-----------------------------------------------------------------------------
class Demonstration
{
public:
	Demonstration(coordinate_system_list_t coordinate_systems,
				  const std::vector<arma::vec>& points,
				  const parameters_t& parameters)
	: coordinate_systems(coordinate_systems)
	{
		points_original = mat(2, points.size());

		for (size_t i = 0; i < points.size(); ++i) {
			points_original(0, i) = points[i](0);
			points_original(1, i) = points[i](1);
		}

		update(parameters);
	}


	//-------------------------------------------------------------------------
	// Resample the trajectory and recompute it in each reference frame
	// according to the provided parameters
	//-------------------------------------------------------------------------
	void update(const parameters_t& parameters)
	{
		// Resampling of the trajectory
		arma::vec x = points_original.row(0).t();
		arma::vec y = points_original.row(1).t();
		arma::vec x2(parameters.nb_data);
		arma::vec y2(parameters.nb_data);

		arma::vec from_indices = arma::linspace<arma::vec>(0, points_original.n_cols - 1, points_original.n_cols);
		arma::vec to_indices = arma::linspace<arma::vec>(0, points_original.n_cols - 1, parameters.nb_data);

		interp1(from_indices, x, to_indices, x2, "*linear");
		interp1(from_indices, y, to_indices, y2, "*linear");

		points = mat(2 * parameters.nb_deriv, parameters.nb_data);
		points(span(0), span::all) = x2.t();
		points(span(1), span::all) = y2.t();

		// Compute the derivatives
		mat D = (diagmat(ones(1, parameters.nb_data - 1), -1) -
				 eye(parameters.nb_data, parameters.nb_data)) / parameters.dt;

		D(parameters.nb_data - 1, parameters.nb_data - 1) = 0.0;

		points(span(2, 3), span::all) = points(span(0, 1), span::all) * pow(D, 1);

		// Compute the points in each coordinate system
		points_in_coordinate_systems.clear();

		for (int m = 0; m < coordinate_systems.size(); ++m) {
			points_in_coordinate_systems.push_back(
				pinv(coordinate_systems[m].orientation) *
				(points - repmat(coordinate_systems[m].position, 1, parameters.nb_data))
			);
		}
	}


	//-------------------------------------------------------------------------
	// Returns the coordinates of a point in a specific reference frame of
	// the demonstration
	//-------------------------------------------------------------------------
	arma::vec convert_to_coordinate_system(const arma::vec& point, int frame) const {
		vec original_point = zeros(points.n_rows);
		original_point(span(0, 1)) = point(span(0, 1));

		vec result = pinv(coordinate_systems[frame].orientation) *
					 (original_point - coordinate_systems[frame].position);

		return result(span(0, 1));
	}


public:
	coordinate_system_list_t coordinate_systems;
	arma::mat				 points_original;
	arma::mat				 points;
	matrix_list_t			 points_in_coordinate_systems;
};


//-----------------------------------------------------------------------------
// Represents a list of demonstrations
//-----------------------------------------------------------------------------
typedef std::vector<Demonstration> demonstration_list_t;




/******************************* MAIN FUNCTION *******************************/

int main(int argc, char **argv) {

	arma_rng::set_seed_random();

	// Model
	model_t model;

	// Parameters
	model.parameters.nb_states = 4;
	model.parameters.nb_frames = 2;
	model.parameters.nb_deriv  = 2;
	model.parameters.nb_data   = 100;
	model.parameters.dt        = 0.1f;


    // List of demonstrations and reproductions
	demonstration_list_t demos;

    vector_list_t trajectory;

	return 0;
}
