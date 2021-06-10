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

#include <kdl/frames.hpp>

using namespace arma;

/***************************** ALGORITHM SECTION *****************************/

typedef std::vector<vec> vector_list_t;
typedef std::vector<mat> matrix_list_t;

//-----------------------------------------------------------------------------
// Contains all the parameters used by the algorithm. Some of them are
// modifiable through the UI, others are hard-coded.
//-----------------------------------------------------------------------------
struct parameters_t
{
	int nb_states; // Number of components in the GMM
	int nb_frames; // Number of candidate frames of reference
	int nb_deriv;  // Number of static and dynamic features
	int nb_data;   // Number of datapoints in a trajectory <==== not used for product of Gaussians!!!
	float dt;	   // Time step duration					<==== not used for product of Gaussians!!!
};

//-----------------------------------------------------------------------------
// Model trained using the algorithm
//-----------------------------------------------------------------------------
struct model_t
{
	parameters_t parameters; // Parameters used to train the model

	// These lists contain one element per GMM state and per frame (access them
	// by doing: mu[state][frame])
	std::vector<vector_list_t> mu;
	std::vector<matrix_list_t> sigma;

	int nb_var;
	mat pix;
	vec priors;
};

//-----------------------------------------------------------------------------
// Represents a coordinate system, aka a reference frame
//-----------------------------------------------------------------------------
struct coordinate_system_t
{

	coordinate_system_t(const arma::vec &position, const arma::mat &orientation,
						const parameters_t &parameters)
	{

		this->position = zeros(3 + (parameters.nb_deriv - 1) * 3);
		this->position(span(0, 2)) = position(span(0, 2));
		this->orientation = kron(eye(parameters.nb_deriv, parameters.nb_deriv),
								 orientation(span(0, 2), span(0, 2)));
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
				  const std::vector<arma::vec> &points,
				  const parameters_t &parameters)
		: coordinate_systems(coordinate_systems)
	{
		points_original = mat(3, points.size());

		for (size_t i = 0; i < points.size(); ++i)
		{
			points_original(0, i) = points[i](0);
			points_original(1, i) = points[i](1);
			points_original(2, i) = points[i](2);
		}

		update(parameters);
	}

	//-------------------------------------------------------------------------
	// Resample the trajectory and recompute it in each reference frame
	// according to the provided parameters
	//-------------------------------------------------------------------------
	void update(const parameters_t &parameters)
	{
		// Resampling of the trajectory
		arma::vec x = points_original.row(0).t();
		arma::vec y = points_original.row(1).t();
		arma::vec z = points_original.row(2).t();

		arma::vec x2(parameters.nb_data);
		arma::vec y2(parameters.nb_data);
		arma::vec z2(parameters.nb_data);

		arma::vec from_indices = arma::linspace<arma::vec>(0, points_original.n_cols - 1, points_original.n_cols);
		arma::vec to_indices = arma::linspace<arma::vec>(0, points_original.n_cols - 1, parameters.nb_data);

		interp1(from_indices, x, to_indices, x2, "*linear");
		interp1(from_indices, y, to_indices, y2, "*linear");
		interp1(from_indices, z, to_indices, z2, "*linear");

		points = mat(3 * parameters.nb_deriv, parameters.nb_data);
		points(span(0), span::all) = x2.t();
		points(span(1), span::all) = y2.t();
		points(span(2), span::all) = z2.t();

		// // Compute the derivatives

		// mat D = (diagmat(ones(1, parameters.nb_data - 1), -1) - eye(parameters.nb_data, parameters.nb_data)) / parameters.dt;

		// D(parameters.nb_data - 1, parameters.nb_data - 1) = 0.0;

		// points(span(2, 3), span::all) = points(span(0, 1), span::all) * pow(D, 1);

		// Compute the points in each coordinate system
		points_in_coordinate_systems.clear();

		for (int m = 0; m < coordinate_systems.size(); ++m)
		{
			points_in_coordinate_systems.push_back(
				pinv(coordinate_systems[m].orientation) *
				(points - repmat(coordinate_systems[m].position, 1, parameters.nb_data)));
		}
	}

	//-------------------------------------------------------------------------
	// Returns the coordinates of a point in a specific reference frame of
	// the demonstration
	//-------------------------------------------------------------------------
	arma::vec convert_to_coordinate_system(const arma::vec &point, int frame) const
	{
		vec original_point = zeros(points.n_rows);
		original_point(span(0, 1)) = point(span(0, 1));

		vec result = pinv(coordinate_systems[frame].orientation) *
					 (original_point - coordinate_systems[frame].position);

		return result(span(0, 1));
	}

public:
	coordinate_system_list_t coordinate_systems;
	arma::mat points_original;
	arma::mat points;
	matrix_list_t points_in_coordinate_systems;
};

//-----------------------------------------------------------------------------
// Represents a list of demonstrations
//-----------------------------------------------------------------------------
typedef std::vector<Demonstration> demonstration_list_t;

//-----------------------------------------------------------------------------
// Return the likelihood of datapoint(s) to be generated by a Gaussian
// parameterized by center and covariance
//-----------------------------------------------------------------------------
arma::vec gaussPDF(const mat &data, colvec mu, mat sigma)
{

	int nb_var = data.n_rows;
	int nb_data = data.n_cols;

	mat data2 = data.t() - repmat(mu.t(), nb_data, 1);
	vec prob = sum((data2 * inv(sigma)) % data2, 1);

	prob = exp(-0.5 * prob) / sqrt(pow((2 * datum::pi), nb_var) * det(sigma) + DBL_MIN);

	return prob;
}

//-----------------------------------------------------------------------------
// Training of a task-parameterized Gaussian mixture model (GMM) with an
// expectation-maximization (EM) algorithm.
//
// The approach allows the modulation of the centers and covariance matrices of
// the Gaussians with respect to external parameters represented in the form of
// candidate coordinate systems.
//-----------------------------------------------------------------------------
void train_EM_tensorGMM(const demonstration_list_t &demos,
						model_t &model)
{

	const int nb_max_steps = 100;				 // Maximum number of iterations allowed
	const int nb_min_steps = 5;					 // Minimum number of iterations allowed
	const double max_diff_log_likelihood = 1e-5; // Likelihood increase threshold
	// to stop the algorithm
	const double diag_reg_fact = 1e-2; // Regularization term is optional

	cube data(model.nb_var, model.parameters.nb_frames,
			  demos.size() * model.parameters.nb_data);

	for (int n = 0; n < demos.size(); ++n)
	{
		for (int m = 0; m < model.parameters.nb_frames; ++m)
		{
			data(span::all, span(m), span(n * model.parameters.nb_data, (n + 1) * model.parameters.nb_data - 1)) =
				demos[n].points_in_coordinate_systems[m];
		}
	}

	std::vector<double> log_likelihoods;

	for (int iter = 0; iter < nb_max_steps; ++iter)
	{

		// E-step
		mat L = ones(model.parameters.nb_states, data.n_slices);

		for (int i = 0; i < model.parameters.nb_states; ++i)
		{
			for (int m = 0; m < model.parameters.nb_frames; ++m)
			{

				// Matricization/flattening of tensor
				mat data_mat(data(span::all, span(m), span::all));

				vec gamma0 = gaussPDF(data_mat, model.mu[i][m], model.sigma[i][m]);

				L(i, span::all) = L(i, span::all) % gamma0.t();
			}

			L(i, span::all) = L(i, span::all) * model.priors(i);
		}

		mat gamma = L / repmat(sum(L, 0) + DBL_MIN, L.n_rows, 1);
		mat gamma2 = gamma / repmat(sum(gamma, 1), 1, data.n_slices);

		model.pix = gamma2;

		// M-step
		for (int i = 0; i < model.parameters.nb_states; ++i)
		{

			// Update priors
			model.priors(i) = sum(gamma(i, span::all)) / data.n_slices;

			for (int m = 0; m < model.parameters.nb_frames; ++m)
			{

				// Matricization/flattening of tensor
				mat data_mat(data(span::all, span(m), span::all));

				// Update mu
				model.mu[i][m] = data_mat * gamma2(i, span::all).t();

				// Update sigma
				mat data_tmp = data_mat - repmat(model.mu[i][m], 1, data.n_slices);
				model.sigma[i][m] = data_tmp * diagmat(gamma2(i, span::all)) * data_tmp.t() +
									eye(data_tmp.n_rows, data_tmp.n_rows) * diag_reg_fact;
			}
		}

		// Compute average log-likelihood
		log_likelihoods.push_back(vec(sum(log(sum(L, 0)), 1))[0] / data.n_slices);

		// Stop the algorithm if EM converged (small change of log-likelihood)
		if (iter >= nb_min_steps)
		{
			if (log_likelihoods[iter] - log_likelihoods[iter - 1] < max_diff_log_likelihood)
				break;
		}
	}
}

//-----------------------------------------------------------------------------
// Initialization of Gaussian Mixture Model (GMM) parameters by clustering an
// ordered dataset into equal bins
//-----------------------------------------------------------------------------
void init_tensorGMM_kbins(const demonstration_list_t &demos,
						  model_t &model)
{

	model.priors.resize(model.parameters.nb_states);
	model.mu.clear();
	model.sigma.clear();

	model.nb_var = demos[0].points_in_coordinate_systems[0].n_rows;

	// Initialize bins
	uvec t_sep = linspace<uvec>(0, model.parameters.nb_data - 1,
								model.parameters.nb_states + 1);

	struct bin_t
	{
		mat data;
		vec mu;
		mat sigma;
	};

	std::vector<bin_t> bins;
	for (int i = 0; i < model.parameters.nb_states; ++i)
	{
		bin_t bin;
		bin.data = zeros(model.nb_var * model.parameters.nb_frames + 1, //adding time as last dimension
						 demos.size() * (t_sep(i + 1) - t_sep(i) + 1));

		bins.push_back(bin);
	}

	// Split each demonstration in K equal bins
	for (int n = 0; n < demos.size(); ++n)
	{

		for (int i = 0; i < model.parameters.nb_states; ++i)
		{
			int bin_size = t_sep(i + 1) - t_sep(i) + 1;

			for (int m = 0; m < model.parameters.nb_frames; ++m)
			{
				bins[i].data(span(m * model.nb_var, (m + 1) * model.nb_var - 1),
							 span(n * bin_size, (n + 1) * bin_size - 1)) =
					demos[n].points_in_coordinate_systems[m](span::all, span(t_sep(i), t_sep(i + 1)));
			}
		}
	}

	// Calculate statistics on bin data
	for (int i = 0; i < model.parameters.nb_states; ++i)
	{
		bins[i].mu = mean(bins[i].data, 1);
		bins[i].sigma = cov(bins[i].data.t());
		model.priors(i) = bins[i].data.n_elem;
	}

	// Reshape GMM into a tensor
	for (int i = 0; i < model.parameters.nb_states; ++i)
	{
		model.mu.push_back(vector_list_t());
		model.sigma.push_back(matrix_list_t());
	}

	for (int m = 0; m < model.parameters.nb_frames; ++m)
	{
		for (int i = 0; i < model.parameters.nb_states; ++i)
		{
			model.mu[i].push_back(bins[i].mu(span(m * model.nb_var, (m + 1) * model.nb_var - 1)));

			model.sigma[i].push_back(bins[i].sigma(span(m * model.nb_var, (m + 1) * model.nb_var - 1),
												   span(m * model.nb_var, (m + 1) * model.nb_var - 1)));
		}
	}

	model.priors /= sum(model.priors);
}

//-----------------------------------------------------------------------------
// Training of the model
//-----------------------------------------------------------------------------
void learn(const demonstration_list_t &demos, model_t &model)
{

	init_tensorGMM_kbins(demos, model);
	train_EM_tensorGMM(demos, model);
}

std::vector<std::array<double, 8>> getTrajectoryFromCsvFile(const std::string &filename)
{
	std::vector<std::array<double, 8>> result;
	std::cout << filename << std::endl;

	std::ifstream csvFile(filename);

	if (!csvFile.is_open())
		throw std::runtime_error("Could not open csv file");

	double val;
	std::string line;
	while (std::getline(csvFile, line))
	{
		std::stringstream ss(line);
		std::array<double, 8> pose;
		unsigned int colIdx = 0;
		while (ss >> val)
		{
			pose[colIdx] = val;
			if (ss.peek() == ',')
			{
				ss.ignore();
			}

			colIdx++;
		}
		result.push_back(pose);
	}

	csvFile.close();
	return result;
}

void printTrajectoryData(const std::vector<std::array<double, 8>> &data)
{
	for (unsigned int i = 0; i < data.size(); i++)
	{
		std::cout << "Frame: " << data[i][0] << " x: " << data[i][1] << " y: " << data[i][2] << " z: " << data[i][3] << " qx: " << data[i][4] << " qy: " << data[i][5] << " qz: " << data[i][6] << " qw: " << data[i][7] << std::endl;
	}
}

/******************************* MAIN FUNCTION *******************************/

int main(int argc, char **argv)
{

	arma_rng::set_seed_random();

	// Model
	model_t model;

	// Parameters
	model.parameters.nb_states = 3;
	model.parameters.nb_frames = 2;
	model.parameters.nb_deriv = 1;
	model.parameters.nb_data = 100;
	model.parameters.dt = 0.1f;

	// List of demonstrations and reproductions
	demonstration_list_t demos;

	for (int nDemo = 1; nDemo < 5; nDemo++)
	{
		vector_list_t trajectory;
		std::string csvFile = "/home/elisabeth/repos/teo-sharon/programs/GenerateManipulationTrajectories/trajectories/test/test-right-arm-motion-smooth" + std::to_string(nDemo) + "-optimized.csv";

		std::vector<std::array<double, 8>> desiredTrajectoryData = getTrajectoryFromCsvFile(csvFile);
		// printTrajectoryData(desiredTrajectoryData);

		// Lets try with only position

		for (unsigned int i = 0; i < desiredTrajectoryData.size(); i++)
		{
			vec position = {desiredTrajectoryData[i][1], desiredTrajectoryData[i][2], desiredTrajectoryData[i][3]};
			// std::cout << "Position: " << position[0] << " " << position[1] << " " << position[2] << std::endl;
			trajectory.push_back(position);
		}

		// How to create the task frames?
		// Need position vec and orientation as a matrix.

		coordinate_system_list_t coordinateSystems;

		//Initial pose frame
		KDL::Rotation rotKdl = KDL::Rotation::Quaternion(desiredTrajectoryData[0][4], desiredTrajectoryData[0][5], desiredTrajectoryData[0][6], desiredTrajectoryData[0][7]);
		arma::mat initialOrientation = {{rotKdl.data[0], rotKdl.data[1], rotKdl.data[2]},
										{rotKdl.data[3], rotKdl.data[4], rotKdl.data[5]},
										{rotKdl.data[6], rotKdl.data[7], rotKdl.data[8]}};
		arma::vec initialPosition = {desiredTrajectoryData[0][1], desiredTrajectoryData[0][2], desiredTrajectoryData[0][3]};

		coordinate_system_t initialFrame(initialPosition, initialOrientation, model.parameters);

		//Final pose frame
		rotKdl = KDL::Rotation::Quaternion(desiredTrajectoryData[desiredTrajectoryData.size() - 1][4], desiredTrajectoryData[desiredTrajectoryData.size() - 1][5], desiredTrajectoryData[desiredTrajectoryData.size() - 1][6], desiredTrajectoryData[desiredTrajectoryData.size() - 1][7]);
		arma::mat finalOrientation = {{rotKdl.data[0], rotKdl.data[1], rotKdl.data[2]},
									  {rotKdl.data[3], rotKdl.data[4], rotKdl.data[5]},
									  {rotKdl.data[6], rotKdl.data[7], rotKdl.data[8]}};
		arma::vec finalPosition = {desiredTrajectoryData[desiredTrajectoryData.size() - 1][1], desiredTrajectoryData[desiredTrajectoryData.size() - 1][2], desiredTrajectoryData[desiredTrajectoryData.size() - 1][3]};

		coordinate_system_t finalFrame(finalPosition, finalOrientation, model.parameters);

		coordinateSystems.push_back(initialFrame);
		coordinateSystems.push_back(finalFrame);

		//Create demonstration

		Demonstration demonstration(coordinateSystems, trajectory, model.parameters);
		demos.push_back(demonstration);

		//Check the transformation in the task frames.
		std::cout << "Points in initial pose task frame" << std::endl;
		demonstration.points_in_coordinate_systems[0].print();

		std::cout << "Points in final pose task frame" << std::endl;
		demonstration.points_in_coordinate_systems[1].print();
	}
	// TODO! Create training function
	learn(demos, model);

	return 0;
}
