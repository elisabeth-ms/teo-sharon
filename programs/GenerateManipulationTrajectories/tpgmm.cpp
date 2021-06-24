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

#include <matplot/matplot.h>

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

		this->position = zeros(2 + (parameters.nb_deriv - 1) * 2);
		this->position(span(0, 2)) = position(span(0, 2));
		std::cout << "position" << std::endl;
		this->position.print();
		this->orientation = kron(eye(parameters.nb_deriv - 1, parameters.nb_deriv - 1),
								 orientation(span(0, 2), span(0, 2)));
		std::cout << "orientation" << std::endl;
		this->orientation.print();
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
		points_original = mat(4, points.size()); // Added one for time(4 dim)

		for (size_t i = 0; i < points.size(); ++i)
		{
			points_original(0, i) = points[i](0);
			points_original(1, i) = points[i](1);
			points_original(2, i) = points[i](2);
			points_original(3, i) = i;
		}

		update(parameters);
	}

	//-------------------------------------------------------------------------
	// Resample the trajectory and recompute it in each reference frame
	// according to the provided parameters
	//-------------------------------------------------------------------------
	void update(const parameters_t &parameters)
	{
		std::cout << "Updating..." << std::endl;
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

		points = mat(2 * parameters.nb_deriv, parameters.nb_data);
		points(span(0), span::all) = x2.t();
		points(span(1), span::all) = y2.t();
		points(span(2), span::all) = z2.t();

		std::cout << "Points " << points.n_cols << " " << points.n_rows << std::endl;
		std::cout << "Points" << std::endl;
		points.print();
		// Compute the derivatives

		mat D = (diagmat(ones(1, parameters.nb_data - 1), -1) - eye(parameters.nb_data, parameters.nb_data)) / parameters.dt;

		D(parameters.nb_data - 1, parameters.nb_data - 1) = 0.0;
		points(span(3, 5), span::all) = points(span(0, 2), span::all) * pow(D, 1);

		// Compute the points in each coordinate system
		points_in_coordinate_systems.clear();

		points = join_vert(points, linspace(0, points.n_cols - 1, points.n_cols).t());
		// Projected trajectories in each task frame
		for (int m = 0; m < coordinate_systems.size(); ++m)
		{
			coordinate_systems[m].orientation.print();
			points_in_coordinate_systems.push_back(join_vert(pinv(coordinate_systems[m].orientation) *
																 (points.rows(0, points.n_rows - 2) - repmat(coordinate_systems[m].position, 1, parameters.nb_data)),
															 points.row(points.n_rows - 1)));
		}
		std::cout << "Updated" << std::endl;
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
// Represents a task frame in joint space
//-----------------------------------------------------------------------------
struct task_frame_t
{

	task_frame_t(const arma::vec &q,
				 const parameters_t &parameters)
	{

		this->q = zeros(2 * (parameters.nb_deriv - 1) + 2);
		// q(span(0, 7)).print();
		this->q(span(0, 7)) = q(span(0, 7));
		// std::cout << "position" << std::endl;
		// this->q.print();
	}
	vec q;
};

//-----------------------------------------------------------------------------
// Represents a list of task frames in joint space
//-----------------------------------------------------------------------------
typedef std::vector<task_frame_t> task_frame_list_t;

//-----------------------------------------------------------------------------
// Contains all the needed informations about a demonstration in joint space, like:
//	 - its task reference frames in joint space
//	 - the trajectory originally optimized from data recorded with the mocap in joint space
//	 - the resampled trajectory in joint space
//	 - the trajectory expressed in each task reference frame
//-----------------------------------------------------------------------------
class DemonstrationJointsSpace
{
public:
	DemonstrationJointsSpace(task_frame_list_t task_frames,
							 const std::vector<arma::vec> &points,
							 const parameters_t &parameters)
		: task_frames(task_frames)
	{
		points_original = mat(9, points.size()); // Added one for time(9 dim)

		for (size_t i = 0; i < points.size(); ++i)
		{
			for (size_t j = 0; j < points[0].size(); j++)
			{
				points_original(j, i) = points[i](j);
			}
			points_original(8, i) = i;
		}

		update(parameters);
	}

	//-------------------------------------------------------------------------
	// Resample the trajectory and recompute it in each reference frame
	// according to the provided parameters
	//-------------------------------------------------------------------------
	void update(const parameters_t &parameters)
	{
		std::cout << "Updating..." << std::endl;
		// Resampling of the trajectory
		arma::vec q0 = points_original.row(0).t();
		arma::vec q1 = points_original.row(1).t();
		arma::vec q2 = points_original.row(2).t();
		arma::vec q3 = points_original.row(3).t();
		arma::vec q4 = points_original.row(4).t();
		arma::vec q5 = points_original.row(5).t();
		arma::vec q6 = points_original.row(6).t();
		arma::vec q7 = points_original.row(7).t();

		arma::vec q0_2(parameters.nb_data);
		arma::vec q1_2(parameters.nb_data);
		arma::vec q2_2(parameters.nb_data);
		arma::vec q3_2(parameters.nb_data);
		arma::vec q4_2(parameters.nb_data);
		arma::vec q5_2(parameters.nb_data);
		arma::vec q6_2(parameters.nb_data);
		arma::vec q7_2(parameters.nb_data);

		arma::vec from_indices = arma::linspace<arma::vec>(0, points_original.n_cols - 1, points_original.n_cols);
		arma::vec to_indices = arma::linspace<arma::vec>(0, points_original.n_cols - 1, parameters.nb_data);

		interp1(from_indices, q0, to_indices, q0_2, "*linear");
		interp1(from_indices, q1, to_indices, q1_2, "*linear");
		interp1(from_indices, q2, to_indices, q2_2, "*linear");
		interp1(from_indices, q3, to_indices, q3_2, "*linear");
		interp1(from_indices, q4, to_indices, q4_2, "*linear");
		interp1(from_indices, q5, to_indices, q5_2, "*linear");
		interp1(from_indices, q6, to_indices, q6_2, "*linear");
		interp1(from_indices, q7, to_indices, q7_2, "*linear");

		points = mat(2 * parameters.nb_deriv, parameters.nb_data);
		points(span(0), span::all) = q0_2.t();
		points(span(1), span::all) = q1_2.t();
		points(span(2), span::all) = q2_2.t();
		points(span(3), span::all) = q3_2.t();
		points(span(4), span::all) = q4_2.t();
		points(span(5), span::all) = q5_2.t();
		points(span(6), span::all) = q6_2.t();
		points(span(7), span::all) = q7_2.t();

		// Compute the derivatives

		mat D = (diagmat(ones(1, parameters.nb_data - 1), -1) - eye(parameters.nb_data, parameters.nb_data)) / parameters.dt;

		D(parameters.nb_data - 1, parameters.nb_data - 1) = 0.0;
		points(span(8, 15), span::all) = points(span(0, 7), span::all) * pow(D, 1);

		// Compute points in each task frame
		points_in_task_frames.clear();

		points = join_vert(points, linspace(0, points.n_cols - 1, points.n_cols).t());
		std::cout << "Points " << points.n_cols << " " << points.n_rows << std::endl;
		std::cout << "Points" << std::endl;
		points.row(1).print();
		// Projected trajectories in each task frame
		for (int m = 0; m < task_frames.size(); ++m)
		{
			points_in_task_frames.push_back(join_vert(points.rows(0, points.n_rows - 2) - repmat(task_frames[m].q, 1, parameters.nb_data), points.row(points.n_rows - 1)));
		}
		points_in_task_frames[0].row(1).print();
		(points.row(1) - task_frames[0].q[1]).print();
		// points_in_task_frames[0].print();
		// 	points_in_coordinate_systems.push_back(join_vert(pinv(coordinate_systems[m].orientation) *
		// 														 (points.rows(0, points.n_rows - 2) - repmat(coordinate_systems[m].position, 1, parameters.nb_data)),
		// 													 points.row(points.n_rows - 1)));
		// }
		std::cout << "Updated" << std::endl;
	}

public:
	task_frame_list_t task_frames;
	arma::mat points_original;
	arma::mat points;
	matrix_list_t points_in_task_frames;
};

//-----------------------------------------------------------------------------
// Represents a list of demonstrations
//-----------------------------------------------------------------------------
typedef std::vector<DemonstrationJointsSpace> demonstration_joint_space_list_t;

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

	arma::Mat<double> testSigma;
	int nb_var = data.n_rows;
	int nb_data = data.n_cols;

	mat data2 = data.t() - repmat(mu.t(), nb_data, 1);
	mat test = (data2 * inv(sigma)) % data2;
	vec prob = sum(test, 1);
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
// Centers and covariance matrices of the nb_States Gaussians in the nb_frames task frames.
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
				model.sigma[i][m].print();

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
// Training of a task-parameterized Gaussian mixture model (GMM) with an
// expectation-maximization (EM) algorithm.
//
// The approach allows the modulation of the centers and covariance matrices of
// the Gaussians with respect to external parameters represented in the form of
// candidate coordinate systems.
// Centers and covariance matrices of the nb_States Gaussians in the nb_frames task frames.
//-----------------------------------------------------------------------------
void train_EM_tensorGMM(const demonstration_joint_space_list_t &demos,
						model_t &model)
{

	const int nb_max_steps = 100;				 // Maximum number of iterations allowed
	const int nb_min_steps = 5;					 // Minimum number of iterations allowed
	const double max_diff_log_likelihood = 1e-3; // Likelihood increase threshold
	// to stop the algorithm
	const double diag_reg_fact = 1e-5; // Regularization term is optional

	cube data(model.nb_var, model.parameters.nb_frames,
			  demos.size() * model.parameters.nb_data);

	for (int n = 0; n < demos.size(); ++n)
	{
		for (int m = 0; m < model.parameters.nb_frames; ++m)
		{
			data(span::all, span(m), span(n * model.parameters.nb_data, (n + 1) * model.parameters.nb_data - 1)) =
				demos[n].points_in_task_frames[m];
		}
	}

	std::vector<double> log_likelihoods;
	std::cout << "E-step" << std::endl;
	for (int iter = 0; iter < nb_max_steps; ++iter)
	{

		// E-step
		mat L = ones(model.parameters.nb_states, data.n_slices);

		for (int i = 0; i < model.parameters.nb_states; ++i)
		{
			std::cout << "model states: " << i << std::endl;

			for (int m = 0; m < model.parameters.nb_frames; ++m)
			{

				// Matricization/flattening of tensor
				mat data_mat(data(span::all, span(m), span::all));
				std::cout << "gaussPDF" << std::endl;
				std::cout << "sigma cols: " << model.sigma[i][m].n_cols << " rows: " << model.sigma[i][m].n_rows << std::endl;
				model.sigma[i][m].print();

				vec gamma0 = gaussPDF(data_mat, model.mu[i][m], model.sigma[i][m]);

				L(i, span::all) = L(i, span::all) % gamma0.t();
			}

			L(i, span::all) = L(i, span::all) * model.priors(i);
		}
		std::cout << "E-step done" << std::endl;

		mat gamma = L / repmat(sum(L, 0) + DBL_MIN, L.n_rows, 1);
		mat gamma2 = gamma / repmat(sum(gamma, 1), 1, data.n_slices);

		model.pix = gamma2;

		// M-step
		std::cout << "M-step" << std::endl;
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

	std::cout << "Nb var: " << model.nb_var << std::endl;

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
// Initialization of Gaussian Mixture Model (GMM) parameters by clustering an
// ordered dataset into equal bins
//-----------------------------------------------------------------------------
void init_tensorGMM_kbins(const demonstration_joint_space_list_t &demos,
						  model_t &model)
{

	model.priors.resize(model.parameters.nb_states);
	model.mu.clear();
	model.sigma.clear();

	model.nb_var = demos[0].points_in_task_frames[0].n_rows;

	std::cout << "Nb var: " << model.nb_var << std::endl;

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
					demos[n].points_in_task_frames[m](span::all, span(t_sep(i), t_sep(i + 1)));
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

	std::cout << "Init tensor GMM kbins" << std::endl;
	init_tensorGMM_kbins(demos, model);
	std::cout << "Train EM tensor GMM" << std::endl;
	train_EM_tensorGMM(demos, model);
}

//-----------------------------------------------------------------------------
// Training of the model
//-----------------------------------------------------------------------------
void learn(const demonstration_joint_space_list_t &demos, model_t &model)
{

	std::cout << "Init tensor GMM kbins" << std::endl;
	init_tensorGMM_kbins(demos, model);
	std::cout << "Train EM tensor GMM" << std::endl;
	train_EM_tensorGMM(demos, model);
}

std::vector<std::array<double, 9>> getTrajectoryFromCsvFile(const std::string &filename)
{
	std::vector<std::array<double, 9>> result;
	std::cout << filename << std::endl;

	std::ifstream csvFile(filename);

	if (!csvFile.is_open())
		throw std::runtime_error("Could not open csv file");

	double val;
	std::string line;
	while (std::getline(csvFile, line))
	{
		std::stringstream ss(line);
		std::array<double, 9> pose;
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
		std::cout << pose[2]<<std::endl;
	}

	csvFile.close();
	return result;
}

void printTrajectoryData(const std::vector<std::array<double, 9>> &data)
{
	for (unsigned int i = 0; i < data.size(); i++)
	{
		std::cout << "Frame: " << data[i][0] << "q0: " << data[i][1] << " q1: " << data[i][2] << " q2: " << data[i][3] << " q3: " << data[i][4] << " q4: " << data[i][5] << " q5: " << data[i][6] << " q6: " << data[i][7] << "q7: " << data[i][8] << std::endl;
	}
}

void computeGMR(model_t model, int nbGMRComponents, arma::cube &muGMR, arma::field<arma::cube> &sigmaGMR, const std::vector<gfx2::transforms_t> &transforms, std::vector<mat> &muPList, std::vector<mat> &sigmaPList, const mat &inputs)
{

	if (model.mu.size() > 0)
	{

		mat H(model.parameters.nb_states, inputs.size());

		//Init GMR mu and sigma
		// arma::cube muGMR(3*model.parameters.nb_deriv, inputs.size(), model.parameters.nb_frames);

		for (unsigned int i = 0; i < model.parameters.nb_frames; i++)
		{
			sigmaGMR(i).set_size(2 * model.parameters.nb_deriv, 2 * model.parameters.nb_deriv, inputs.size());
			sigmaGMR(i) = zeros(2 * model.parameters.nb_deriv, 2 * model.parameters.nb_deriv, inputs.size());
		}

		// Local trajectories distributions in each of the nb_frames task frames
		for (unsigned int m = 0; m < model.parameters.nb_frames; m++)
		{
			// Compute activation weight
			for (unsigned int i = 0; i < model.parameters.nb_states; i++)
			{
				H.row(i) = model.priors(i) * gaussPDF(inputs, model.mu[i][m].row(model.nb_var - 1), model.sigma[i][m].row(model.nb_var - 1).col(model.nb_var - 1)).t();
			}
			H = H / repmat(sum(H + 1e-300), model.parameters.nb_states, 1);

			mat muTmp(2 * model.parameters.nb_deriv, model.parameters.nb_states);

			mat sigmaTmp;

			for (int t = 0; t < inputs.size(); t++)
			{
				// Compute conditional means
				for (unsigned int i = 0; i < model.parameters.nb_states; i++)
				{

					muTmp.col(i) = model.mu[i][m].subvec(0, 2 * model.parameters.nb_deriv - 1) +
								   model.sigma[i][m].col(2 * model.parameters.nb_deriv).rows(0, 2 * model.parameters.nb_deriv - 1) * inv(model.sigma[i][m].row(model.nb_var - 1).col(model.nb_var - 1)) * (inputs(t) - model.mu[i][m].row(model.nb_var - 1));

					muGMR.slice(m).col(t) += H(i, t) * muTmp.col(i);
				}
				std::cout << "muGMR done!" << std::endl;

				// Compute conditional covariances
				for (int i = 0; i < model.parameters.nb_states; i++)
				{
					sigmaTmp = model.sigma[i][m].submat(0, 0, model.nb_var - 2, model.nb_var - 2) -
							   model.sigma[i][m].col(2 * model.parameters.nb_deriv).rows(0, 2 * model.parameters.nb_deriv - 1) *
								   inv(model.sigma[i][m].row(model.nb_var - 1).col(model.nb_var - 1)) *
								   model.sigma[i][m].row(2 * model.parameters.nb_deriv).cols(0, 2 * model.parameters.nb_deriv - 1);
					sigmaGMR(m).slice(t) += H(i, t) * (sigmaTmp + muTmp.col(i) * muTmp.col(i).t());
				}
				sigmaGMR(m).slice(t) += -muGMR.slice(m).col(t) * muGMR.slice(m).col(t).t() + eye(2 * model.parameters.nb_deriv, 2 * model.parameters.nb_deriv) * 1e-4;
				std::cout << "sigmaGMR done!" << std::endl;
			}
		}

		// transform mu/sigma GMR components into coordinate systems

		cube muGMRt = zeros(3, inputs.size(), model.parameters.nb_frames);
		field<cube> sigmaGMRt(model.parameters.nb_frames);

		for (int i = 0; i < model.parameters.nb_frames; i++)
		{
			sigmaGMRt(i).resize(3, 3, inputs.size());
			sigmaGMRt(i) = zeros(3, 3, inputs.size());
		}
		for (int f = 0; f < model.parameters.nb_frames; f++)
		{
			// Transform conditional means and covariances into coordinate system f
			muGMRt.slice(f) = transforms[f].rotation.submat(0, 0, 2, 2) * muGMR.slice(f).rows(0, 2);
			vec b = {transforms[f].position(0), transforms[f].position(1), transforms[f].position(2)};
			muGMRt.slice(f).each_col() += b;

			for (unsigned int t = 0; t < inputs.size(); t++)
			{
				sigmaGMRt(f).slice(t) = transforms[f].rotation.submat(0, 0, 2, 2) * sigmaGMR(f).slice(t).submat(0, 0, 2, 2) * transforms[f].rotation.submat(0, 0, 2, 2).t();
			}
		}

		// Product

		vec maxMuP = zeros(3, 1);
		mat maxSigmaP = eye(3, 3);

		// std::cout << "inputs " << inputs.size() << std::endl;
		// inputs.print();

		std::vector<double> tMean(nbGMRComponents), xMean(nbGMRComponents);

		for (unsigned int t = 0; t < inputs.size(); t++)
		{

			vec muP = zeros(3, 1);
			mat sigmaP = eye(3, 3);

			for (int m = 0; m < model.parameters.nb_frames; m++)
			{

				sigmaP += inv(sigmaGMRt(m).slice(t) + eye(3, 3) * 1e-4);
				muP += inv(sigmaGMRt(m).slice(t) + eye(3, 3) * 1e-4) * muGMRt.slice(m).col(t);
			}

			sigmaP = inv(sigmaP);
			muP = sigmaP * muP;

			std::cout << "t: " << t << std::endl;
			std::cout << "sigma" << std::endl;
			sigmaP.print();
			std::cout << "mu" << std::endl;
			muP.print();

			muPList.push_back(muP);
			sigmaPList.push_back(sigmaP);

			// tMean.at(t) = inputs(t);
			// xMean.at(t) = muP[0];
		}
	}
}

// void batchLQRReproduction(const model_t& model, const coordinate_system_list_t& coordinateSystems, int referenceDemonstrationIndex = 0){
// 	std::vector< vector_list_t > mu;		// Indexing: mu[state][frame]
// 	std::vector< matrix_list_t > inv_sigma; // Indexing: inv_sigma[state][frame]

// 	// GMM projection, see Eq. (5)
// 	for (int i = 0; i < model.parameters.nb_states; ++i) {
// 		mu.push_back(vector_list_t());
// 		inv_sigma.push_back(matrix_list_t());

// 		for (int m = 0; m < model.parameters.nb_frames; ++m) {
// 			mu[i].push_back(coordinate_systems[m].orientation * model.mu[i][m] +
// 							coordinate_systems[m].position);

// 			inv_sigma[i].push_back(coordinate_systems[m].orientation *
// 								   model.precomputed.inv_sigma[i][m] *
// 								   coordinate_systems[m].orientation.t());
// 		}
// 	}
// }

void computeGMR(model_t model, int nbGMRComponents, arma::cube &muGMR, arma::field<arma::cube> &sigmaGMR, const task_frame_list_t &transforms, std::vector<mat> &muPList, std::vector<mat> &sigmaPList, const mat &inputs)
{

	if (model.mu.size() > 0)
	{

		mat H(model.parameters.nb_states, inputs.size());

		//Init GMR mu and sigma
		// arma::cube muGMR(3*model.parameters.nb_deriv, inputs.size(), model.parameters.nb_frames);

		for (unsigned int i = 0; i < model.parameters.nb_frames; i++)
		{
			sigmaGMR(i).set_size(2 * model.parameters.nb_deriv, 2 * model.parameters.nb_deriv, inputs.size());
			sigmaGMR(i) = zeros(2 * model.parameters.nb_deriv, 2 * model.parameters.nb_deriv, inputs.size());
		}

		// Local trajectories distributions in each of the nb_frames task frames
		for (unsigned int m = 0; m < model.parameters.nb_frames; m++)
		{
			// Compute activation weight
			for (unsigned int i = 0; i < model.parameters.nb_states; i++)
			{
				H.row(i) = model.priors(i) * gaussPDF(inputs, model.mu[i][m].row(model.nb_var - 1), model.sigma[i][m].row(model.nb_var - 1).col(model.nb_var - 1)).t();
			}
			H = H / repmat(sum(H + 1e-300), model.parameters.nb_states, 1);

			mat muTmp(2 * model.parameters.nb_deriv, model.parameters.nb_states);

			mat sigmaTmp;

			for (int t = 0; t < inputs.size(); t++)
			{
				// Compute conditional means
				for (unsigned int i = 0; i < model.parameters.nb_states; i++)
				{

					muTmp.col(i) = model.mu[i][m].subvec(0, 2 * model.parameters.nb_deriv - 1) +
								   model.sigma[i][m].col(2 * model.parameters.nb_deriv).rows(0, 2 * model.parameters.nb_deriv - 1) * inv(model.sigma[i][m].row(model.nb_var - 1).col(model.nb_var - 1)) * (inputs(t) - model.mu[i][m].row(model.nb_var - 1));

					muGMR.slice(m).col(t) += H(i, t) * muTmp.col(i);
				}
				std::cout << "muGMR done!" << std::endl;

				// Compute conditional covariances
				for (int i = 0; i < model.parameters.nb_states; i++)
				{
					sigmaTmp = model.sigma[i][m].submat(0, 0, model.nb_var - 2, model.nb_var - 2) -
							   model.sigma[i][m].col(2 * model.parameters.nb_deriv).rows(0, 2 * model.parameters.nb_deriv - 1) *
								   inv(model.sigma[i][m].row(model.nb_var - 1).col(model.nb_var - 1)) *
								   model.sigma[i][m].row(2 * model.parameters.nb_deriv).cols(0, 2 * model.parameters.nb_deriv - 1);
					sigmaGMR(m).slice(t) += H(i, t) * (sigmaTmp + muTmp.col(i) * muTmp.col(i).t());
				}
				sigmaGMR(m).slice(t) += -muGMR.slice(m).col(t) * muGMR.slice(m).col(t).t() + eye(2 * model.parameters.nb_deriv, 2 * model.parameters.nb_deriv) * 1e-4;
				std::cout << "sigmaGMR done!" << std::endl;
			}
		}

		// transform mu/sigma GMR components into coordinate systems

		cube muGMRt = zeros(8, inputs.size(), model.parameters.nb_frames);
		field<cube> sigmaGMRt(model.parameters.nb_frames);

		for (int i = 0; i < model.parameters.nb_frames; i++)
		{
			sigmaGMRt(i).resize(8, 8, inputs.size());
			sigmaGMRt(i) = zeros(8, 8, inputs.size());
		}
		for (int f = 0; f < model.parameters.nb_frames; f++)
		{
			// Transform conditional means and covariances into coordinate system f
			muGMRt.slice(f) = muGMR.slice(f).rows(0, 7);
			vec b = {transforms[f].q(0), transforms[f].q(1), transforms[f].q(2), transforms[f].q(3), transforms[f].q(4), transforms[f].q(5), transforms[f].q(6), transforms[f].q(7)};
			std::cout << "b" << std::endl;
			b.print();
			muGMRt.slice(f).each_col() += b;
			for (unsigned int t = 0; t < inputs.size(); t++)
			{
				sigmaGMRt(f).slice(t) = sigmaGMR(f).slice(t).submat(0, 0, 7, 7);
			}
		}

		// // Product

		vec maxMuP = zeros(8, 1);
		mat maxSigmaP = eye(8, 8);

		std::cout << "inputs " << inputs.size() << std::endl;
		inputs.print();

		std::vector<double> tMean(nbGMRComponents), xMean(nbGMRComponents);

		for (unsigned int t = 0; t < inputs.size(); t++)
		{

			vec muP = zeros(8, 1);
			mat sigmaP = eye(8, 8);

			for (int m = 0; m < model.parameters.nb_frames; m++)
			{

				sigmaP += inv(sigmaGMRt(m).slice(t) + eye(8, 8) * 1e-4);
				muP += inv(sigmaGMRt(m).slice(t) + eye(8, 8) * 1e-4) * muGMRt.slice(m).col(t);
			}

			sigmaP = inv(sigmaP);
			muP = sigmaP * muP;

			std::cout << "t: " << t << std::endl;
			std::cout << "sigma" << std::endl;
			sigmaP.print();
			std::cout << "mu" << std::endl;
			muP.print();

			muPList.push_back(muP);
			sigmaPList.push_back(sigmaP);

			tMean.at(t) = inputs(t);
			xMean.at(t) = muP[0];
		}
	}
}

// void batchLQRReproduction(const model_t& model, const coordinate_system_list_t& coordinateSystems, int referenceDemonstrationIndex = 0){
// 	std::vector< vector_list_t > mu;		// Indexing: mu[state][frame]
// 	std::vector< matrix_list_t > inv_sigma; // Indexing: inv_sigma[state][frame]

// 	// GMM projection, see Eq. (5)
// 	for (int i = 0; i < model.parameters.nb_states; ++i) {
// 		mu.push_back(vector_list_t());
// 		inv_sigma.push_back(matrix_list_t());

// 		for (int m = 0; m < model.parameters.nb_frames; ++m) {
// 			mu[i].push_back(coordinate_systems[m].orientation * model.mu[i][m] +
// 							coordinate_systems[m].position);

// 			inv_sigma[i].push_back(coordinate_systems[m].orientation *
// 								   model.precomputed.inv_sigma[i][m] *
// 								   coordinate_systems[m].orientation.t());
// 		}
// 	}
// }

void plotDemos3D(demonstration_list_t demos, bool showGraphic)
{

	std::vector<std::vector<double>> xDemos;
	std::vector<std::vector<double>> yDemos;
	std::vector<std::vector<double>> zDemos;
	std::vector<std::vector<double>> tDemos;

	for (unsigned int nDemo = 0; nDemo < demos.size(); nDemo++)
	{
		std::vector<double> sampleX(demos[nDemo].points.n_cols);
		std::vector<double> sampleY;
		std::vector<double> sampleZ;
		std::vector<double> sampleT;

		for (unsigned int i = 0; i < demos[nDemo].points.n_cols; i++)
		{
			sampleX.at(i) = (demos[nDemo].points(0, i));
			sampleY.push_back(demos[nDemo].points(1, i));
			sampleZ.push_back(demos[nDemo].points(2, i));
			sampleT.push_back(demos[nDemo].points(6, i));
		}
		xDemos.push_back(sampleX);
		yDemos.push_back(sampleY);
		zDemos.push_back(sampleZ);
		tDemos.push_back(sampleT);
	}

	matplot::plot3(xDemos, yDemos, zDemos, "--");

	if (showGraphic)
	{
		matplot::show();
	}
	else
	{
		matplot::hold(matplot::on);
	}
}

void plotGMR3D(mat inputs, std::vector<mat> mu, std::vector<mat> sigma, int stepSigmaPlot, bool showGraphic, coordinate_system_list_t coordinateSystems)
{

	std::vector<double> tmean(inputs.size()), muX(inputs.size()), muY(inputs.size()), muZ(inputs.size());
	for (unsigned int i = 0; i < tmean.size(); i++)
	{
		tmean.at(i) = inputs[i];
		muX.at(i) = mu[i][0];
		muY.at(i) = mu[i][1];
		muZ.at(i) = mu[i][2];
	}

	matplot::plot3(muX, muY, muZ);

	for (int iTrajectory = 0; iTrajectory < sigma.size(); iTrajectory = iTrajectory + stepSigmaPlot)
	{
		vec eigval;
		mat eigvec;
		eig_sym(eigval, eigvec, sigma[iTrajectory]);

		float N = 1;
		vec radii = {N * sqrt(eigval[0]), N * sqrt(eigval[1]), N * sqrt(eigval[2])};

		vec u = linspace<vec>(0.0, 2.0 * 3.1415, 20);
		vec v = linspace<vec>(0.0, 3.1415, 20);
		mat x, y, z;

		vec sinu = sin(u);
		vec cosu = cos(u);
		vec sinv = sin(v);
		vec cosv = cos(v);
		for (int i = 0; i < u.n_rows; i++)
		{
			x = join_rows(x, radii[0] * cosu[i] * sinv);
			y = join_rows(y, radii[1] * sinu[i] * sinv);
			z = join_rows(z, radii[2] * 1 * cosv);
		}

		std::cout << eigvec << std::endl;
		eigvec.print();
		mat points;
		for (int i = 0; i < x.n_rows; i++)
		{
			for (int j = 0; j < x.n_cols; j++)
			{
				// std::cout<<"test prev"<<std::endl;
				vec test = {x.at(i, j), y.at(i, j), z.at(i, j)};
				vec rotatedTest = test;
				rotatedTest[0] = arma::dot(test, eigvec.col(0).t());
				rotatedTest[1] = arma::dot(test, eigvec.col(1).t());
				rotatedTest[2] = arma::dot(test, eigvec.col(2).t());

				points = join_rows(points, rotatedTest);
			}
		}
		std::cout << "points " << points.n_cols << " " << points.n_rows << std::endl;
		std::vector<double> pointsX, pointsY, pointsZ;
		for (int i = 0; i < points.n_cols; i++)
		{
			pointsX.push_back(points.at(0, i) + mu[iTrajectory][0]);
			pointsY.push_back(points.at(1, i) + mu[iTrajectory][1]);
			pointsZ.push_back(points.at(2, i) + mu[iTrajectory][2]);
		}
		std::cout << pointsX.size() << std::endl;
		matplot::scatter3(pointsX, pointsY, pointsZ);
	}
	if (showGraphic)
		matplot::show();
}

void plotTimelineGMR(demonstration_list_t demos, mat inputs, std::vector<mat> mu, std::vector<mat> sigma, bool showgraphic, coordinate_system_list_t coordinateSystems)
{

	std::vector<double> tmean(inputs.size()), muX(inputs.size()), muY(inputs.size()), muZ(inputs.size());
	std::vector<double> errX, errY, errZ;

	for (unsigned int i = 0; i < tmean.size(); i++)
	{
		tmean.at(i) = inputs[i];
		muX.at(i) = mu[i][0];
		muY.at(i) = mu[i][1];
		muZ.at(i) = mu[i][2];
		errX.push_back(sqrt(sigma[i].at(0, 0)));
		errY.push_back(sqrt(sigma[i].at(1, 1)));
		errZ.push_back(sqrt(sigma[i].at(2, 2)));
	}

	std::vector<double> startX, startT, startY, startZ;
	startX.push_back(coordinateSystems[0].position[0]);
	startY.push_back(coordinateSystems[0].position[1]);
	startZ.push_back(coordinateSystems[0].position[2]);
	startT.push_back(inputs[0]);

	startX.push_back(coordinateSystems[1].position[0]);
	startY.push_back(coordinateSystems[1].position[1]);
	startZ.push_back(coordinateSystems[1].position[2]);
	startT.push_back(inputs[inputs.size() - 1]);

	auto f = matplot::figure();
	f->width(f->width() * 3);
	f->height(f->height() * 2);
	matplot::gca()->title_enhanced(true);

	std::vector<std::vector<double>> xDemos;
	std::vector<std::vector<double>> yDemos;
	std::vector<std::vector<double>> zDemos;
	std::vector<std::vector<double>> tDemos;

	for (unsigned int nDemo = 0; nDemo < demos.size(); nDemo++)
	{
		std::vector<double> sampleX(demos[nDemo].points.n_cols);
		std::vector<double> sampleY;
		std::vector<double> sampleZ;
		std::vector<double> sampleT;

		for (unsigned int i = 0; i < demos[nDemo].points.n_cols; i++)
		{
			sampleX.at(i) = (demos[nDemo].points(0, i));
			sampleY.push_back(demos[nDemo].points(1, i));
			sampleZ.push_back(demos[nDemo].points(2, i));
			sampleT.push_back(demos[nDemo].points(6, i));
		}
		xDemos.push_back(sampleX);
		yDemos.push_back(sampleY);
		zDemos.push_back(sampleZ);
		tDemos.push_back(sampleT);
	}

	auto ax1 = matplot::subplot(1, 3, 0);
	matplot::errorbar(tmean, muX, errX)->filled_curve(true);
	matplot::hold(matplot::on);
	for (int nDemo = 0; nDemo < demos.size(); nDemo++)
		matplot::plot(tDemos[nDemo], xDemos[nDemo], "--");
	matplot::plot(startT, startX, "gx")->line_width(2).marker_size(10);
	matplot::title(ax1, "TPGMR x");
	matplot::xlabel(ax1, "t");
	matplot::ylabel(ax1, "x");

	auto ax2 = matplot::subplot(1, 3, 1);
	matplot::errorbar(tmean, muY, errY)->filled_curve(true);
	matplot::hold(matplot::on);
	for (int nDemo = 0; nDemo < demos.size(); nDemo++)
		matplot::plot(tDemos[nDemo], yDemos[nDemo], "--");

	matplot::plot(startT, startY, "gx")->line_width(2).marker_size(10);
	matplot::title(ax2, "TPGMR y");
	matplot::xlabel(ax2, "t");
	matplot::ylabel(ax2, "y");

	auto ax3 = matplot::subplot(1, 3, 2);
	matplot::errorbar(tmean, muZ, errZ)->filled_curve(true);
	matplot::hold(matplot::on);
	for (int nDemo = 0; nDemo < demos.size(); nDemo++)
		matplot::plot(tDemos[nDemo], zDemos[nDemo], "--");

	matplot::plot(startT, startZ, "gx")->line_width(2).marker_size(10);
	matplot::title(ax3, "TPGMR z");
	matplot::xlabel(ax3, "t");
	matplot::ylabel(ax3, "z");

	if (showgraphic)
	{
		matplot::show();
	}
}

void plotTimelineGMR(demonstration_joint_space_list_t demos, mat inputs, std::vector<mat> mu, std::vector<mat> sigma, bool showgraphic, task_frame_list_t task_frames)
{
	std::vector<double> muQ[8];
	std::vector<double> tmean(inputs.size());
	std::vector<double> errQ[8];
	for (unsigned int i = 0; i < tmean.size(); i++)
	{
		tmean.at(i) = inputs[i];
		for (unsigned int j = 0; j < 8; j++)
		{

			muQ[j].push_back(mu.at(i).at(j));

			errQ[j].push_back(sqrt(sigma[i].at(j, j)));
		}
	}
	// std::vector<double> startX, startT, startY, startZ;
	// startX.push_back(coordinateSystems[0].position[0]);
	// startY.push_back(coordinateSystems[0].position[1]);
	// startZ.push_back(coordinateSystems[0].position[2]);
	// startT.push_back(inputs[0]);

	// startX.push_back(coordinateSystems[1].position[0]);
	// startY.push_back(coordinateSystems[1].position[1]);
	// startZ.push_back(coordinateSystems[1].position[2]);
	// startT.push_back(inputs[inputs.size() - 1]);

	auto f = matplot::figure();
	f->width(f->width() * 2);
	f->height(f->height() * 2);
	matplot::gca()->title_enhanced(true);

	std::vector<std::vector<double>> q0Demos, q1Demos, q2Demos;
	// std::vector<std::vector<double>> yDemos;
	// std::vector<std::vector<double>> zDemos;
	std::vector<std::vector<double>> tDemos;

	for (unsigned int nDemo = 0; nDemo < demos.size(); nDemo++)
	{
		std::vector<double> sampleq0(demos[nDemo].points.n_cols);
		std::vector<double> sampleq1(demos[nDemo].points.n_cols);
		std::vector<double> sampleq2(demos[nDemo].points.n_cols);

		std::vector<double> sampleT;
		for (unsigned int i = 0; i < demos[nDemo].points.n_cols; i++)
		{
			sampleq0.at(i) = (demos[nDemo].points(0, i));
			sampleq1.at(i) = (demos[nDemo].points(1, i));
			sampleq2.at(i) = (demos[nDemo].points(2, i));

			sampleT.push_back(demos[nDemo].points(16, i));
		}
		q0Demos.push_back(sampleq0);
		q1Demos.push_back(sampleq1);
		q2Demos.push_back(sampleq2);

		tDemos.push_back(sampleT);
	}

	auto ax1 = matplot::subplot(2, 4, 0);
	matplot::errorbar(tmean, muQ[0], errQ[0])->filled_curve(true);
	matplot::hold(matplot::on);
	for (int nDemo = 0; nDemo < demos.size(); nDemo++)
		matplot::plot(tDemos[nDemo], q0Demos[nDemo], "--");
	// matplot::plot(startT, startX, "gx")->line_width(2).marker_size(10);
	// matplot::title(ax1, "TPGMR x");
	// matplot::xlabel(ax1, "t");
	// matplot::ylabel(ax1, "x");

	auto ax2 = matplot::subplot(2, 4, 1);
	matplot::errorbar(tmean, muQ[1], errQ[1])->filled_curve(true);
	matplot::hold(matplot::on);
	for (int nDemo = 0; nDemo < demos.size(); nDemo++)
		matplot::plot(tDemos[nDemo], q1Demos[nDemo], "--");

	ax2 = matplot::subplot(2, 4, 2);
	matplot::errorbar(tmean, muQ[2], errQ[2])->filled_curve(true);
	matplot::hold(matplot::on);
	for (int nDemo = 0; nDemo < demos.size(); nDemo++)
		matplot::plot(tDemos[nDemo], q2Demos[nDemo], "--");

	// matplot::plot(startT, startY, "gx")->line_width(2).marker_size(10);
	// matplot::title(ax2, "TPGMR y");
	// matplot::xlabel(ax2, "t");
	// matplot::ylabel(ax2, "y");

	// auto ax3 = matplot::subplot(1, 3, 2);
	// matplot::errorbar(tmean, muZ, errZ)->filled_curve(true);
	// matplot::hold(matplot::on);
	// for (int nDemo = 0; nDemo < demos.size(); nDemo++)
	// 	matplot::plot(tDemos[nDemo], zDemos[nDemo], "--");

	// matplot::plot(startT, startZ, "gx")->line_width(2).marker_size(10);
	// matplot::title(ax3, "TPGMR z");
	// matplot::xlabel(ax3, "t");
	// matplot::ylabel(ax3, "z");

	if (showgraphic)
	{
		matplot::show();
	}
}

void plotTimelineLocalTrajectoryGMR(demonstration_list_t demos, mat inputs, const arma::cube &muGMR, const arma::field<arma::cube> &sigmaGMR, bool showGraphic, int taskFrame)
{
	std::vector<double> tmean(inputs.size()), muX(inputs.size()), muY(inputs.size()), muZ(inputs.size());
	std::vector<double> errX, errY, errZ;

	for (unsigned int i = 0; i < tmean.size(); i++)
	{
		tmean.at(i) = inputs[i];
		std::cout << "MuGMR task frame" << std::endl;
		muGMR.slice(taskFrame).col(i).print();
		muX.at(i) = muGMR.slice(taskFrame).col(i)[0];
		muY.at(i) = muGMR.slice(taskFrame).at(1, i);
		muZ.at(i) = muGMR.slice(taskFrame).col(i)[2];
		errX.push_back(sqrt(sigmaGMR(taskFrame).slice(i).at(0, 0)));
		errY.push_back(sqrt(sigmaGMR(taskFrame).slice(i).at(1, 1)));
		errZ.push_back(sqrt(sigmaGMR(taskFrame).slice(i).at(2, 2)));
	}

	auto f = matplot::figure();
	f->width(f->width() * 3);
	f->height(f->height() * 2);
	matplot::gca()->title_enhanced(true);

	std::vector<std::vector<double>> xDemos;
	std::vector<std::vector<double>> yDemos;
	std::vector<std::vector<double>> zDemos;
	std::vector<std::vector<double>> tDemos;

	for (unsigned int nDemo = 0; nDemo < demos.size(); nDemo++)
	{
		std::vector<double> sampleX(demos[nDemo].points_in_coordinate_systems[taskFrame].n_cols);
		std::vector<double> sampleY(demos[nDemo].points_in_coordinate_systems[taskFrame].n_cols);
		std::vector<double> sampleZ(demos[nDemo].points_in_coordinate_systems[taskFrame].n_cols);
		std::vector<double> sampleT(demos[nDemo].points_in_coordinate_systems[taskFrame].n_cols);

		for (unsigned int i = 0; i < demos[nDemo].points.n_cols; i++)
		{
			sampleX.at(i) = (demos[nDemo].points_in_coordinate_systems[taskFrame](0, i));
			sampleY.at(i) = (demos[nDemo].points_in_coordinate_systems[taskFrame](1, i));
			sampleZ.at(i) = (demos[nDemo].points_in_coordinate_systems[taskFrame](2, i));
			sampleT.at(i) = (demos[nDemo].points_in_coordinate_systems[taskFrame](6, i));
		}
		xDemos.push_back(sampleX);
		yDemos.push_back(sampleY);
		zDemos.push_back(sampleZ);
		tDemos.push_back(sampleT);
	}

	std::string taskFrameTitle = "Task Frame " + std::to_string(taskFrame);
	auto ax1 = matplot::subplot(1, 3, 0);
	matplot::errorbar(tmean, muX, errX)->filled_curve(true);
	matplot::hold(matplot::on);
	for (int nDemo = 0; nDemo < demos.size(); nDemo++)
		matplot::plot(tDemos[nDemo], xDemos[nDemo], "--");
	matplot::title(ax1, taskFrameTitle);
	matplot::xlabel(ax1, "t");
	matplot::ylabel(ax1, "x");

	ax1 = matplot::subplot(1, 3, 1);
	matplot::errorbar(tmean, muY, errY)->filled_curve(true);
	matplot::hold(matplot::on);
	for (int nDemo = 0; nDemo < demos.size(); nDemo++)
		matplot::plot(tDemos[nDemo], yDemos[nDemo], "--");
	matplot::title(ax1, taskFrameTitle);
	matplot::xlabel(ax1, "t");
	matplot::ylabel(ax1, "y");

	ax1 = matplot::subplot(1, 3, 2);
	matplot::errorbar(tmean, muZ, errZ)->filled_curve(true);
	matplot::hold(matplot::on);
	for (int nDemo = 0; nDemo < demos.size(); nDemo++)
		matplot::plot(tDemos[nDemo], zDemos[nDemo], "--");
	matplot::title(ax1, taskFrameTitle);
	matplot::xlabel(ax1, "t");
	matplot::ylabel(ax1, "z");
	if (showGraphic)
	{
		matplot::show();
	}
}
void plotQLimits(const std::vector<double> &tmean, const std::array<double, 8> &qmin, const std::array<double, 8> &qmax, const unsigned int &jointNumber, double centerQ)
{
	std::vector<double> tLimits, qminLimits, qmaxLimits;
	tLimits.push_back(tmean[0]);
	tLimits.push_back(tmean[tmean.size() - 1]);


	qminLimits.push_back(qmin[jointNumber]-centerQ);
	qminLimits.push_back(qmin[jointNumber]-centerQ);

	qmaxLimits.push_back(qmax[jointNumber]-centerQ);
	qmaxLimits.push_back(qmax[jointNumber]-centerQ);

	matplot::plot(tLimits, qminLimits, "r")->line_width(2);
	matplot::plot(tLimits, qmaxLimits, "r")->line_width(2);
}

void plotTimelineLocalTrajectoryGMR(demonstration_joint_space_list_t demos, mat inputs, const arma::cube &muGMR, const arma::field<arma::cube> &sigmaGMR, bool showGraphic, int taskFrame, const std::array<double, 8> &qmin, const std::array<double, 8> &qmax)
{
	std::vector<double> tmean(inputs.size());
	std::vector<double> muQ[8];
	std::vector<double> errQ[8];

	for (unsigned int i = 0; i < tmean.size(); i++)
	{
		tmean.at(i) = inputs[i];
		for (unsigned int j = 0; j < 8; j++)
		{

			muQ[j].push_back(muGMR.slice(taskFrame).col(i)[j]);
			errQ[j].push_back(sqrt(sigmaGMR(taskFrame).slice(i).at(j, j)));
		}
	}

	auto f = matplot::figure();
	f->width(f->width() * 3);
	f->height(f->height() * 2);
	matplot::gca()->title_enhanced(true);

	std::vector<std::vector<double>> tDemos;
	std::vector<std::vector<double>> q0Demos, q1Demos, q2Demos, q3Demos, q4Demos, q5Demos, q6Demos, q7Demos;

	for (unsigned int nDemo = 0; nDemo < demos.size(); nDemo++)
	{
		std::vector<double> sampleq0(demos[nDemo].points_in_task_frames[taskFrame].n_cols);
		std::vector<double> sampleq1(demos[nDemo].points_in_task_frames[taskFrame].n_cols);
		std::vector<double> sampleq2(demos[nDemo].points_in_task_frames[taskFrame].n_cols);
		std::vector<double> sampleq3(demos[nDemo].points_in_task_frames[taskFrame].n_cols);
		std::vector<double> sampleq4(demos[nDemo].points_in_task_frames[taskFrame].n_cols);
		std::vector<double> sampleq5(demos[nDemo].points_in_task_frames[taskFrame].n_cols);
		std::vector<double> sampleq6(demos[nDemo].points_in_task_frames[taskFrame].n_cols);
		std::vector<double> sampleq7(demos[nDemo].points_in_task_frames[taskFrame].n_cols);

		std::vector<double> sampleT(demos[nDemo].points_in_task_frames[taskFrame].n_cols);

		for (unsigned int i = 0; i < demos[nDemo].points.n_cols; i++)
		{
			sampleq0.at(i) = (demos[nDemo].points_in_task_frames[taskFrame](0, i));
			sampleq1.at(i) = demos[nDemo].points_in_task_frames[taskFrame](1, i);
			sampleq2.at(i) = demos[nDemo].points_in_task_frames[taskFrame](2, i);
			sampleq3.at(i) = demos[nDemo].points_in_task_frames[taskFrame](3, i);
			sampleq4.at(i) = demos[nDemo].points_in_task_frames[taskFrame](4, i);
			sampleq5.at(i) = demos[nDemo].points_in_task_frames[taskFrame](5, i);
			sampleq6.at(i) = demos[nDemo].points_in_task_frames[taskFrame](6, i);
			sampleq7.at(i) = demos[nDemo].points_in_task_frames[taskFrame](7, i);

			sampleT.at(i) = (demos[nDemo].points_in_task_frames[taskFrame](16, i));
		}
		q0Demos.push_back(sampleq0);
		q1Demos.push_back(sampleq1);
		q2Demos.push_back(sampleq2);
		q3Demos.push_back(sampleq3);
		q4Demos.push_back(sampleq4);
		q5Demos.push_back(sampleq5);
		q6Demos.push_back(sampleq6);
		q7Demos.push_back(sampleq7);

		tDemos.push_back(sampleT);
	}

	std::string taskFrameTitle = "Task Frame " + std::to_string(taskFrame);
	matplot::title(taskFrameTitle);

	// Joint 0
	auto ax1 = matplot::subplot(2, 4, 0);
	matplot::errorbar(tmean, muQ[0], errQ[0])->filled_curve(true);
	matplot::hold(matplot::on);
	for (int nDemo = 0; nDemo < demos.size(); nDemo++)
		matplot::plot(tDemos[nDemo], q0Demos[nDemo], "--");

	plotQLimits(tmean, qmin, qmax, 0,  muGMR.slice(taskFrame).at(0,0));
	matplot::xlabel(ax1, "t");
	matplot::ylabel(ax1, "q(0)");

	// Joint 1
	ax1 = matplot::subplot(2, 4, 1);
	matplot::errorbar(tmean, muQ[1], errQ[1])->filled_curve(true);
	matplot::hold(matplot::on);
	for (int nDemo = 0; nDemo < demos.size(); nDemo++)
		matplot::plot(tDemos[nDemo], q1Demos[nDemo], "--");

	plotQLimits(tmean, qmin, qmax,1, muGMR.slice(taskFrame).at(0,1));
	matplot::xlabel(ax1, "t");
	matplot::ylabel(ax1, "q(1)");

	// Joint 2
	ax1 = matplot::subplot(2, 4, 2);
	matplot::errorbar(tmean, muQ[2], errQ[2])->filled_curve(true);
	matplot::hold(matplot::on);
	for (int nDemo = 0; nDemo < demos.size(); nDemo++)
		matplot::plot(tDemos[nDemo], q2Demos[nDemo], "--");
	plotQLimits(tmean, qmin, qmax, 2,  muGMR.slice(taskFrame).at(0,2));
	matplot::xlabel(ax1, "t");
	matplot::ylabel(ax1, "q(2)");

	// Joint 3
	ax1 = matplot::subplot(2, 4, 3);
	matplot::errorbar(tmean, muQ[3], errQ[3])->filled_curve(true);
	matplot::hold(matplot::on);
	for (int nDemo = 0; nDemo < demos.size(); nDemo++)
		matplot::plot(tDemos[nDemo], q3Demos[nDemo], "--");
	plotQLimits(tmean, qmin, qmax, 3,  muGMR.slice(taskFrame).at(0,3));
	matplot::xlabel(ax1, "t");
	matplot::ylabel(ax1, "q(3)");

	// Joint 4
	ax1 = matplot::subplot(2, 4, 4);
	matplot::errorbar(tmean, muQ[4], errQ[4])->filled_curve(true);
	matplot::hold(matplot::on);
	for (int nDemo = 0; nDemo < demos.size(); nDemo++)
		matplot::plot(tDemos[nDemo], q4Demos[nDemo], "--");
	plotQLimits(tmean, qmin, qmax, 4,  muGMR.slice(taskFrame).at(0,4));
	matplot::xlabel(ax1, "t");
	matplot::ylabel(ax1, "q(4)");

	// Joint 5
	ax1 = matplot::subplot(2, 4, 5);
	matplot::errorbar(tmean, muQ[5], errQ[5])->filled_curve(true);
	matplot::hold(matplot::on);
	for (int nDemo = 0; nDemo < demos.size(); nDemo++)
		matplot::plot(tDemos[nDemo], q5Demos[nDemo], "--");
	plotQLimits(tmean, qmin, qmax, 5,  muGMR.slice(taskFrame).at(0,5));
	matplot::xlabel(ax1, "t");
	matplot::ylabel(ax1, "q(5)");

	// Joint 6
	ax1 = matplot::subplot(2, 4, 6);
	matplot::errorbar(tmean, muQ[6], errQ[6])->filled_curve(true);
	matplot::hold(matplot::on);
	for (int nDemo = 0; nDemo < demos.size(); nDemo++)
		matplot::plot(tDemos[nDemo], q6Demos[nDemo], "--");
	plotQLimits(tmean, qmin, qmax, 6,  muGMR.slice(taskFrame).at(0,6));
	matplot::xlabel(ax1, "t");
	matplot::ylabel(ax1, "q(6)");

	// Joint 7
	ax1 = matplot::subplot(2, 4, 7);
	matplot::errorbar(tmean, muQ[7], errQ[7])->filled_curve(true);
	matplot::hold(matplot::on);
	for (int nDemo = 0; nDemo < demos.size(); nDemo++)
		matplot::plot(tDemos[nDemo], q7Demos[nDemo], "--");
	plotQLimits(tmean, qmin, qmax, 7,  muGMR.slice(taskFrame).at(0,7));
	matplot::xlabel(ax1, "t");
	matplot::ylabel(ax1, "q(7)");

	if (showGraphic)
	{
		matplot::show();
	}
}

/******************************* MAIN FUNCTION *******************************/

int main(int argc, char **argv)
{

	arma_rng::set_seed_random();

	// Model
	model_t model;

	// Parameters
	model.parameters.nb_states = 2;
	model.parameters.nb_frames = 2;
	model.parameters.nb_deriv = 8;
	model.parameters.nb_data = 150;
	model.parameters.dt = 1.0f;

	// List of demonstrations and reproductions
	// demonstration_list_t demos;
	// coordinate_system_list_t coordinateSystems;

	demonstration_joint_space_list_t demos;
	task_frame_list_t taskFrames;

	//for(int indexSave = 1; indexSave<= 10; indexSave++){
	for (int nDemo = 1; nDemo <= 5; nDemo++)
	{
		vector_list_t trajectory;
		taskFrames.clear();
		std::string csvFile = "/home/elisabeth/repos/teo-sharon/programs/GenerateManipulationTrajectories/trajectories/test/test-right-arm-motion-smooth" + std::to_string(nDemo) + "-joint.csv";

		std::vector<std::array<double, 9>> desiredTrajectoryData = getTrajectoryFromCsvFile(csvFile);
		// printTrajectoryData(desiredTrajectoryData);

		// Lets try with only position

		for (unsigned int i = 0; i < desiredTrajectoryData.size(); i++)
		{
			vec q = {desiredTrajectoryData[i][1], desiredTrajectoryData[i][2], desiredTrajectoryData[i][3], desiredTrajectoryData[i][4], desiredTrajectoryData[i][5], desiredTrajectoryData[i][6], desiredTrajectoryData[i][7], desiredTrajectoryData[i][8]};
			trajectory.push_back(q);
		}

		// How to create the task frames?
		// Need position vec and orientation as a matrix.

		//Initial pose frame
		// KDL::Rotation rotKdl = KDL::Rotation::Quaternion(desiredTrajectoryData[0][4], desiredTrajectoryData[0][5], desiredTrajectoryData[0][6], desiredTrajectoryData[0][7]);
		// arma::mat initialOrientation = {{rotKdl.data[0], rotKdl.data[1], rotKdl.data[2]},
		// 								{rotKdl.data[3], rotKdl.data[4], rotKdl.data[5]},
		// 								{rotKdl.data[6], rotKdl.data[7], rotKdl.data[8]}};

		arma::vec qInit = {desiredTrajectoryData[0][1], desiredTrajectoryData[0][2], desiredTrajectoryData[0][3], desiredTrajectoryData[0][4], desiredTrajectoryData[0][5], desiredTrajectoryData[0][6], desiredTrajectoryData[0][7], desiredTrajectoryData[0][8]};

		// coordinate_system_t initialFrame(initialPosition, initialOrientation, model.parameters);
		task_frame_t initialFrame(qInit, model.parameters);

		arma::vec qFinal = {desiredTrajectoryData[desiredTrajectoryData.size() - 1][1], desiredTrajectoryData[desiredTrajectoryData.size() - 1][2], desiredTrajectoryData[desiredTrajectoryData.size() - 1][3], desiredTrajectoryData[desiredTrajectoryData.size() - 1][4], desiredTrajectoryData[desiredTrajectoryData.size() - 1][5], desiredTrajectoryData[desiredTrajectoryData.size() - 1][6], desiredTrajectoryData[desiredTrajectoryData.size() - 1][7], desiredTrajectoryData[desiredTrajectoryData.size() - 1][8]};

		task_frame_t finalFrame(qFinal, model.parameters);

		taskFrames.push_back(initialFrame);
		taskFrames.push_back(finalFrame);

		std::cout << "Lets add the demonstration " << nDemo << std::endl;

		DemonstrationJointsSpace demonstration(taskFrames, trajectory, model.parameters);

		demos.push_back(demonstration);

		// std::cout << "Points wrt initial frame" << std::endl;
		// demonstration.points_in_task_frames[0].print();

		// std::cout << "Points wrt final frame" << std::endl;
		// demonstration.points_in_task_frames[1].print();
	}

	std::cout << "Number of demos: " << demos.size() << std::endl;
	learn(demos, model);

	std::cout << "Demos learned" << std::endl;

	int nbGMRComponents = 20;
	mat inputs = linspace(0, model.parameters.nb_data, nbGMRComponents);
	inputs = inputs.t();
	arma::cube muGMR(2 * model.parameters.nb_deriv, inputs.size(), model.parameters.nb_frames);
	arma::field<cube> sigmaGMR(model.parameters.nb_frames);

	// // Check with the initial and final

	task_frame_list_t transforms;

	transforms.push_back(taskFrames[0]);
	transforms.push_back(taskFrames[1]);

	std::vector<mat> muPList;
	std::vector<mat> sigmaPList;

	computeGMR(model, nbGMRComponents, muGMR, sigmaGMR, transforms, muPList, sigmaPList, inputs);
	std::cout << "GMR done!" << std::endl;


	std::array<double, 8> qmin, qmax;
	// Min joints limits
	qmin[0] = -59.3;
	qmin[1] = -15.4;
	qmin[2] = -98.1;
	qmin[3] = -75.5;
	qmin[4] = -80.1;
	qmin[5] = -99.6;
	qmin[6] = -80.4;
	qmin[7] = -115.1;

	// Max joint limits
	qmax[0] = 46.3;
	qmax[1] = 10.1;
	qmax[2] = 106.0;
	qmax[3] = 22.4;
	qmax[4] = 57.0;
	qmax[5] = 98.4;
	qmax[6] = 99.6;
	qmax[7] = 44.7;

	plotTimelineLocalTrajectoryGMR(demos, inputs, muGMR, sigmaGMR, true, 0, qmin, qmax);

	// 	//Final pose frame
	// 	rotKdl = KDL::Rotation::Quaternion(desiredTrajectoryData[desiredTrajectoryData.size() - 1][4], desiredTrajectoryData[desiredTrajectoryData.size() - 1][5], desiredTrajectoryData[desiredTrajectoryData.size() - 1][6], desiredTrajectoryData[desiredTrajectoryData.size() - 1][7]);
	// 	arma::mat finalOrientation = {{rotKdl.data[0], rotKdl.data[1], rotKdl.data[2]},
	// 								  {rotKdl.data[3], rotKdl.data[4], rotKdl.data[5]},
	// 								  {rotKdl.data[6], rotKdl.data[7], rotKdl.data[8]}};
	// 	arma::vec finalPosition = {desiredTrajectoryData[desiredTrajectoryData.size() - 1][1], desiredTrajectoryData[desiredTrajectoryData.size() - 1][2], desiredTrajectoryData[desiredTrajectoryData.size() - 1][3]};

	// 	coordinate_system_t finalFrame(finalPosition, finalOrientation, model.parameters);

	// 	coordinateSystems.push_back(initialFrame);
	// 	coordinateSystems.push_back(finalFrame);

	// 	//Create demonstration

	// 	Demonstration demonstration(coordinateSystems, trajectory, model.parameters);
	// 	demos.push_back(demonstration);

	// 	//Check the transformation in the task frames.
	// 	std::cout << "Points in initial pose task frame" << std::endl;
	// 	demonstration.points_in_coordinate_systems[0].print();

	// 	std::cout << "Points in final pose task frame" << std::endl;
	// 	demonstration.points_in_coordinate_systems[1].print();
	// }

	// std::cout << "Number of demos: " << demos.size() << std::endl;
	// learn(demos, model);

	// std::cout << "Demos learned" << std::endl;

	// int nbGMRComponents = 20;
	// mat inputs = linspace(0, model.parameters.nb_data, nbGMRComponents);
	// inputs = inputs.t();
	// arma::cube muGMR(2 * model.parameters.nb_deriv, inputs.size(), model.parameters.nb_frames);
	// arma::field<cube> sigmaGMR(model.parameters.nb_frames);

	// // Check with the initial and final

	// std::vector<gfx2::transforms_t> transforms;

	// for (unsigned int nFrames = 0; nFrames < coordinateSystems.size(); nFrames++)
	// {
	// 	gfx2::transforms_t testTransform;
	// 	testTransform.position(0) = demos[0].coordinate_systems[nFrames].position[0];
	// 	testTransform.position(1) = demos[0].coordinate_systems[nFrames].position[1];
	// 	testTransform.position(2) = demos[0].coordinate_systems[nFrames].position[2];

	// 	for (unsigned int i = 0; i < 3; i++)
	// 	{
	// 		for (unsigned int j = 0; j < 3; j++)
	// 		{
	// 			testTransform.rotation(i, j) = demos[0].coordinate_systems[nFrames].orientation(i, j);
	// 		}
	// 	}

	// 	std::cout << "Frame " << nFrames << " ---------------------------------" << std::endl;
	// 	std::cout << "Position: " << std::endl;
	// 	testTransform.position.print();
	// 	std::cout << "Rotation: " << std::endl;
	// 	testTransform.rotation.print();
	// 	transforms.push_back(testTransform);
	// }

	// std::vector<mat> muPList;
	// std::vector<mat> sigmaPList;
	// computeGMR(model, nbGMRComponents, muGMR, sigmaGMR, transforms, muPList, sigmaPList, inputs);

	// // Lets plot gmr and the demonstrations

	// plotDemos3D(demos, false);

	// std::cout << "plotgmr3d" << std::endl;
	// plotGMR3D(inputs, muPList, sigmaPList, 2, false, demos[0].coordinate_systems);

	// plotTimelineGMR(demos, inputs, muPList, sigmaPList, false, demos[0].coordinate_systems);

	// plotTimelineLocalTrajectoryGMR(demos, inputs, muGMR, sigmaGMR, true, 0);

	//std::string fileName =  std::to_string(indexSave)+"Demo_GMM25States_GMR20Components.pdf";
	//matplot::save("/home/elisabeth/repos/teo-sharon/programs/GenerateManipulationTrajectories/results/GMM25States_GMR20Components/"+fileName);
	//}
	// std::cout<<muPList.size()<<std::endl;

	return 0;
}
