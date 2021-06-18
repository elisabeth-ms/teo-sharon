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

void computeGMR(model_t model, int nbGMRComponents, arma::cube &muGMR, arma::field<arma::cube> &sigmaGMR, const std::vector<gfx2::transforms_t> &transforms, std::vector<mat> &muPList, std::vector<mat> &sigmaPList)
{

	if (model.mu.size() > 0)
	{
		mat inputs = linspace(0, 100, nbGMRComponents);
		inputs = inputs.t();

		mat H(model.parameters.nb_states, inputs.size());

		//Init GMR mu and sigma
		// arma::cube muGMR(3*model.parameters.nb_deriv, inputs.size(), model.parameters.nb_frames);

		for (unsigned int i = 0; i < model.parameters.nb_frames; i++)
		{
			sigmaGMR(i).set_size(2 * model.parameters.nb_deriv, 2 * model.parameters.nb_deriv, inputs.size());
			sigmaGMR(i) = zeros(2 * model.parameters.nb_deriv, 2 * model.parameters.nb_deriv, inputs.size());
		}

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

		std::cout << "inputs " << inputs.size() << std::endl;
		inputs.print();

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

void plotDemos3D(demonstration_list_t demos, bool showGraphic){

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

	if (showGraphic){
		matplot::show();
	}
	else{
		matplot::hold(matplot::on);
	}
}

void plotGMR3D(mat inputs, std::vector<mat> mu, std::vector<mat> sigma, int stepSigmaPlot, bool showGraphic, coordinate_system_list_t coordinateSystems){

std::vector<double> tmean(inputs.size()), muX(inputs.size()), muY(inputs.size()), muZ(inputs.size());
	for (unsigned int i = 0; i < tmean.size(); i++)
	{
		tmean.at(i) = inputs[i];
		muX.at(i) = mu[i][0];
		muY.at(i) = mu[i][1];
		muZ.at(i) = mu[i][2];
	}

	matplot::plot3(muX, muY, muZ);

	for (int iTrajectory = 0; iTrajectory < sigma.size(); iTrajectory=iTrajectory+stepSigmaPlot)
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

		std::cout<<eigvec<<std::endl;
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
	if(showGraphic)
		matplot::show();
}

void plotTimelineGMR(demonstration_list_t demos, mat inputs, std::vector<mat> mu, std::vector<mat> sigma, bool showgraphic, coordinate_system_list_t coordinateSystems){

	std::vector<double> tmean(inputs.size()), muX(inputs.size()), muY(inputs.size()), muZ(inputs.size());
	std::vector<double> errX, errY, errZ;

	for (unsigned int i = 0; i < tmean.size(); i++)
	{
		tmean.at(i) = inputs[i];
		muX.at(i) = mu[i][0];
		muY.at(i) = mu[i][1];
		muZ.at(i) = mu[i][2];
		errX.push_back(sqrt(sigma[i].at(0,0)));
		errY.push_back(sqrt(sigma[i].at(1,1)));
		errZ.push_back(sqrt(sigma[i].at(2,2)));
	}

	

	auto f= matplot::figure(true);

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

	

	auto ax1 = matplot::subplot(1,3,0);
	matplot::errorbar(tmean, muX, errX)->filled_curve(true);
	matplot::hold(matplot::on);
	for(int nDemo=0; nDemo<demos.size(); nDemo++)
		matplot::plot(tDemos[nDemo], xDemos[nDemo], "--");

	matplot::title(ax1, "TPGMR x");
	matplot::xlabel(ax1,"t");
	matplot::ylabel(ax1, "x");

	auto ax2 = matplot::subplot(1,3,1);
	matplot::errorbar(tmean, muY, errY)->filled_curve(true);
	matplot::hold(matplot::on);
	for(int nDemo=0; nDemo<demos.size(); nDemo++)
		matplot::plot(tDemos[nDemo], yDemos[nDemo], "--");
	matplot::title(ax2, "TPGMR y");
	matplot::xlabel(ax2,"t");
	matplot::ylabel(ax2, "y");


	auto ax3 = matplot::subplot(1,3,2);
	matplot::errorbar(tmean, muZ, errZ)->filled_curve(true);
	matplot::hold(matplot::on);
	for(int nDemo=0; nDemo<demos.size(); nDemo++)
		matplot::plot(tDemos[nDemo], zDemos[nDemo], "--");
	matplot::title(ax3, "TPGMR z");
	matplot::xlabel(ax3,"t");
	matplot::ylabel(ax3, "z");
	

	if(showgraphic){
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
	model.parameters.nb_states = 15;
	model.parameters.nb_frames = 2;
	model.parameters.nb_deriv = 3;
	model.parameters.nb_data = 100;
	model.parameters.dt = 0.1f;

	// List of demonstrations and reproductions
	demonstration_list_t demos;
	coordinate_system_list_t coordinateSystems;

	for (int nDemo = 1; nDemo <= 10; nDemo++)
	{
		vector_list_t trajectory;
		coordinateSystems.clear();
		std::string csvFile = "/home/elisabeth/repos/teo-sharon/programs/GenerateManipulationTrajectories/trajectories/test/test-right-arm-motion-smooth" + std::to_string(nDemo) + "-reaching.csv";

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

	std::cout << "Number of demos: " << demos.size() << std::endl;
	learn(demos, model);

	std::cout << "Demos learned" << std::endl;

	// arma::vec mu = zeros(3, 1);
	// arma::mat cov = zeros(3, 3);

	// for (unsigned int f = 0; f < model.parameters.nb_frames; f++)
	// {
	// 	vec mutmp = coordinateSystems[f].orientation * model.mu[0][f] + coordinateSystems[f].position;
	// 	arma::mat covtmp = coordinateSystems[f].orientation * model.sigma[0][f] * coordinateSystems[f].orientation.t();
	// 	arma::mat icovtmp = arma::inv(covtmp);
	// 	cov = cov + icovtmp;
	// 	mu = mu + icovtmp * mutmp;
	// }

	// cov = arma::inv(cov);
	// mu = cov * mu;
	// std::cout << "Sigma:" << std::endl;
	// cov.print();

	// std::cout << "mu:" << std::endl;
	// mu.print();

	int nbGMRComponents = 100;
	mat inputs = linspace(0, 100, nbGMRComponents);
	inputs = inputs.t();
	arma::cube muGMR(2 * model.parameters.nb_deriv, inputs.size(), model.parameters.nb_frames);
	arma::field<cube> sigmaGMR(model.parameters.nb_frames);

	// Check with the initial and final

	std::vector<gfx2::transforms_t> transforms;

	for (unsigned int nFrames = 0; nFrames < coordinateSystems.size(); nFrames++)
	{
		gfx2::transforms_t testTransform;
		testTransform.position(0) = coordinateSystems[nFrames].position[0];
		testTransform.position(1) = coordinateSystems[nFrames].position[1];
		testTransform.position(2) = coordinateSystems[nFrames].position[2];

		for (unsigned int i = 0; i < 3; i++)
		{
			for (unsigned int j = 0; j < 3; j++)
			{
				testTransform.rotation(i, j) = coordinateSystems[nFrames].orientation(i, j);
			}
		}

		std::cout << "Frame " << nFrames << " ---------------------------------" << std::endl;
		std::cout << "Position: " << std::endl;
		testTransform.position.print();
		std::cout << "Rotation: " << std::endl;
		testTransform.rotation.print();
		transforms.push_back(testTransform);
	}

	std::vector<mat> muPList;
	std::vector<mat> sigmaPList;
	computeGMR(model, nbGMRComponents, muGMR, sigmaGMR, transforms, muPList, sigmaPList);

	// Lets plot gmr and the demonstrations

	plotDemos3D(demos, false);

	std::cout<<"plotgmr3d"<<std::endl;
	plotGMR3D(inputs, muPList, sigmaPList,15, true, coordinateSystems);
	
	plotTimelineGMR(demos,inputs, muPList, sigmaPList, true,coordinateSystems);



	// std::cout<<muPList.size()<<std::endl;


	return 0;
}
