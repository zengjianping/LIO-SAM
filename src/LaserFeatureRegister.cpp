
#include "LaserFeatureRegister.hpp"

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <opencv2/opencv.hpp>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Defines LaserFeatureRegistration

LaserFeatureRegistration::LaserFeatureRegistration(const Options& options)
    : options_(options)
{
}

LaserFeatureRegistration::~LaserFeatureRegistration()
{
}

void LaserFeatureRegistration::setEdgeFeatureCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudCurr, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudLast)
{
    edgeCloudCurr_ = cloudCurr;
    edgeCloudLast_ = cloudLast;
    kdtreeEdgeCloud_->setInputCloud(edgeCloudLast_);
}

void LaserFeatureRegistration::setSurfFeatureCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudCurr, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudLast)
{
    surfCloudCurr_ = cloudCurr;
    surfCloudLast_ = cloudLast;
    kdtreeSurfCloud_->setInputCloud(surfCloudLast_);
}

bool LaserFeatureRegistration::process(const Eigen::Isometry3d& initPose, Eigen::Isometry3d& finalPose)
{
    bool converged = false;

    setInitPose(initPose);

    for (int iterCount=0; iterCount < options_.maxIterCount; iterCount++) {
        prepareProcessing();

        processEdgeFeatureCloud();

        processSurfFeatureCloud();

        if (solveOptimProblem(iterCount, converged)) {
            if (converged)
                break;
        } else {
            return false;
        }
    }

    return getFinalPose(finalPose);
}

void LaserFeatureRegistration::processEdgeFeatureCloud()
{
    int numFeatures = edgeCloudCurr_->points.size();
    int numValids = 0;

    for (int i = 0; i < numFeatures; i++) {
        Eigen::Vector3d currPoint, lastPointA, lastPointB;
        pcl::PointXYZI pointOri = edgeCloudCurr_->points[i], pointSel;
        bool res = matchOneEdgeFeature(pointOri, pointSel, currPoint, lastPointA, lastPointB);
        if (res) {
            addOneEdgeFeature(pointOri, pointSel, currPoint, lastPointA, lastPointB);
            numValids++;
        }
    }
}

bool LaserFeatureRegistration::matchOneEdgeFeature(const pcl::PointXYZI& pointOri, pcl::PointXYZI& pointSel, Eigen::Vector3d& currPoint,
        Eigen::Vector3d& lastPointA, Eigen::Vector3d& lastPointB)
{
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    pointAssociateToMap(pointOri, pointSel);
    kdtreeEdgeCloud_->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

    if (pointSearchSqDis[4] < 1.0) {
        std::vector<Eigen::Vector3d> nearEdges;
        Eigen::Vector3d centPoint(0, 0, 0);
        for (int j = 0; j < 5; j++) {
            const pcl::PointXYZI& point = edgeCloudLast_->points[pointSearchInd[j]];
            Eigen::Vector3d nearEdge(point.x, point.y, point.z);
            centPoint = centPoint + nearEdge;
            nearEdges.push_back(nearEdge);
        }
        centPoint = centPoint / 5.0;

        Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
        for (int j = 0; j < 5; j++) {
            Eigen::Matrix<double, 3, 1> zeroMean = nearEdges[j] - centPoint;
            covMat = covMat + zeroMean * zeroMean.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

        // if is indeed line feature, note Eigen library sort eigenvalues in increasing order
        if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
            Eigen::Vector3d unitDirection = saes.eigenvectors().col(2);
            currPoint = Eigen::Vector3d(pointOri.x, pointOri.y, pointOri.z);
            lastPointA = 0.1 * unitDirection + centPoint;
            lastPointB = -0.1 * unitDirection + centPoint;
            return true;
        }
    }

    return false;
}

void LaserFeatureRegistration::processSurfFeatureCloud()
{
    int numFeatures = surfCloudCurr_->points.size();
    int numValids = 0;

    for (int i = 0; i < numFeatures; i++) {
        Eigen::Vector3d currPoint, planeNorm;
        double planeIntercept = 0;
        pcl::PointXYZI pointOri = surfCloudCurr_->points[i], pointSel;
        bool res = matchOneSurfFeature(pointOri, pointSel, currPoint, planeNorm, planeIntercept);
        if (res) {
            addOneSurfFeature(pointOri, pointSel, currPoint, planeNorm, planeIntercept);
            numValids++;
        }
    }
}

bool LaserFeatureRegistration::matchOneSurfFeature(const pcl::PointXYZI& pointOri, pcl::PointXYZI& pointSel, Eigen::Vector3d& currPoint,
        Eigen::Vector3d& planeNorm, double& planeIntercept)
{
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    pointAssociateToMap(pointOri, pointSel);
    kdtreeSurfCloud_->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

    Eigen::Matrix<double, 5, 3> matA0;
    Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();

    if (pointSearchSqDis[4] < 1.0) {
        for (int j = 0; j < 5; j++) {
            const pcl::PointXYZI& point = surfCloudLast_->points[pointSearchInd[j]];
            matA0(j, 0) = point.x;
            matA0(j, 1) = point.y;
            matA0(j, 2) = point.z;
            //printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j, 2));
        }

        // find the norm of plane
        planeNorm = matA0.colPivHouseholderQr().solve(matB0);
        planeIntercept = 1 / planeNorm.norm();
        planeNorm.normalize();

        // Here n(pa, pb, pc) is unit norm of plane
        bool planeValid = true;
        for (int j = 0; j < 5; j++) {
            const pcl::PointXYZI& point = surfCloudLast_->points[pointSearchInd[j]];
            // if OX * n > 0.2, then plane is not fit well
            double distance = fabs(planeNorm(0) * point.x + planeNorm(1) * point.y + planeNorm(2) * point.z + planeIntercept);
            if (distance > 0.2) {
                planeValid = false;
                break;
            }
        }

        if (planeValid)
        {
            currPoint = Eigen::Vector3d(pointOri.x, pointOri.y, pointOri.z);
            return true;
        }
    }

    return false;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Declare LaserFeatureRegistrationCeres

class LaserFeatureRegistrationCeres : public LaserFeatureRegistration
{
public:
    LaserFeatureRegistrationCeres(const Options& options);
    virtual ~LaserFeatureRegistrationCeres();

protected:
    virtual void setInitPose(const Eigen::Isometry3d& initPose);
    virtual bool getFinalPose(Eigen::Isometry3d& finalPose);
    virtual void pointAssociateToMap(const pcl::PointXYZI& inp, pcl::PointXYZI& outp);
    virtual bool prepareProcessing();
    virtual void addOneEdgeFeature(const pcl::PointXYZI& pointOri, const pcl::PointXYZI& pointSel, const Eigen::Vector3d& currPoint,
        const Eigen::Vector3d& lastPointA, const Eigen::Vector3d& lastPointB);
    virtual void addOneSurfFeature(const pcl::PointXYZI& pointOri, const pcl::PointXYZI& pointSel, const Eigen::Vector3d& currPoint,
        const Eigen::Vector3d& planeNorm, double planeIntercept);
    virtual bool solveOptimProblem(int iterCount, bool& converged);

protected:
    double parameters_[7] = {0, 0, 0, 1, 0, 0, 0};
    Eigen::Map<Eigen::Quaterniond> quaterCurr2Last_ = Eigen::Map<Eigen::Quaterniond>(parameters_);
    Eigen::Map<Eigen::Vector3d> transCurr2Last_ = Eigen::Map<Eigen::Vector3d>(parameters_ + 4);
    boost::shared_ptr<ceres::Problem> problem_;
    boost::shared_ptr<ceres::LossFunction> lossFunction_;
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Define LaserFeatureRegistrationCeres

struct LaserEdgeFeatureFunctor
{
	LaserEdgeFeatureFunctor(Eigen::Vector3d _currPoint, Eigen::Vector3d _lastPointA, Eigen::Vector3d _lastPointB, double _s)
		: currPoint(_currPoint), lastPointA(_lastPointA), lastPointB(_lastPointB), s(_s)
    {
    }

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		Eigen::Matrix<T, 3, 1> cp{T(currPoint.x()), T(currPoint.y()), T(currPoint.z())};
		Eigen::Matrix<T, 3, 1> lpa{T(lastPointA.x()), T(lastPointA.y()), T(lastPointA.z())};
		Eigen::Matrix<T, 3, 1> lpb{T(lastPointB.x()), T(lastPointB.y()), T(lastPointB.z())};

		//Eigen::Quaternion<T> quaterCurr2Last{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
		Eigen::Quaternion<T> quaterCurr2Last{q[3], q[0], q[1], q[2]};
		Eigen::Quaternion<T> quaterIdentity{T(1), T(0), T(0), T(0)};
		quaterCurr2Last = quaterIdentity.slerp(T(s), quaterCurr2Last);
		Eigen::Matrix<T, 3, 1> transCurr2Last{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

		Eigen::Matrix<T, 3, 1> lp = quaterCurr2Last * cp + transCurr2Last;
		Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
		Eigen::Matrix<T, 3, 1> de = lpa - lpb;

		residual[0] = nu.x() / de.norm();
		residual[1] = nu.y() / de.norm();
		residual[2] = nu.z() / de.norm();

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d _currPoint, const Eigen::Vector3d _lastPointA,
									   const Eigen::Vector3d _lastPointB, const double _s)
	{
		return (new ceres::AutoDiffCostFunction<LaserEdgeFeatureFunctor, 3, 4, 3>(
			new LaserEdgeFeatureFunctor(_currPoint, _lastPointA, _lastPointB, _s)));
	}

	Eigen::Vector3d currPoint, lastPointA, lastPointB;
	double s;
};

struct LaserSurfFeatureFunctor
{
	LaserSurfFeatureFunctor(Eigen::Vector3d _currPoint, Eigen::Vector3d _lastPointJ, Eigen::Vector3d _lastPointL, Eigen::Vector3d _lastPointM, double _s)
		: currPoint(_currPoint), lastPointJ(_lastPointJ), lastPointL(_lastPointL), lastPointM(_lastPointM), s(_s)
	{
		ljmNorm = (lastPointJ - lastPointL).cross(lastPointJ - lastPointM);
		ljmNorm.normalize();
	}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		Eigen::Matrix<T, 3, 1> cp{T(currPoint.x()), T(currPoint.y()), T(currPoint.z())};
		Eigen::Matrix<T, 3, 1> lpj{T(lastPointJ.x()), T(lastPointJ.y()), T(lastPointJ.z())};
		//Eigen::Matrix<T, 3, 1> lpl{T(lastPointL.x()), T(lastPointL.y()), T(lastPointL.z())};
		//Eigen::Matrix<T, 3, 1> lpm{T(lastPointM.x()), T(lastPointM.y()), T(lastPointM.z())};
		Eigen::Matrix<T, 3, 1> ljm{T(ljmNorm.x()), T(ljmNorm.y()), T(ljmNorm.z())};

		//Eigen::Quaternion<T> quaterCurr2Last{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
		Eigen::Quaternion<T> quaterCurr2Last{q[3], q[0], q[1], q[2]};
		Eigen::Quaternion<T> quaterIdentity{T(1), T(0), T(0), T(0)};
		quaterCurr2Last = quaterIdentity.slerp(T(s), quaterCurr2Last);
		Eigen::Matrix<T, 3, 1> transCurr2Last{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

		Eigen::Matrix<T, 3, 1> lp = quaterCurr2Last * cp + transCurr2Last;
		residual[0] = (lp - lpj).dot(ljm);

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d _currPoint, const Eigen::Vector3d _lastPointJ,
			const Eigen::Vector3d _lastPointL, const Eigen::Vector3d _lastPointM, const double _s)
	{
		return (new ceres::AutoDiffCostFunction<LaserSurfFeatureFunctor, 1, 4, 3>(
			new LaserSurfFeatureFunctor(_currPoint, _lastPointJ, _lastPointL, _lastPointM, _s)));
	}

	Eigen::Vector3d currPoint, lastPointJ, lastPointL, lastPointM;
	Eigen::Vector3d ljmNorm;
	double s;
};

struct LaserSurfNormFeatureFunctor
{
	LaserSurfNormFeatureFunctor(Eigen::Vector3d _currPoint, Eigen::Vector3d _planeNorm, double _planeIntercept)
            : currPoint(_currPoint), planeNorm(_planeNorm), planeIntercept(_planeIntercept)
    {
    }

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		Eigen::Quaternion<T> quaterCurr2Last{q[3], q[0], q[1], q[2]};
		Eigen::Matrix<T, 3, 1> transCurr2Last{t[0], t[1], t[2]};
		Eigen::Matrix<T, 3, 1> cp{T(currPoint.x()), T(currPoint.y()), T(currPoint.z())};
		Eigen::Matrix<T, 3, 1> lp = quaterCurr2Last * cp + transCurr2Last;
		Eigen::Matrix<T, 3, 1> norm(T(planeNorm.x()), T(planeNorm.y()), T(planeNorm.z()));
		residual[0] = norm.dot(lp) + T(planeIntercept);
		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d _currPoint, const Eigen::Vector3d _planeNorm,
									   const double _planeIntercept)
	{
		return (new ceres::AutoDiffCostFunction<LaserSurfNormFeatureFunctor, 1, 4, 3>(
			new LaserSurfNormFeatureFunctor(_currPoint, _planeNorm, _planeIntercept)));
	}

	Eigen::Vector3d currPoint;
	Eigen::Vector3d planeNorm;
	double planeIntercept;
};


Eigen::Matrix<double,3,3> skew(Eigen::Matrix<double,3,1>& mat_in)
{
    Eigen::Matrix<double,3,3> skew_mat;
    skew_mat.setZero();
    skew_mat(0,1) = -mat_in(2);
    skew_mat(0,2) =  mat_in(1);
    skew_mat(1,2) = -mat_in(0);
    skew_mat(1,0) =  mat_in(2);
    skew_mat(2,0) = -mat_in(1);
    skew_mat(2,1) =  mat_in(0);
    return skew_mat;
}

void getTransformFromSe3(const Eigen::Matrix<double,6,1>& se3, Eigen::Quaterniond& q, Eigen::Vector3d& t)
{
    Eigen::Vector3d omega(se3.data());
    Eigen::Vector3d upsilon(se3.data()+3);
    Eigen::Matrix3d Omega = skew(omega);

    double theta = omega.norm();
    double half_theta = 0.5*theta;

    double imag_factor;
    double real_factor = cos(half_theta);
    if (theta<1e-10) {
        double theta_sq = theta*theta;
        double theta_po4 = theta_sq*theta_sq;
        imag_factor = 0.5-0.0208333*theta_sq+0.000260417*theta_po4;
    }
    else {
        double sin_half_theta = sin(half_theta);
        imag_factor = sin_half_theta/theta;
    }

    q = Eigen::Quaterniond(real_factor, imag_factor*omega.x(), imag_factor*omega.y(), imag_factor*omega.z());

    Eigen::Matrix3d J;
    if (theta<1e-10) {
        J = q.matrix();
    }
    else {
        Eigen::Matrix3d Omega2 = Omega*Omega;
        J = (Eigen::Matrix3d::Identity() + (1-cos(theta))/(theta*theta)*Omega + (theta-sin(theta))/(pow(theta,3))*Omega2);
    }

    t = J*upsilon;
}

class EdgeAnalyticCostFunction : public ceres::SizedCostFunction<1, 7>
{
public:
    EdgeAnalyticCostFunction(Eigen::Vector3d _currPoint, Eigen::Vector3d _lastPointA, Eigen::Vector3d _lastPointB)
        : currPoint(_currPoint), lastPointA(_lastPointA), lastPointB(_lastPointB) {}
    virtual ~EdgeAnalyticCostFunction() {}

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        Eigen::Map<const Eigen::Quaterniond> quaterCurr2Last(parameters[0]);
        Eigen::Map<const Eigen::Vector3d> transCurr2Last(parameters[0] + 4);
        Eigen::Vector3d lp = quaterCurr2Last * currPoint + transCurr2Last;

        Eigen::Vector3d nu = (lp - lastPointA).cross(lp - lastPointB);
        Eigen::Vector3d de = lastPointA - lastPointB;
        double de_norm = de.norm();
        residuals[0] = nu.norm()/de_norm;

        if(jacobians != NULL) {
            if(jacobians[0] != NULL) {
                Eigen::Matrix3d skew_lp = skew(lp);
                Eigen::Matrix<double, 3, 6> dp_by_se3;
                dp_by_se3.block<3,3>(0,0) = -skew_lp;
                (dp_by_se3.block<3,3>(0, 3)).setIdentity();
                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > J_se3(jacobians[0]);
                J_se3.setZero();
                Eigen::Matrix3d skew_de = skew(de);
                J_se3.block<1,6>(0,0) = - nu.transpose() / nu.norm() * skew_de * dp_by_se3/de_norm;
            }
        }

        return true;
    }

private:
    Eigen::Vector3d currPoint;
    Eigen::Vector3d lastPointA;
    Eigen::Vector3d lastPointB;
};

class SurfNormAnalyticCostFunction : public ceres::SizedCostFunction<1, 7>
{
public:
    SurfNormAnalyticCostFunction(Eigen::Vector3d _currPoint, Eigen::Vector3d _planeNorm, double _planeIntercept)
        : currPoint(_currPoint), planeNorm(_planeNorm), planeIntercept(_planeIntercept) {}
    virtual ~SurfNormAnalyticCostFunction() {}

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        Eigen::Map<const Eigen::Quaterniond> quaterCurr2Last(parameters[0]);
        Eigen::Map<const Eigen::Vector3d> transCurr2Last(parameters[0] + 4);
        Eigen::Vector3d lp = quaterCurr2Last * currPoint + transCurr2Last;
        residuals[0] = planeNorm.dot(lp) + planeIntercept;

        if(jacobians != NULL) {
            if(jacobians[0] != NULL) {
                Eigen::Matrix3d skew_lp = skew(lp);
                Eigen::Matrix<double, 3, 6> dp_by_se3;
                dp_by_se3.block<3,3>(0,0) = -skew_lp;
                (dp_by_se3.block<3,3>(0, 3)).setIdentity();
                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > J_se3(jacobians[0]);
                J_se3.setZero();
                J_se3.block<1,6>(0,0) = planeNorm.transpose() * dp_by_se3;
            }
        }
        return true;
    }

private:
    Eigen::Vector3d currPoint;
    Eigen::Vector3d planeNorm;
    double planeIntercept;
};

class PoseSE3Parameterization : public ceres::LocalParameterization
{
public:
    PoseSE3Parameterization() {}
    virtual ~PoseSE3Parameterization() {}

    virtual int GlobalSize() const { return 7; }
    virtual int LocalSize() const { return 6; }

    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const
    {
        Eigen::Quaterniond delta_q;
        Eigen::Vector3d delta_t;
        getTransformFromSe3(Eigen::Map<const Eigen::Matrix<double,6,1>>(delta), delta_q, delta_t);

        Eigen::Map<const Eigen::Quaterniond> quater(x);
        Eigen::Map<const Eigen::Vector3d> trans(x + 4);
        Eigen::Map<Eigen::Quaterniond> quater_plus(x_plus_delta);
        Eigen::Map<Eigen::Vector3d> trans_plus(x_plus_delta + 4);

        quater_plus = delta_q * quater;
        trans_plus = delta_q * trans + delta_t;

        return true;
    }

    virtual bool ComputeJacobian(const double* x, double* jacobian) const
    {
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
        (j.topRows(6)).setIdentity();
        (j.bottomRows(1)).setZero();
        return true;
    }
};


LaserFeatureRegistrationCeres::LaserFeatureRegistrationCeres(const Options& options)
    : LaserFeatureRegistration(options)
{
}

LaserFeatureRegistrationCeres::~LaserFeatureRegistrationCeres()
{
}

void LaserFeatureRegistrationCeres::setInitPose(const Eigen::Isometry3d& initPose)
{
    quaterCurr2Last_ = Eigen::Quaterniond(initPose.rotation());
    transCurr2Last_ = initPose.translation();
}

bool LaserFeatureRegistrationCeres::getFinalPose(Eigen::Isometry3d& finalPose)
{
    finalPose = Eigen::Isometry3d::Identity();
    finalPose.linear() = quaterCurr2Last_.toRotationMatrix();
    finalPose.translation() = transCurr2Last_;
    return true;
}

void LaserFeatureRegistrationCeres::pointAssociateToMap(const pcl::PointXYZI& inp, pcl::PointXYZI& outp)
{
	Eigen::Vector3d inPoint(inp.x, inp.y, inp.z);
	Eigen::Vector3d outPoint = quaterCurr2Last_ * inPoint + transCurr2Last_;
	outp.x = outPoint.x();
	outp.y = outPoint.y();
	outp.z = outPoint.z();
	outp.intensity = inp.intensity;
}

bool LaserFeatureRegistrationCeres::prepareProcessing()
{
    ceres::Problem::Options problem_options;
    problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problem_.reset(new ceres::Problem(problem_options));
    lossFunction_.reset(new ceres::HuberLoss(0.1));
    ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
    problem_->AddParameterBlock(parameters_, 4, q_parameterization);
    problem_->AddParameterBlock(parameters_ + 4, 3);
    return true;
}

void LaserFeatureRegistrationCeres::addOneEdgeFeature(const pcl::PointXYZI& pointOri, const pcl::PointXYZI& pointSel, const Eigen::Vector3d& currPoint,
        const Eigen::Vector3d& lastPointA, const Eigen::Vector3d& lastPointB)
{
    ceres::CostFunction *costFunction = LaserEdgeFeatureFunctor::Create(currPoint, lastPointA, lastPointB, 1.0);
    problem_->AddResidualBlock(costFunction, lossFunction_.get(), parameters_, parameters_ + 4);
}

void LaserFeatureRegistrationCeres::addOneSurfFeature(const pcl::PointXYZI& pointOri, const pcl::PointXYZI& pointSel, const Eigen::Vector3d& currPoint,
        const Eigen::Vector3d& planeNorm, double planeIntercept)
{
    ceres::CostFunction *costFunction = LaserSurfNormFeatureFunctor::Create(currPoint, planeNorm, planeIntercept);
    problem_->AddResidualBlock(costFunction, lossFunction_.get(), parameters_, parameters_ + 4);
}

bool LaserFeatureRegistrationCeres::solveOptimProblem(int iterCount, bool& converged)
{
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 4;
    options.minimizer_progress_to_stdout = false;
    options.check_gradients = false;
    options.gradient_check_relative_precision = 1e-4;
    ceres::Solver::Summary summary;
    ceres::Solve(options, problem_.get(), &summary);
    return true;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Declare LaserFeatureRegistrationNewton

class LaserFeatureRegistrationNewton : public LaserFeatureRegistration
{
public:
    LaserFeatureRegistrationNewton(const Options& options);
    virtual ~LaserFeatureRegistrationNewton();

protected:
    virtual void setInitPose(const Eigen::Isometry3d& initPose);
    virtual bool getFinalPose(Eigen::Isometry3d& finalPose);
    virtual void pointAssociateToMap(const pcl::PointXYZI& inp, pcl::PointXYZI& outp);
    virtual bool prepareProcessing();
    virtual void addOneEdgeFeature(const pcl::PointXYZI& pointOri, const pcl::PointXYZI& pointSel, const Eigen::Vector3d& currPoint,
        const Eigen::Vector3d& lastPointA, const Eigen::Vector3d& lastPointB);
    virtual void addOneSurfFeature(const pcl::PointXYZI& pointOri, const pcl::PointXYZI& pointSel, const Eigen::Vector3d& currPoint,
        const Eigen::Vector3d& planeNorm, double planeIntercept);
    virtual bool solveOptimProblem(int iterCount, bool& converged);

protected:
    float transformTobeMapped[6] = {0, 0, 0, 0, 0, 0};
    Eigen::Affine3f transPointAssociateToMap;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudOri;
    pcl::PointCloud<pcl::PointXYZI>::Ptr coeffSel;
    bool isDegenerate = false;
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Define LaserFeatureRegistrationNewton

inline Eigen::Affine3f transToAffine3f(float transformIn[])
{
    return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
}

inline void affine3fToTrans(const Eigen::Affine3f& affinePose, float transformIn[])
{
    pcl::getTranslationAndEulerAngles(affinePose, transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
}

LaserFeatureRegistrationNewton::LaserFeatureRegistrationNewton(const Options& options)
    : LaserFeatureRegistration(options)
{
}

LaserFeatureRegistrationNewton::~LaserFeatureRegistrationNewton()
{
}

void LaserFeatureRegistrationNewton::setInitPose(const Eigen::Isometry3d& initPose)
{
    Eigen::Affine3f affinePose = Eigen::Affine3f::Identity();
    affinePose.linear() = initPose.rotation().cast<float>();
    affinePose.translation() = initPose.translation().cast<float>();
    affine3fToTrans(affinePose, transformTobeMapped);
}

bool LaserFeatureRegistrationNewton::getFinalPose(Eigen::Isometry3d& finalPose)
{
    Eigen::Affine3f affinePose = transToAffine3f(transformTobeMapped);
    finalPose = Eigen::Isometry3d::Identity();
    finalPose.linear() = affinePose.rotation().cast<double>();
    finalPose.translation() = affinePose.translation().cast<double>();
    return true;
}

void LaserFeatureRegistrationNewton::pointAssociateToMap(const pcl::PointXYZI& inp, pcl::PointXYZI& outp)
{
    outp.x = transPointAssociateToMap(0,0) * inp.x + transPointAssociateToMap(0,1) * inp.y + transPointAssociateToMap(0,2) * inp.z + transPointAssociateToMap(0,3);
    outp.y = transPointAssociateToMap(1,0) * inp.x + transPointAssociateToMap(1,1) * inp.y + transPointAssociateToMap(1,2) * inp.z + transPointAssociateToMap(1,3);
    outp.z = transPointAssociateToMap(2,0) * inp.x + transPointAssociateToMap(2,1) * inp.y + transPointAssociateToMap(2,2) * inp.z + transPointAssociateToMap(2,3);
	outp.intensity = inp.intensity;
}

bool LaserFeatureRegistrationNewton::prepareProcessing()
{
    transPointAssociateToMap = transToAffine3f(transformTobeMapped);
    laserCloudOri->clear();
    coeffSel->clear();
    return true;
}

void LaserFeatureRegistrationNewton::addOneEdgeFeature(const pcl::PointXYZI& pointOri, const pcl::PointXYZI& pointSel, const Eigen::Vector3d& currPoint,
        const Eigen::Vector3d& lastPointA, const Eigen::Vector3d& lastPointB)
{
    float x0 = pointSel.x;
    float y0 = pointSel.y;
    float z0 = pointSel.z;
    float x1 = lastPointA.x();
    float y1 = lastPointA.y();
    float z1 = lastPointA.z();
    float x2 = lastPointB.x();
    float y2 = lastPointB.y();
    float z2 = lastPointB.z();

    float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                    + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                    + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)) * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));
    float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));
    float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;
    float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;
    float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;
    float ld2 = a012 / l12;

    float s = 1 - 0.9 * fabs(ld2);
    pcl::PointXYZI coeff;
    coeff.x = s * la;
    coeff.y = s * lb;
    coeff.z = s * lc;
    coeff.intensity = s * ld2;

    if (s > 0.1) {
        laserCloudOri->push_back(pointOri);
        coeffSel->push_back(coeff);
    }
}

void LaserFeatureRegistrationNewton::addOneSurfFeature(const pcl::PointXYZI& pointOri, const pcl::PointXYZI& pointSel, const Eigen::Vector3d& currPoint,
        const Eigen::Vector3d& planeNorm, double planeIntercept)
{
    double pa = planeNorm.x();
    double pb = planeNorm.y();
    double pc = planeNorm.z();
    double pd = planeIntercept;
    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x
            + pointOri.y * pointOri.y + pointOri.z * pointOri.z));
    pcl::PointXYZI coeff;
    coeff.x = s * pa;
    coeff.y = s * pb;
    coeff.z = s * pc;
    coeff.intensity = s * pd2;

    if (s > 0.1) {
        laserCloudOri->push_back(pointOri);
        coeffSel->push_back(coeff);
    }
}

bool LaserFeatureRegistrationNewton::solveOptimProblem(int iterCount, bool& converged)
{
    // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
    // lidar <- camera      ---     camera <- lidar
    // x = z                ---     x = y
    // y = x                ---     y = z
    // z = y                ---     z = x
    // roll = yaw           ---     roll = pitch
    // pitch = roll         ---     pitch = yaw
    // yaw = pitch          ---     yaw = roll

    // lidar -> camera
    float srx = sin(transformTobeMapped[1]);
    float crx = cos(transformTobeMapped[1]);
    float sry = sin(transformTobeMapped[2]);
    float cry = cos(transformTobeMapped[2]);
    float srz = sin(transformTobeMapped[0]);
    float crz = cos(transformTobeMapped[0]);

    int laserCloudSelNum = laserCloudOri->size();
    if (laserCloudSelNum < 50) {
        return false;
    }

    cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

    for (int i = 0; i < laserCloudSelNum; i++) {
        pcl::PointXYZI pointOri, coeff;
        // lidar -> camera
        pointOri.x = laserCloudOri->points[i].y;
        pointOri.y = laserCloudOri->points[i].z;
        pointOri.z = laserCloudOri->points[i].x;
        // lidar -> camera
        coeff.x = coeffSel->points[i].y;
        coeff.y = coeffSel->points[i].z;
        coeff.z = coeffSel->points[i].x;
        coeff.intensity = coeffSel->points[i].intensity;
        // in camera
        float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                    + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                    + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;
        float ary = ((cry*srx*srz - crz*sry)*pointOri.x + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                    + ((-cry*crz - srx*sry*srz)*pointOri.x + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;
        float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                    + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                    + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;
        // camera -> lidar
        matA.at<float>(i, 0) = arz;
        matA.at<float>(i, 1) = arx;
        matA.at<float>(i, 2) = ary;
        matA.at<float>(i, 3) = coeff.z;
        matA.at<float>(i, 4) = coeff.x;
        matA.at<float>(i, 5) = coeff.y;
        matB.at<float>(i, 0) = -coeff.intensity;
    }

    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    cv::Mat matP;
    matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));

    if (iterCount == 0) {

        cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

        cv::eigen(matAtA, matE, matV);
        matV.copyTo(matV2);

        isDegenerate = false;
        float eignThre[6] = {100, 100, 100, 100, 100, 100};
        for (int i = 5; i >= 0; i--) {
            if (matE.at<float>(0, i) < eignThre[i]) {
                for (int j = 0; j < 6; j++) {
                    matV2.at<float>(i, j) = 0;
                }
                isDegenerate = true;
            } else {
                break;
            }
        }
        matP = matV.inv() * matV2;
    }

    if (isDegenerate) {
        cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
        matX.copyTo(matX2);
        matX = matP * matX2;
    }

    transformTobeMapped[0] += matX.at<float>(0, 0);
    transformTobeMapped[1] += matX.at<float>(1, 0);
    transformTobeMapped[2] += matX.at<float>(2, 0);
    transformTobeMapped[3] += matX.at<float>(3, 0);
    transformTobeMapped[4] += matX.at<float>(4, 0);
    transformTobeMapped[5] += matX.at<float>(5, 0);

    float deltaR = sqrt(
                        pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                        pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                        pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
    float deltaT = sqrt(
                        pow(matX.at<float>(3, 0) * 100, 2) +
                        pow(matX.at<float>(4, 0) * 100, 2) +
                        pow(matX.at<float>(5, 0) * 100, 2));

    if (deltaR < 0.05 && deltaT < 0.05) {
        converged = true;
    }
    return true;
}


LaserFeatureRegistration* LaserFeatureRegistration::createInstance(Type type, const Options& options)
{
    LaserFeatureRegistration *instance = nullptr;

    if (type == LaserFeatureRegistration::NEWTON) {
        instance = new LaserFeatureRegistrationNewton(options);
    }
    else if (type == LaserFeatureRegistration::CERES) {
        instance = new LaserFeatureRegistrationCeres(options);
    }

    return instance;
}


