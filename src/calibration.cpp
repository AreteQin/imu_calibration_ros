/*
 * imu_tk - Inertial Measurement Unit Toolkit
 *
 *  Copyright (c) 2014, Alberto Pretto <pretto@diag.uniroma1.it>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "calibration.h"
#include "filters.h"
#include "integration.h"
//#include "imu_tk/visualization.h"

#include <limits>
#include <iostream>
#include "ceres/ceres.h"

using namespace imu_tk;
using namespace Eigen;
using namespace std;

template
class MultiPosCalibration_<double>;

template
class MultiPosCalibration_<float>;

template<typename T1>
struct MultiPosAccResidual {
    MultiPosAccResidual(const T1 &g_mag, const Eigen::Matrix<T1, 3, 1> &sample) :
            g_mag_(g_mag),
            sample_(sample) {}

    template<typename _T2>
    bool operator()(const _T2 *const params, _T2 *residuals) const {
        Eigen::Matrix<_T2, 3, 1> raw_samp(_T2(sample_(0)), _T2(sample_(1)), _T2(sample_(2)));
        /* Assume body frame same as accelerometer frame,
         * so bottom left params in the misalignment matris are set to zero */
        CalibratedTriad_<_T2> calib_triad(params[0], params[1], params[2],
                                          _T2(0), _T2(0), _T2(0),
                                          params[3], params[4], params[5],
                                          params[6], params[7], params[8]);

        Eigen::Matrix<_T2, 3, 1> calib_samp = calib_triad.unbiasNormalize(raw_samp); // h(a,theta) = T*K*(a-b)
        residuals[0] = _T2(g_mag_) - calib_samp.norm();
        return true;
    }

    static ceres::CostFunction *Create(const T1 &g_mag, const Eigen::Matrix<T1, 3, 1> &sample) {
        return (new ceres::AutoDiffCostFunction<MultiPosAccResidual, 1, 9>(
                new MultiPosAccResidual<T1>(g_mag, sample)));
    }

    const T1 g_mag_;
    const Eigen::Matrix<T1, 3, 1> sample_;
};

template<typename T1>
struct MultiPosGyroResidual {
    MultiPosGyroResidual(const Eigen::Matrix<T1, 3, 1> &g_versor_pos0,
                         const Eigen::Matrix<T1, 3, 1> &g_versor_pos1,
                         const std::vector<TriadData_<T1> > &gyro_samples,
                         const DataInterval &gyro_interval_pos01,
                         T1 dt, bool optimize_bias) :

            g_versor_pos0_(g_versor_pos0),
            g_versor_pos1_(g_versor_pos1),
            gyro_samples_(gyro_samples),
            interval_pos01_(gyro_interval_pos01),
            dt_(dt), optimize_bias_(optimize_bias) {}

    template<typename _T2>
    bool operator()(const _T2 *const params, _T2 *residuals) const {
        CalibratedTriad_<_T2> calib_triad(params[0], params[1], params[2],
                                          params[3], params[4], params[5],
                                          params[6], params[7], params[8],
                                          optimize_bias_ ? params[9] : _T2(0),
                                          optimize_bias_ ? params[10] : _T2(0),
                                          optimize_bias_ ? params[11] : _T2(0)); // 是否优化偏置，否则这一项为零（因为已经去除了bias）

        std::vector<TriadData_<_T2> > calib_gyro_samples;
        calib_gyro_samples.reserve(interval_pos01_.end_idx - interval_pos01_.start_idx + 1);

        for (int i = interval_pos01_.start_idx; i <= interval_pos01_.end_idx; i++)
            calib_gyro_samples.push_back(TriadData_<_T2>(calib_triad.unbiasNormalize(gyro_samples_[i])));

        Eigen::Matrix<_T2, 3, 3> rot_mat;
        integrateGyroInterval(calib_gyro_samples, rot_mat, _T2(dt_));

        Eigen::Matrix<_T2, 3, 1> diff = rot_mat.transpose() * g_versor_pos0_.template cast<_T2>() -
                                        g_versor_pos1_.template cast<_T2>();

        residuals[0] = diff(0);
        residuals[1] = diff(1);
        residuals[2] = diff(2);

        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Matrix<T1, 3, 1> &g_versor_pos0,
                                       const Eigen::Matrix<T1, 3, 1> &g_versor_pos1,
                                       const std::vector<TriadData_<T1> > &gyro_samples,
                                       const DataInterval &gyro_interval_pos01,
                                       T1 dt, bool optimize_bias) {
        if (optimize_bias)
            return (new ceres::AutoDiffCostFunction<MultiPosGyroResidual, 3, 12>(
                    new MultiPosGyroResidual(g_versor_pos0, g_versor_pos1, gyro_samples,
                                             gyro_interval_pos01, dt, optimize_bias)));
        else
            return (new ceres::AutoDiffCostFunction<MultiPosGyroResidual, 3, 9>(
                    new MultiPosGyroResidual(g_versor_pos0, g_versor_pos1, gyro_samples,
                                             gyro_interval_pos01, dt, optimize_bias)));
    }

    const Eigen::Matrix<T1, 3, 1> g_versor_pos0_, g_versor_pos1_;
    const std::vector<TriadData_<T1> > gyro_samples_;
    const DataInterval interval_pos01_;
    const T1 dt_;
    const bool optimize_bias_;
};

template<typename T>
MultiPosCalibration_<T>::MultiPosCalibration_() :
        g_mag_(9.8),
        min_num_intervals_(12),
        init_interval_duration_(T(30.0)),
        interval_n_samples_(100),
        acc_use_means_(false),
        gyro_dt_(-1.0),
        optimize_gyro_bias_(false),
        verbose_output_(false) {}

template<typename T>
bool MultiPosCalibration_<T>::calibrateAcc(const std::vector<TriadData_<T> > &acc_samples) {

    min_cost_static_intervals_.clear();
    calib_acc_samples_.clear();
    calib_gyro_samples_.clear();

    int n_samps = acc_samples.size();
    LOG(INFO) << "Accelerometers calibration: " << n_samps << " measurements" << endl;

    DataInterval init_static_interval = DataInterval::initialInterval(acc_samples, init_interval_duration_);
    LOG(INFO) << "Start index: " << init_static_interval.start_idx << endl;
    LOG(INFO) << "End index: " << init_static_interval.end_idx << endl;
    Eigen::Matrix<T, 3, 1> acc_variance = dataVariance(acc_samples, init_static_interval);
    T norm_th = acc_variance.norm();

    LOG(INFO) << "Accelerometer's variance norm: " << norm_th << endl;

    T min_cost = std::numeric_limits<T>::max();
    int min_cost_th = -1;
    std::vector<double> min_cost_calib_params;

    for (int th_mult = 2;
         th_mult <= 10; th_mult++) { // test different integers multiple of the variance magnitude
        std::vector<imu_tk::DataInterval> static_intervals;
        std::vector<imu_tk::TriadData_<T> > static_samples;
        std::vector<double> acc_calib_params(9);

        acc_calib_params[0] = init_acc_calib_.misYZ();
        acc_calib_params[1] = init_acc_calib_.misZY();
        acc_calib_params[2] = init_acc_calib_.misZX();

        acc_calib_params[3] = init_acc_calib_.scaleX();
        acc_calib_params[4] = init_acc_calib_.scaleY();
        acc_calib_params[5] = init_acc_calib_.scaleZ();

        acc_calib_params[6] = init_acc_calib_.biasX();
        acc_calib_params[7] = init_acc_calib_.biasY();
        acc_calib_params[8] = init_acc_calib_.biasZ();

        std::vector<DataInterval> extracted_intervals;
        staticIntervalsDetector(acc_samples, th_mult * norm_th, static_intervals);
        extractIntervalsSamples(acc_samples, static_intervals,
                                static_samples, extracted_intervals,
                                interval_n_samples_, acc_use_means_);

        if (verbose_output_) {
            LOG(INFO) << "Accelerometers calibration: extracted " << extracted_intervals.size()
                      << " intervals using threshold multiplier " << th_mult << " -> ";
        }

// Perform here a quality test
        if (extracted_intervals.size() < min_num_intervals_) {
            if (verbose_output_) LOG(WARNING) << "Not enough intervals, calibration is not reliable" << endl;
            continue;
        }

        if (verbose_output_) LOG(INFO) << "Trying calibrate... " << endl;

        ceres::Problem problem;
        for (int i = 0; i < static_samples.size(); i++) {
            ceres::CostFunction *cost_function =
                    MultiPosAccResidual<T>::Create(g_mag_, static_samples[i].data());

            problem.AddResidualBlock(cost_function, NULL /* squared loss */, acc_calib_params.data());
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = verbose_output_;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        if (summary.final_cost < min_cost) {
            min_cost = summary.final_cost;
            min_cost_th = th_mult;
            min_cost_static_intervals_ = static_intervals;
            min_cost_calib_params = acc_calib_params;
        }
        cout << "residual " << summary.final_cost << endl;
    }

    if (min_cost_th < 0) {
        if (verbose_output_)
            LOG(ERROR) << "Accelerometers calibration: Can't obtain any calibratin with the current dataset" << endl;
        return false;
    }

    acc_calib_ = CalibratedTriad_<T>(min_cost_calib_params[0],
                                     min_cost_calib_params[1],
                                     min_cost_calib_params[2],
                                     0, 0, 0,
                                     min_cost_calib_params[3],
                                     min_cost_calib_params[4],
                                     min_cost_calib_params[5],
                                     min_cost_calib_params[6],
                                     min_cost_calib_params[7],
                                     min_cost_calib_params[8]);

    calib_acc_samples_.reserve(n_samps);

// Calibrate the input accelerometer data with the obtained calibration
    for (int i = 0; i < n_samps; i++)
        calib_acc_samples_.push_back(acc_calib_.unbiasNormalize(acc_samples[i]));

    if (verbose_output_) {
//        Plot plot;
//        plot.plotIntervals(calib_acc_samples_, min_cost_static_intervals_);

        LOG(INFO) << "Accelerometers calibration: Better calibration obtained using threshold multiplier "
                  << min_cost_th
                  << " with residual " << min_cost << endl
                  << acc_calib_ << endl
                  << "Accelerometers calibration: inverse scale factors:" << endl
                  << 1.0 / acc_calib_.scaleX() << endl
                  << 1.0 / acc_calib_.scaleY() << endl
                  << 1.0 / acc_calib_.scaleZ() << endl;

//        waitForKey();
    }

    return true;

}

template<typename T>
bool MultiPosCalibration_<T>::calibrateAccGyro(const vector<TriadData_<T> > &acc_samples,
                                               const vector<TriadData_<T> > &gyro_samples) {
    if (!calibrateAcc(acc_samples))
        return false;

    LOG(INFO) << "Gyroscopes calibration: calibrating...";

    std::vector<TriadData_<T> > static_acc_means;
    std::vector<DataInterval> extracted_intervals;
    extractIntervalsSamples(calib_acc_samples_, min_cost_static_intervals_,
                            static_acc_means, extracted_intervals,
                            interval_n_samples_, true);

    int n_static_pos = static_acc_means.size(), n_samps = gyro_samples.size();

// Compute the gyroscopes biases in the (static) initialization interval
    DataInterval init_static_interval = DataInterval::initialInterval(gyro_samples, init_interval_duration_);
    Eigen::Matrix<T, 3, 1> gyro_bias = dataMean(gyro_samples, init_static_interval);

    gyro_calib_ = CalibratedTriad_<T>(0, 0, 0, 0, 0, 0,
                                      1.0, 1.0, 1.0,
                                      gyro_bias(0), gyro_bias(1), gyro_bias(2));


// calibrated_gyro_samples_ already cleared in calibrateAcc()
    calib_gyro_samples_.reserve(n_samps);
// Remove the bias
    for (int i = 0; i < n_samps; i++)
        calib_gyro_samples_.push_back(gyro_calib_.unbias(gyro_samples[i]));

    std::vector<double> gyro_calib_params(12);

    gyro_calib_params[0] = init_gyro_calib_.misYZ();
    gyro_calib_params[1] = init_gyro_calib_.misZY();
    gyro_calib_params[2] = init_gyro_calib_.misZX();
    gyro_calib_params[3] = init_gyro_calib_.misXZ();
    gyro_calib_params[4] = init_gyro_calib_.misXY();
    gyro_calib_params[5] = init_gyro_calib_.misYX();

    gyro_calib_params[6] = init_gyro_calib_.scaleX();
    gyro_calib_params[7] = init_gyro_calib_.scaleY();
    gyro_calib_params[8] = init_gyro_calib_.scaleZ();

// Bias has been estimated and removed in the initialization period
    gyro_calib_params[9] = 0.0;
    gyro_calib_params[10] = 0.0;
    gyro_calib_params[11] = 0.0;

    ceres::Problem problem;

    for (int i = 0; i < n_static_pos - 1; i++) {
        Eigen::Matrix<T, 3, 1> g_versor_pos0 = static_acc_means[i].data(),
                g_versor_pos1 = static_acc_means[i + 1].data();

// calculate directions of gravity in the two static intervals
        g_versor_pos0 /= g_versor_pos0.norm();
        g_versor_pos1 /= g_versor_pos1.norm();

        int gyro_idx0 = -1, gyro_idx1 = -1;
        T ts0 = calib_acc_samples_[extracted_intervals[i].end_idx].timestamp(),
                ts1 = calib_acc_samples_[extracted_intervals[i + 1].start_idx].timestamp();

// Assume monotone signal time
// determine the start and end indices of the gyroscopes samples in the two static intervals
        for (int t_idx = 0; t_idx < n_samps; t_idx++) {
            if (gyro_idx0 < 0) { // first sample
                if (calib_gyro_samples_[t_idx].timestamp() >= ts0) // rotation start time
                    gyro_idx0 = t_idx;
            } else { // not first sample
                if (calib_gyro_samples_[t_idx].timestamp() >= ts1) { // rotation end time
                    gyro_idx1 = t_idx - 1;
                    break;
                }
            }
        }

//     cout<<"from "<<calibrated_gyro_samples_[gyro_idx0].timestamp()<<" to "
//         <<calibrated_gyro_samples_[gyro_idx1].timestamp()
//         <<" v0 : "<< g_versor_pos0(0)<<" "<< g_versor_pos0(1)<<" "<< g_versor_pos0(2)
//         <<" v1 : "<< g_versor_pos1(0)<<" "<< g_versor_pos1(1)<<" "<< g_versor_pos1(2)<<endl;

        DataInterval gyro_interval(gyro_idx0, gyro_idx1);

        ceres::CostFunction *cost_function =
                MultiPosGyroResidual<T>::Create(g_versor_pos0, g_versor_pos1, calib_gyro_samples_,
                                                gyro_interval, gyro_dt_, optimize_gyro_bias_);

        problem.AddResidualBlock(cost_function, NULL /* squared loss */, gyro_calib_params.data());

    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = verbose_output_;

    ceres::Solver::Summary summary;

    ceres::Solve(options, &problem, &summary);
    gyro_calib_ = CalibratedTriad_<T>(gyro_calib_params[0],
                                      gyro_calib_params[1],
                                      gyro_calib_params[2],
                                      gyro_calib_params[3],
                                      gyro_calib_params[4],
                                      gyro_calib_params[5],
                                      gyro_calib_params[6],
                                      gyro_calib_params[7],
                                      gyro_calib_params[8],
                                      gyro_bias(0) + gyro_calib_params[9],
                                      gyro_bias(1) + gyro_calib_params[10],
                                      gyro_bias(2) + gyro_calib_params[11]);

// Calibrate the input gyroscopes data with the obtained calibration
    for (int i = 0; i < n_samps; i++)
        calib_gyro_samples_.push_back(gyro_calib_.unbiasNormalize(gyro_samples[i]));

    if (verbose_output_) {

        LOG(INFO) << summary.FullReport() << endl;
        LOG(INFO) << "Gyroscopes calibration: residual " << summary.final_cost << endl
                  << gyro_calib_ << endl
                  << "Gyroscopes calibration: inverse scale factors:" << endl
                  << 1.0 / gyro_calib_.scaleX() << endl
                  << 1.0 / gyro_calib_.scaleY() << endl
                  << 1.0 / gyro_calib_.scaleZ() << endl;
    }

    return true;
}
