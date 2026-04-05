#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

namespace rc26_localization {

class ESIKF {
public:
    ESIKF() { reset(Eigen::Isometry3d::Identity()); }

    void setProcessNoise(double accel_noise, double gyro_noise, double accel_bias_noise, double gyro_bias_noise) {
        Q_.setZero();
        Q_.block<3, 3>(0, 0).diagonal().setConstant(std::max(accel_noise, 1e-6));
        Q_.block<3, 3>(3, 3).diagonal().setConstant(std::max(gyro_noise, 1e-6));
        Q_.block<3, 3>(6, 6).diagonal().setConstant(std::max(accel_noise, 1e-6));
        Q_.block<3, 3>(9, 9).diagonal().setConstant(std::max(accel_bias_noise, 1e-8));
        Q_.block<3, 3>(12, 12).diagonal().setConstant(std::max(gyro_bias_noise, 1e-8));
    }

    void reset(const Eigen::Isometry3d& T_map_odom) {
        p_ = T_map_odom.translation();
        q_ = Eigen::Quaterniond(T_map_odom.rotation());
        v_.setZero();
        b_a_.setZero();
        b_g_.setZero();
        P_.setIdentity();
        P_ *= 1e-2;
    }

    void predict(const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, double dt) {
        if (dt <= 0.0) {
            return;
        }
        const Eigen::Vector3d omega = gyro - b_g_;
        const double theta = omega.norm() * dt;
        Eigen::Quaterniond dq = Eigen::Quaterniond::Identity();
        if (theta > 1e-12) {
            dq = Eigen::Quaterniond(Eigen::AngleAxisd(theta, omega.normalized()));
        }
        q_ = (q_ * dq).normalized();

        Eigen::Vector3d acc_world = q_ * (accel - b_a_);
        acc_world.z() -= 9.81;
        p_ += v_ * dt + 0.5 * acc_world * dt * dt;
        v_ += acc_world * dt;

        P_ += Q_ * dt;
    }

    void update(const Eigen::Matrix4d& T_gicp, const Eigen::Matrix<double, 6, 6>& R_obs) {
        const Eigen::Vector3d p_meas = T_gicp.block<3, 1>(0, 3);
        const Eigen::Quaterniond q_meas(T_gicp.block<3, 3>(0, 0));

        const Eigen::Vector3d dp = p_meas - p_;
        const Eigen::Matrix3d R_err = q_.toRotationMatrix().transpose() * q_meas.toRotationMatrix();
        const Eigen::AngleAxisd aa(R_err);
        Eigen::Vector3d dtheta = Eigen::Vector3d::Zero();
        if (std::isfinite(aa.angle()) && aa.axis().allFinite()) {
            dtheta = aa.axis() * aa.angle();
        }

        Eigen::Matrix<double, 6, 1> y;
        y << dp, dtheta;

        Eigen::Matrix<double, 6, 15> H = Eigen::Matrix<double, 6, 15>::Zero();
        H.block<3, 3>(0, 0).setIdentity();
        H.block<3, 3>(3, 3).setIdentity();

        const Eigen::Matrix<double, 6, 6> S = H * P_ * H.transpose() + R_obs;
        const Eigen::Matrix<double, 15, 6> K = P_ * H.transpose() * S.inverse();
        const Eigen::Matrix<double, 15, 1> dx = K * y;

        p_ += dx.block<3, 1>(0, 0);
        q_ = (q_ * smallAngleQuat(dx.block<3, 1>(3, 0))).normalized();
        v_ += dx.block<3, 1>(6, 0);
        b_a_ += dx.block<3, 1>(9, 0);
        b_g_ += dx.block<3, 1>(12, 0);

        const Eigen::Matrix<double, 15, 15> I = Eigen::Matrix<double, 15, 15>::Identity();
        P_ = (I - K * H) * P_;
    }

    Eigen::Isometry3d getMapToOdom() const {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() = q_.toRotationMatrix();
        T.translation() = p_;
        return T;
    }

    Eigen::Matrix<double, 6, 6> getCovariance() const {
        Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Zero();
        cov.block<3, 3>(0, 0) = P_.block<3, 3>(0, 0);
        cov.block<3, 3>(3, 3) = P_.block<3, 3>(3, 3);
        return cov;
    }

private:
    static Eigen::Quaterniond smallAngleQuat(const Eigen::Vector3d& dtheta) {
        const double norm = dtheta.norm();
        if (norm < 1e-12) {
            return Eigen::Quaterniond::Identity();
        }
        return Eigen::Quaterniond(Eigen::AngleAxisd(norm, dtheta / norm));
    }

    Eigen::Vector3d p_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d v_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d b_a_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d b_g_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond q_{Eigen::Quaterniond::Identity()};
    Eigen::Matrix<double, 15, 15> P_{Eigen::Matrix<double, 15, 15>::Identity()};
    Eigen::Matrix<double, 15, 15> Q_{Eigen::Matrix<double, 15, 15>::Identity()};
};

}  // namespace rc26_localization
