#ifndef KALMANFILTER_HPP__
#define KALMANFILTER_HPP__

#include "Matrix.hpp"
#include <vector>

namespace KalmanFilter {
    template <typename T, size_t M>
    Matrix<T, M, 1> getCalculatedMu(const Matrix<T, M, 1> prevMu, const Matrix<T, M, 1> update, const Matrix<T, M, M> A, const Matrix<T, M, M> B);

    template <typename T, size_t M, size_t N>
    Matrix<T, M, N> getCalculatedSigma(const Matrix<T, 2, M> A, const Matrix<T, M, N> prevSigma, bool hasConnection);

    template <typename T, size_t M, size_t N>
    Matrix<T, M, N> getKalmanGain(const Matrix<T, M, N> calculatedSigma, const Matrix<T, M, N> sensorCovarianceMatrix, bool hasConnection);

    template <typename T, size_t M, size_t N>
    Matrix<T, M, N> calculateSigma(const Matrix<T, M, N> calculatedSigma, const Matrix<T, M, N> kalmanGain);

    template <typename T, size_t M, size_t N>
    Matrix<T, M, 1> calculateMu(const Matrix<T, M, 1> calculatedMu, const Matrix<T, M, N> kalmanGain, const Matrix<T, M, 1> measurement);

    template <typename T>
    void performKalman(Matrix<T, 2, 2> A, Matrix<T, 2, 2> B, Matrix<T, 2, 1> update, Matrix<T, 2, 2> sigma, Matrix<T, 2, 2> sensorCovariance, Matrix<T, 2, 1> measurements);

    template <typename T, size_t size>
    Matrix<T, size, size> getCovarianceMatrix(std::array<T, size> variables, bool hasConnection);
}

#include "KalmanFilter.inc"

#endif // KALMANFILTER_HPP__