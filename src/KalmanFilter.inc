namespace KalmanFilter {
    template <typename T, size_t M>
    Matrix<T, M, 1> getCalculatedMu(const Matrix<T, M, 1> prevMu, const Matrix<T, M, 1> update, const Matrix<T, M, M> A, const Matrix<T, M, M> B) {
        return A * prevMu + B * update;
    }

    template <typename T, size_t M, size_t N>
    Matrix<T, M, N> getCalculatedSigma(const Matrix<T, 2, M> A, const Matrix<T, M, N> prevSigma, bool hasConnection) {
        Matrix<T, M, N> result =  A * prevSigma * A.transpose();

        if (!hasConnection) {
            size_t curColumn = N - 1;
            for (size_t row = 0; row < M; ++row) {
                result[row][curColumn] = 0;
                --curColumn;
            }
        }

        return result;
    }

    template <typename T, size_t M, size_t N>
    Matrix<T, M, N> getKalmanGain(const Matrix<T, M, N> calculatedSigma, const Matrix<T, M, N> sensorCovarianceMatrix, bool hasConnection) {
        const Matrix <T, M, N> C = calculatedSigma.identity();
        Matrix<T, M, N> result = calculatedSigma * C.transpose();
        const Matrix<T, M, N> secondResult = C * calculatedSigma * C.transpose() + sensorCovarianceMatrix;


        for (size_t row = 0; row < M; ++row) {
            for (size_t column = 0; column < N; ++column) {
                if (result[row][column] != 0 && secondResult[row][column] != 0) {
                    result[row][column] = result[row][column] / secondResult[row][column];
                }
            }
        }

        if (!hasConnection) {
            size_t curColumn = N - 1;
            for (size_t row = 0; row < M; ++row) {
                result[row][curColumn] = 0;
                --curColumn;
            }
        }

        return result;
    }

    template <typename T, size_t M, size_t N>
    Matrix<T, M, N> calculateSigma(const Matrix<T, M, N> calculatedSigma, const Matrix<T, M, N> kalmanGain) {
        Matrix<T, M, N> C = kalmanGain.identity();
        return (calculatedSigma.identity() - kalmanGain * C) * calculatedSigma;
    }

    template <typename T, size_t M, size_t N>
    Matrix<T, M, 1> calculateMu(const Matrix<T, M, 1> calculatedMu, const Matrix<T, M, N> kalmanGain, const Matrix<T, M, 1> measurement) {
        Matrix<T, M, N> C = kalmanGain.identity();
        return calculatedMu + kalmanGain * (measurement - C * calculatedMu);
    }

    template <typename T>
    void performKalman(Matrix<T, 2, 2> A, Matrix<T, 2, 2> B, Matrix<T, 2, 1> update, Matrix<T, 2, 2> sigma, Matrix<T, 2, 2> sensorCovariance, Matrix<T, 2, 1> measurements) {
        Matrix<double, 2, 1> mu = measurements;
        Matrix<double, 2, 1> calculatedMu = getCalculatedMu(mu, update, A, B);
        Matrix<double, 2, 2> calculatedSigma = getCalculatedSigma(A, sigma, false);
        Matrix<double, 2, 2> kalmanGain = getKalmanGain(calculatedSigma, sensorCovariance, false);
        sigma = calculateSigma(calculatedSigma, kalmanGain);
        mu = calculateMu(calculatedMu, kalmanGain, measurements);
        std::cout << mu.to_string() << std::endl;
    }

    template <typename T, size_t size>
    Matrix<T, size, size> getCovarianceMatrix(std::array<T, size> variables, bool hasConnection) {
        Matrix<T, size, size> result;

        for (size_t row = 0; row < size; ++row) {
            for (size_t column = 0; column < size; ++column) {

                result[row][column] = variables[row] * variables[column];
            }
        }

        if (!hasConnection) {
            size_t curColumn = size - 1;
            for (size_t row = 0; row < size; ++row) {
                result[row][curColumn] = 0;
                --curColumn;
            }
        }

        return result;
    }

}