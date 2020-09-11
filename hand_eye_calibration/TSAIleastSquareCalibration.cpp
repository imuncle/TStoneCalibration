#include "TSAIleastSquareCalibration.h"

TSAIleastSquareCalibration::TSAIleastSquareCalibration()
{

}

void TSAIleastSquareCalibration::crossprod(cv::Mat a, cv::Mat &ax)
{
    ax = cv::Mat::zeros(cv::Size(3, 3), CV_64F);
    if (a.rows == 3 && a.cols == 1) {
        ax.at<double>(0, 1) = -a.at<double>(2, 0);
        ax.at<double>(0, 2) =  a.at<double>(1, 0);

        ax.at<double>(1, 0) =  a.at<double>(2, 0);
        ax.at<double>(1, 2) = -a.at<double>(0, 0);

        ax.at<double>(2, 0) = -a.at<double>(1, 0);
        ax.at<double>(2, 1) =  a.at<double>(0, 0);
    }
}

void TSAIleastSquareCalibration::runCalibration(
        std::vector<cv::Mat> eHa,
        std::vector<cv::Mat> cHb,
        int mode, // 1 for eye in hand, 0 for eye to hand
        cv::Mat &extrinsicMatrix)
{
    cv::Mat invM, rgij, rcij, rngij, rncij, Pgij, Pcij, Pskew, invA;
    std::vector<cv::Mat> Hgij;
    std::vector<cv::Mat> Hcij;
    int n = cHb.size();
    cv::Mat A = cv::Mat::zeros(cv::Size(3, 3 * (n - 1)), CV_64F);
    cv::Mat b = cv::Mat::zeros(cv::Size(1, 3 * (n - 1)), CV_64F);
    // we have n-1 linearly independent relations between the views
    for (int i = 1; i < n; ++ i) {
        // Transformations between views
        if (mode == 1) {
            invert(eHa[i], invM);
            Hgij.push_back(invM * eHa[i-1]);
        } else {
            invert(eHa[i-1], invM);
            Hgij.push_back(eHa[i] * invM);
        }
        invert(cHb[i-1], invM);
        Hcij.push_back(cHb[i] * invM);
        // turn it into angle axis representation (rodrigues formula: P is
        // the eigenvector of the rotation matrix with eigenvalue 1
        cv::Rodrigues(Hgij[i-1](cv::Range(0, 3), cv::Range(0, 3)), rgij);
        cv::Rodrigues(Hcij[i-1](cv::Range(0, 3), cv::Range(0, 3)), rcij);
        double theta_gij = norm(rgij);
        double theta_cij = norm(rcij);
        rngij = rgij / theta_gij;
        rncij = rcij / theta_cij;
        // Tsai uses a modified version of this
        Pgij = 2 * sin(theta_gij / 2) * rngij;
        Pcij = 2 * sin(theta_cij / 2) * rncij;
        // Now we know that
        // skew(Pgij+Pcij)*x = Pcij-Pgij  which is equivalent to Ax = b
        // So we need to construct vector b and matrix A to solve this
        // overdetermined system. (Note we need >=3 Views to have at least 2
        // linearly independent inter-view relations.
        crossprod(Pgij + Pcij, Pskew);
        for (int r = 0; r < 3; ++ r) {
            for (int c = 0; c < 3; ++ c) {
                A.at<double>(3 * i - 3 + r, c) = Pskew.at<double>(r, c);
            }
            b.at<double>(3 * i - 3 + r, 0) = Pcij.at<double>(r, 0) - Pgij.at<double>(r, 0);
        }
    }

    // Computing Rotation
    invert(A, invA, cv::DECOMP_SVD);
    cv::Mat Pcg_prime = invA * b;
    // Computing residus
    cv::Mat err = A * Pcg_prime - b;
    double residus_TSAI_rotation = sqrt(norm(err) / (n - 1));
    cv::Mat Pcg = 2 * Pcg_prime / (sqrt(1 + norm(Pcg_prime) * norm(Pcg_prime)));
    double norm_Pcg = norm(Pcg);
    crossprod(Pcg, Pskew);
    cv::Mat Rcg = (1 - norm_Pcg * norm_Pcg / 2) * cv::Mat::eye(3, 3, CV_64F) + 0.5 * (Pcg * Pcg.t() + sqrt(4 - norm_Pcg * norm_Pcg) * Pskew);

    // Computing Translation
    cv::Mat I3 = cv::Mat::eye(3, 3, CV_64F);
    for (int i = 1; i < n; ++ i)
    {
        for (int r = 0; r < 3; ++ r) {
            for (int c = 0; c < 3; ++ c) {
                A.at<double>(3 * i - 3 + r, c) = Hgij[i-1].at<double>(r, c) - I3.at<double>(r, c);
            }
            Pcij.at<double>(r, 0) = Hcij[i-1].at<double>(r, 3);
            Pgij.at<double>(r, 0) = Hgij[i-1].at<double>(r, 3);
        }
        cv::Mat Pp = Rcg * Pcij - Pgij;
        for (int r = 0; r < 3; ++ r) {
            b.at<double>(3 * i - 3 + r, 0) = Pp.at<double>(r, 0);
        }
    }
    invert(A, invA, cv::DECOMP_SVD);
    cv::Mat Tcg = invA * b;
    // Computing residus
    err = A * Tcg - b;
    double residus_TSAI_translation = sqrt(norm(err)/(n-1));
    // Estimated transformation

    extrinsicMatrix = cv::Mat::zeros(cv::Size(4, 4), CV_64F);
    for (int r = 0; r < 3; ++ r) {
        for (int c = 0; c < 3; ++ c) {
            extrinsicMatrix.at<double>(r, c) = Rcg.at<double>(r, c);
        }
        extrinsicMatrix.at<double>(r, 3) = Tcg.at<double>(r, 0);
    }
    extrinsicMatrix.at<double>(3, 3) = 1;
}
