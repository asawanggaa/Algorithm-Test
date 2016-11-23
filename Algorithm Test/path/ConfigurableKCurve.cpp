//
//  ConfigurableKCurve.cpp
//  I think smooth term and stretch term is necessary for stable stroke, and inner fixed points are not necessary
//
//  Created by ning on 4/7/16.
//

#include "ConfigurablePath.h"
#include "Eigen/Dense"

namespace PredictedPath
{

// -----------------------------------------------------------------
Kcurve::Kcurve (const Samples& inputSamples, float segSize, float smoothness) : ConfigurablePath(inputSamples, segSize), smooth_weight(smoothness)
{
    // compute init shape by fitting, for kcurve it's also solve
    fitShape();
    
    // set up pivots from the input samples
    buildPivots();
}

double Kcurve::fitShape()
{
    double error = 0.0;
    
    int n = (int)m_samples.size();
    
    // incase too few samples
    if(n<6)
        return LARGE_DOUBLE;
    
    Eigen::MatrixXd x(n,2), b(n,2), x0(n,2); //matrix version
    
    // build weights for this smooth
    std::vector<double> wt(n, 1.0);

    // set x, b
    for(int i=0; i<n; i++)
    {
        x0(i,0) = m_samples[i].x;
        x0(i,1) = m_samples[i].y;
        b(i,0) = 0;
        b(i,1) = 0;
    }
    
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n,n);
    //    SparseMatrix<double> A(n,n);
    //    A.reserve(VectorXd::Constant(n, 7));

    // smooth energy sum{delta(k)^2}
    // first 3 rows
    
    
    // inner rows
    for(int I=0; I<n; I++)
    {
        if(I == 0){
            A(I, 0) = smooth_weight * ( 2 * wt[I+1]);
            A(I, 1) = smooth_weight * (-6 * wt[I+1]);
            A(I, 2) = smooth_weight * ( 6 * wt[I+1]);
            A(I, 3) = smooth_weight * (-2 * wt[I+1]);
            continue;
        }
        
        if(I == 1){
            A(I, 0) = smooth_weight * (-6 * wt[I]              );
            A(I, 1) = smooth_weight * (18 * wt[I] + 2 * wt[I+1]);
            A(I, 2) = smooth_weight * (-18* wt[I] - 6 * wt[I+1]);
            A(I, 3) = smooth_weight * (6  * wt[I] + 6 * wt[I+1]);
            A(I, 4) = smooth_weight * (           - 2 * wt[I+1]);
            continue;
        }
        
        if(I == 2){
            A(I, 0) = smooth_weight * (6  * wt[I-1]);
            A(I, 1) = smooth_weight * (-18* wt[I-1] -  6 * wt[I] );
            A(I, 2) = smooth_weight * (18 * wt[I-1] + 18 * wt[I] + 2 * wt[I+1]);
            A(I, 3) = smooth_weight * (-6 * wt[I-1] - 18 * wt[I] - 6 * wt[I+1]);
            A(I, 4) = smooth_weight * (             +  6 * wt[I] + 6 * wt[I+1]);
            A(I, 5) = smooth_weight * (                          - 2 * wt[I+1]);
            continue;
        }
        
        if(I == n-3){
            A(I, I-3) = smooth_weight * ( -2 * wt[I-2]);
            A(I, I-2) = smooth_weight * (  6 * wt[I-2] +  6 * wt[I-1]);
            A(I, I-1) = smooth_weight * ( -6 * wt[I-2] - 18 * wt[I-1] -  6 * wt[I]);
            A(I,  I ) = smooth_weight * (  2 * wt[I-2] + 18 * wt[I-1] + 18 * wt[I]);
            A(I, I+1) = smooth_weight * (              -  6 * wt[I-1] - 18 * wt[I]);
            A(I, I+2) = smooth_weight * (                             +  6 * wt[I]);
            continue;
        }
        
        if(I == n-2){
            A(I, I-3) = smooth_weight * (  -2 * wt[I-2]);
            A(I, I-2) = smooth_weight * (   6 * wt[I-2] +  6 * wt[I-1]);
            A(I, I-1) = smooth_weight * (  -6 * wt[I-2] - 18 * wt[I-1]);
            A(I,  I ) = smooth_weight * (   2 * wt[I-2] + 18 * wt[I-1]);
            A(I, I+1) = smooth_weight * (               -  6 * wt[I-1]);
            continue;
        }
        
        if(I == n-1){
            A(I, I-3) = smooth_weight * ( -2 * wt[I-2]);
            A(I, I-2) = smooth_weight * (  6 * wt[I-2]);
            A(I, I-1) = smooth_weight * ( -6 * wt[I-2]);
            A(I,  I ) = smooth_weight * (  2 * wt[I-2]);
            continue;
        }
        A(I, I-3) = smooth_weight * (-2 * wt[I-2]);
        A(I, I-2) = smooth_weight * ( 6 * wt[I-2] +  6 * wt[I-1]);
        A(I, I-1) = smooth_weight * (-6 * wt[I-2] - 18 * wt[I-1] -  6 * wt[I]);
        A(I, I  ) = smooth_weight * ( 2 * wt[I-2] + 18 * wt[I-1] + 18 * wt[I] + 2 * wt[I+1]);
        A(I, I+1) = smooth_weight * (             -  6 * wt[I-1] - 18 * wt[I] - 6 * wt[I+1]);
        A(I, I+2) = smooth_weight * (                            +  6 * wt[I] + 6 * wt[I+1]);
        A(I, I+3) = smooth_weight * (                                         - 2 * wt[I+1]);
    }
    
    // add relative position of two adjacient points
    Eigen::MatrixXd R0(n, 2);
    for(int i = 0; i < n; i++)
    {
        if(i == 0)
        {
            R0(i, 0) = 0;
            R0(i, 1) = 0;
        }
        else
        {
            R0(i, 0) = m_samples[i].x - m_samples[i-1].x;
            R0(i, 1) = m_samples[i].y - m_samples[i-1].y;
        }
    }
    
    int I;
    // add to lhs A
    I = 0;
    A(I, 0) += length_weight * ( 2 * wt[I+1]);
    A(I, 1) += length_weight * (-2 * wt[I+1]);
    
    for(I = 1; I < n-1; I++)
    {
        A(I, I-1) += length_weight * ( -2 * wt[I]);
        A(I, I)   += length_weight * (  2 * wt[I] + 2 * wt[I+1]);
        A(I, I+1) += length_weight * (            - 2 * wt[I+1]);
    }
    
    I = n-1;
    A(I, n-2) += length_weight * ( -2 * wt[I]);
    A(I, n-1) += length_weight * (  2 * wt[I]);
//
    // add to rhs b
    for(int k = 0; k < 2; k++)
    {
        b(0, k) += -2 * R0(1, k) * length_weight * wt[1];
        
        for(int i = 1; i < n-1; i++)
            b(i, k) += (2 * R0(i, k) * wt[i] - 2 * R0(i+1, k) * wt[i+1]) * length_weight;
        
        b(n-1, k) += 2 * R0(n-1, k) * length_weight * wt[n-1];
    }
	
    // finally set the pin points, we choose a pin point automatically every PIN_SPACING points
    std::vector<int> pinList;
    
    // test only: only pin end points and drag point
    pinList.push_back(0);
    for(int i=1;i<n-1;i++){
        if(m_samples[i].bound==true){
            pinList.push_back(i);
        }
    }
    pinList.push_back(n-1);

//    if(nDragIdx != -1)
//        pinList.push_back(nDragIdx);
    
    // enforce pinned position
    for(int k=0; k<pinList.size(); k++)
    {
        int j = pinList[k];
        
        // set b_i = b_i - A_i_j*pj
        for(int i=0; i<n; i++)
        {
            if(i!=j)
            {
                b(i,0) = b(i,0) - A.coeffRef(i,j)*m_samples[j].x;
                b(i,1) = b(i,1) - A.coeffRef(i,j)*m_samples[j].y;
                A.coeffRef(i, j) = 0;
            }
        }
        
        // clear the j row
        for(int i=0; i<n; i++)
        {
            if(i!=j)
                A.coeffRef(j, i) = 0;
            else
                A.coeffRef(j, i) = 1;
        }
        
        // set expected pin value
        b(j, 0) = m_samples[j].x;
        b(j, 1) = m_samples[j].y;
        
    }
    
    // Single threaded CPU version
    b = b - A*x0;
    x = (A.transpose() * A).ldlt().solve(A.transpose()*b) + x0;
    
    // error by naive position displacement
    for(int i=0; i<n; i++)
        error += (m_samples[i].x - x(i,0)) * (m_samples[i].x - x(i,0)) + (m_samples[i].y - x(i,1)) * (m_samples[i].y - x(i,1));

    // update result
    for(int i=0; i<n; i++)
    {
        m_samples[i].x = x(i,0);
        m_samples[i].y = x(i,1);
    }
    
    m_error = error / m_samples.size();
//    std::cout << "kcurve error " << m_error << std::endl;
    
    return m_error;
}

void Kcurve::buildPivots()
{
}

double Kcurve::projectSamples()
{
    return LARGE_DOUBLE;
}

bool Kcurve::isOnPath(float x, float y)
{
    return false;
}

int Kcurve::getNearestPivot(float x, float y)
{
    return -1;
}

} // end namespace
