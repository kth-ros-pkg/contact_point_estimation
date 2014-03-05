/*
 *  SurfaceNormalEstimatorParams.cpp
 *
 *
 *  Created on: Jan 14, 2014
 *  Authors:   Francisco Viña
 *            fevb <at> kth.se
 */

/* Copyright (c) 2014, Francisco Viña, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <contact_point_estimation/SurfaceNormalEstimatorParams.h>

SurfaceNormalEstimatorParams::SurfaceNormalEstimatorParams()
{

}

SurfaceNormalEstimatorParams::~SurfaceNormalEstimatorParams()
{

}

double SurfaceNormalEstimatorParams::getGammaN() const
{
	return m_gamma_n;
}

void SurfaceNormalEstimatorParams::setGammaN(double gamma_n)
{
	m_gamma_n = gamma_n;
}

double SurfaceNormalEstimatorParams::getBetaN() const
{
	return m_beta_n;
}

void SurfaceNormalEstimatorParams::setBetaN(double beta_n)
{
	m_beta_n = beta_n;
}

Vector3d SurfaceNormalEstimatorParams::getInitialN() const
{
	return m_initial_n;
}

void SurfaceNormalEstimatorParams::setInitialN(const Vector3d &initial_n)
{
	m_initial_n = initial_n.normalized();
}

double SurfaceNormalEstimatorParams::getUpdateFrequency() const
{
    return m_update_frequency;
}

void SurfaceNormalEstimatorParams::setUpdateFrequency(double update_frequency)
{
    m_update_frequency = update_frequency;
}
