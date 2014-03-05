/*
 *  ContactPointEstimatorParams.cpp
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


#include <contact_point_estimation/ContactPointEstimatorParams.h>

ContactPointEstimatorParams::ContactPointEstimatorParams()
{
	m_kappa_r = 0.0;

}

ContactPointEstimatorParams::~ContactPointEstimatorParams()
{

}

double ContactPointEstimatorParams::getGammaR() const
{
	return m_gamma_r;
}

void ContactPointEstimatorParams::setGammaR(double gamma_r)
{
	m_gamma_r = gamma_r;
}

double ContactPointEstimatorParams::getKappaR() const {
	return m_kappa_r;
}

void ContactPointEstimatorParams::setKappaR(double kappa_r) {
	m_kappa_r = kappa_r;
}


double ContactPointEstimatorParams::getBetaR() const
{
	return m_beta_r;
}

void ContactPointEstimatorParams::setBetaR(double beta_r)
{
	m_beta_r = beta_r;
}


Vector3d ContactPointEstimatorParams::getInitialR() const
{
	return m_initial_r;
}

void ContactPointEstimatorParams::setInitialR(const Vector3d &initial_r)
{
	m_initial_r = initial_r;
}

double ContactPointEstimatorParams::getUpdateFrequency() const
{
    return m_update_frequency;
}

void ContactPointEstimatorParams::setUpdateFrequency(double update_frequency)
{
    m_update_frequency = update_frequency;
}
