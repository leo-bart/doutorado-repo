/*
    Sinusoidal Movement - sets an kinematic sine input ( position, velocity,
    acceleration )
    Copyright (C) 2013  Leonardo Baruffaldi leobart@fem.unicamp.br

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/


#ifndef SINUSOIDALMOVEMENT_H
#define SINUSOIDALMOVEMENT_H

#include "fmatvec/fmatvec.h"
#include "mbsim/kinematics.h"

class SinusoidalMovement
{

public:
  SinusoidalMovement();
  SinusoidalMovement ( double _om, double _amp );
  /**
   * Sinusoidal movement with time delay
   * @param _om Angular velocity
   * @param _amp Amplitude
   * @param _tDelay Time delay
   */
  SinusoidalMovement ( double _om, double _amp, double _tDelay );
  virtual ~SinusoidalMovement();

  /// \brief Nested class to inform position
  class Position : public MBSim::Translation
  {
  private:
    double amp;
    double om;
    double t0;
  public:
    /// \brief Constructor with inherited enclosing class
    explicit Position (const SinusoidalMovement& enc) : 
      amp(enc.amplitude), om(enc.angularSpeed), t0(enc.timeDelay) {};
    
    /// \brief Number of dofs
    int getqSize() const
    {
      return 0;
    }

    /// \brief Overloaded () operator
    virtual fmatvec::Vec3 operator() ( const fmatvec::Vec &q, const double &t, const void * =NULL )
    {
      fmatvec::Vec3 PrPK;
      PrPK ( 0 ) = 0.;
      if ( t>t0 )
        {
          PrPK ( 1 ) = -amp*sin ( om* ( t-t0 ) );
        }
      return PrPK;
    };
  };

  /// \brief Nested class to inform velocity
  class Velocity : public MBSim::Function1<fmatvec::Vec3, double>
  {
  private:
    double amp;
    double om;
    double t0;
  public:
    /// \brief Constructor with inherited enclosing class
    explicit Velocity (const SinusoidalMovement& enc) :
      amp(enc.amplitude), om(enc.angularSpeed), t0(enc.timeDelay) {};
    
    /// \brief Overloaded () operator
    fmatvec::Vec3 operator() ( const double& t, const void* )
    {
      fmatvec::Vec3 j;
      j ( 0 ) = 0.;
      if ( t>t0 )
        {
          j ( 1 ) = -amp*cos ( om* ( t-t0 ) ) * om;
        }
      return j;
    }
  };

  /// \brief Nested class to inform acceleration
  class Acceleration : public MBSim::Function1<fmatvec::Vec3, double>
  {
  private:
    double amp;
    double om;
    double t0;
  public:
    /// \brief Contructor inherited from enclosing class
    explicit Acceleration(const SinusoidalMovement& enc) 
      : amp(enc.amplitude), om(enc.angularSpeed), t0(enc.timeDelay) {};
    
    /// \brief Overloaded () operator
    fmatvec::Vec3 operator() ( const double& t, const void* )
    {
      fmatvec::Vec3 dj;
      dj ( 0 ) = 0.;
      if ( t>t0 )
        {
          dj ( 1 ) = amp*sin ( om* ( t-t0 ) ) * om * om;
        }
      return dj;
    }
  };

private:
  double amplitude;
  double angularSpeed; // in rad/time
  double timeDelay;

};

#endif // SINUSOIDALMOVEMENT_H
