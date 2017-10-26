/* Copyright (C) 2004-2016 MBSim Development Team
 *
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 *  
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * Lesser General Public License for more details. 
 *  
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@gmail.com
 */

#ifndef _LINEAR_ELASTIC_FUNCTION_H_
#define _LINEAR_ELASTIC_FUNCTION_H_

#include "mbsim/functions/function.h"
#include <iostream>

namespace MBSim {

  /**
   * \brief tbd
   * \author Martin Foerg
   */
  class LinearElasticFunction : public Function<fmatvec::VecV(fmatvec::VecV,fmatvec::VecV)> {
    public:
      /** 
       * \brief standard constructor
       */
      LinearElasticFunction() { }

      /** 
       * \brief constructor
       * \param stiffness matrix
       * \param damping matrix
       */
      LinearElasticFunction(const fmatvec::SymMatV &K_, const fmatvec::SymMatV &D_) : K(K_), D(D_) { }

      void init(InitStage stage);

      virtual fmatvec::VecV operator()(const fmatvec::VecV& q, const fmatvec::VecV& u) {
    	  VecV results(3,INIT,0);
    	  double d;
    	  results = K*q;
    	  d=results(0);
    	  d=results(1);
    	  d=results(2);
    	  cout<<d;
    	  return K*q + D*u; }

      void setStiffnessMatrix(const fmatvec::SymMatV &K_) { K = K_; }
      void setDampingMatrix(const fmatvec::SymMatV &D_) { D = D_; }

      void initializeUsingXML(xercesc::DOMElement *element);

    protected:
      fmatvec::SymMatV K, D;
  };

}

#endif
