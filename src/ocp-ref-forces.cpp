///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022 LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "sobec/fwd.hpp"
#include "sobec/ocp-walk.hpp"

#include "pinocchio/spatial/force.hpp"

namespace sobec {

  Eigen::MatrixXd computeWeightShareSmoothProfile(const Eigen::Ref<const Eigen::MatrixX2d> contact_pattern,
                                       int duration,
                                       double robotGravityForce)
  {
    Eigen::Index T = contact_pattern.cols(); // time horizon length
    Eigen::Index nc = contact_pattern.rows(); // number of contact
    
    Eigen::MatrixXd contactImportance(nc,T);

    // Contact importance is set at a fair discontinuous share between all the active contacts.
    for( Eigen::Index t=0; t<T; ++T )
      {
        double nbactiv = contact_pattern.col(t).sum();
        contactImportance.col(t) = contact_pattern.col(t)*robotGravityForce/nbactiv;
      }



    for( Eigen::Index t=1; t<T; ++T )
      {
        bool landing = false;
        for( Eigen::Index k=0; k<nc; ++k )
          {
            if((!contact_pattern(k,t-1))&&contact_pattern(k,t)) landing=true;
          }
        if(landing)
          for( Eigen::Index k=0; k<nc; ++k )
            {
              for(Eigen::Index s=0;s<duration;++s)
                contactImportance(t+s,k) = contactImportance(t-1,k)*(1-s/double(duration))
                  + contactImportance(t,k)*(s/double(duration));
            }
      }
    for( Eigen::Index t=1; t<T; ++T )
      {
        for( Eigen::Index k=0; k<nc; ++k )
          {
          }
      }
    
    return contactImportance;
  }

  
  void computeReferenceForces(int duration, double robotGravityForce)
  {
    // Eigen::Ref<const MatrixX2d> contact_pattern;
    // int T = contact_pattern.cols(); // time horizon length
    // int nc = contact_pattern.rows(); // number of contact

    // std::vector< std::vector<pinocchio::Force> > referenceForces;
    // referenceForces.resize(T);

    // const pinocchio::Force & grav = pinocchio::Force::Zero();
    // grav.linear()[2] = robotGravityForce;

    // contact importance = 1/nb_active_contact


 

      //       for( int t=1; t<T; ++T )
      // {
      //   for( int k=0; k<nc; ++k )
      //     {
      //       if((!contact_pattern[k,t-1] ) && contact_pattern[k,t] )
      //         {
      //           // Contact creation for k at t
      //           for(int s=0;s<duration;++s)
      //             {
                    

                    
      //             }
                
                
      //         }

            
      //     }


        
      // }



    
  }
  


  
} // namespace sobec
