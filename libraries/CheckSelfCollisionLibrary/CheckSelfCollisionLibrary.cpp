// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CheckSelfCollisionLibrary.hpp"


namespace roboticslab
{

    bool CheckSelfCollision::jointsInsideBounds(const KDL::JntArray& q){

        if(q.rows()!= qmin.rows() || q.rows()!=qmax.rows()){
            throw std::runtime_error("q size is not equal to qmin and qmax size");
        }
        printf("joints inside bounds");
        for(int i=0; i<qmax.rows(); i++){
            printf("qmax(%d): %f\n", i,qmax(i));
        }
        for(int i = 0; i<q.rows(); i++){
            printf("%f %f %f \n", q(i), qmin(i), qmax(i));
            if(q(i)< qmin(i) || q(i)>qmax(i)){
                return false;
            }
        }
        return true;
    }
/************************************************************************/

/************************************************************************/

/************************************************************************/

} // namespace roboticslab
