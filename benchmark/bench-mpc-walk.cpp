#include <sobec/fwd.hpp>
#include <iostream>
#include <sobec/py2cpp.hpp>
#include <sobec/mpc-walk.hpp>

int main()
{
  using namespace sobec;
  using namespace crocoddyl;
  std::cout << "*** Benchmark start ***" << std::endl;
  boost::shared_ptr<sobec::MPCWalk> mpc
    = sobec::initMPCWalk(PROJECT_SOURCE_DIR
                         "/benchmark/mpc_description.py");
  
  

  Eigen::VectorXd x = mpc->problem->get_x0();
  for( int t=1;t<=100;t++ )
    {
      mpc->calc(x,t);
      x = mpc->solver->get_xs()[1];
    }
  
  
}
