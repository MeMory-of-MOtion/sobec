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
  

  
}
