#ifndef __sobec_py2cpp__
#define __sobec_py2cpp__

#include <boost/smart_ptr.hpp>
#include <crocoddyl/core/fwd.hpp>

namespace sobec {
typedef boost::shared_ptr<crocoddyl::ShootingProblem> ShootingProblemPtr;

ShootingProblemPtr initShootingProblem(const char* fileName);

}  // namespace sobec

#endif  // #ifndef __sobec_py2cpp__
