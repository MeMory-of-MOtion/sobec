#include <cassert>

#include "sobec/gepadd.hpp"

int main() {
  assert(sobec::add(1, 2) == 3);
  assert(sobec::add(5, -1) == 4);
  assert(sobec::add(-3, -1) == -4);
  return 0;
}
