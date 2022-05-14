<<<<<<< HEAD
#pragma once
#include <iostream>
class Action {
public:
  virtual Action *Clone() const = 0;
  virtual void print(std::ostream &os) const = 0;
  // Overloading << operator
  friend std::ostream &operator<<(std::ostream &os, const Action &obj) {
    obj.print(os);
    return os;
  }
};
=======
class Action;
>>>>>>> parent of 2676092... Implemented and tested Lazy Brfs
