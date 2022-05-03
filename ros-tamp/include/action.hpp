#pragma once
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