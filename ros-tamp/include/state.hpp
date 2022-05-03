#pragma once

class State {
public:
  //virtual bool operator==(const State &other_state) const = 0;
  virtual std::size_t GetHash() const = 0;
  virtual State *Clone() const = 0;
  virtual void print(std::ostream &os) const = 0;
  // Overloading << operator
  friend std::ostream &operator<<(std::ostream &os, const State &obj) {
    obj.print(os);
    return os;
  }
};
