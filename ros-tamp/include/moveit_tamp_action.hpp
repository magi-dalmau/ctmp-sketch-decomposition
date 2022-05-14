#pragma once
#include <action.hpp>
class MoveitTampAction : public Action {
public:
  enum Type { PICK, PLACE, MOVE_BASE };
  Type type_;
  MoveitTampAction(Type type) : Action(), type_(type){};
  virtual Action *Clone() const { return new MoveitTampAction(type_); };
  void print(std::ostream &os) const override {
    switch (type_) {
    case PICK:
      os << "PICK";
      break;
    case PLACE:
      os << "PLACE";
      break;
    case MOVE_BASE:
      os << "MOVE_BASE";
      break;
    default:
      break;
    }
  }
};