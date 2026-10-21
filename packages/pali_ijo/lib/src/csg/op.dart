/// Boolean operation, Add, subtract, or intersect.
enum OpType {
  add,
  subtract,
  intersect,
}

extension OpTypeX on OpType {
  static OpType fromInt(int op) {
    switch (op) {
      case 1:
        return OpType.subtract;
      case 2:
        return OpType.intersect;
      default:
        return OpType.add;
    }
  }

  int get asInt {
    switch (this) {
      case OpType.subtract:
        return 1;
      case OpType.intersect:
        return 2;
      case OpType.add:
        return 0;
    }
  }
}
