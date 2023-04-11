#ifndef PYTORCH_MOTION_PLANNER_POSITION_H
#define PYTORCH_MOTION_PLANNER_POSITION_H


class Position {
public:
    double x;
    double y;
    double angle;
    Position(double x, double y, double angle) : x(x), y(y), angle(angle){};
};


#endif //PYTORCH_MOTION_PLANNER_POSITION_H
