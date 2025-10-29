#ifndef ACTION_H
#define ACTION_H

#include <vector>

enum MotionMode {MANUAL, WANDER, FARFROMWALLS};
enum MovingDirection {STOP, FRONT, BACK, LEFT, RIGHT, AUTO};

typedef struct
{
    MotionMode mode;
    MovingDirection direction;
} MotionControl;

class Action
{
public:
    Action();
    
    void manualRobotMotion(MovingDirection direction);
    void avoidObstacles(std::vector<float> lasers, std::vector<float> sonars);
    void keepAsFarthestAsPossibleFromWalls(std::vector<float> lasers, std::vector<float> sonars);

    MotionControl handlePressedKey(char key);

    void correctVelocitiesIfInvalid();
    float getLinearVelocity();
    float getAngularVelocity();

    bool obstaclesFront(std::vector<float> lasers);
    bool obstaclesLeft(std::vector<float> lasers);
    bool obstaclesRight(std::vector<float> lasers);

private:
    float linVel;
    float angVel;
    float cte_integral = 0.0;
    float prev_cte = 0.0;
};

#endif // ACTION_H
