#include "Action.h"

#include "Utils.h"

bool Action::obstaclesFront(std::vector<float> lasers){
    if(lasers[90] < 2.0 || lasers[91] < 2.0 || lasers[89] < 2.0){
        return true;
    }
    return false;
}
bool Action::obstaclesLeft(std::vector<float> lasers){ 
    for(int i = 80; i >= 60; i--){
        if(lasers[i] < 1.5){
            return true;
        }
    }
    return false;
}

bool Action::obstaclesRight(std::vector<float> lasers){
    for(int i = 100; i <= 120; i++){
        if(lasers[i] < 1.5){
            return true;
        }
    }
    return false;
}

Action::Action()
{
    linVel = 1.0;
    angVel = 0.5;
}

void Action::avoidObstacles(std::vector<float> lasers, std::vector<float> sonars)
{
    // 0
    //    90
    // 180
    // ang vel pos -> sentido anti horario
    if(obstaclesLeft(lasers)){
        linVel= 0.0; 
        angVel= -0.3;
    }else if(obstaclesRight(lasers)){
        linVel= 0.0; 
        angVel= 0.3;
    }else if(obstaclesFront(lasers)){
        linVel= 0.0; 
        angVel= 0.3;
    }else{
        linVel= 0.5; angVel= 0.0;
    }

}

void Action::keepAsFarthestAsPossibleFromWalls(std::vector<float> lasers, std::vector<float> sonars)
{
    float cte;
    float tp = 0.4;
    float ti = 0.001;
    float td = 5;
    float min_front_distance = 1.5;
    float max_cte = 1.5;
    float left_distance = std::min(lasers[0], 2.2f);
    float right_distance = std::min(lasers[180], 2.2f);
    float front_distance = lasers[90];
    // pos -> 0 é maior, vira sentido horario (neg)

    if(front_distance > min_front_distance){
        cte = right_distance - left_distance; //cte posi -> esq maior, vira sentido anti horario
        linVel = 3.0;

        if(cte > max_cte) cte = max_cte;
        if(cte < -max_cte) cte = -max_cte;

        cte_integral += cte;
        if (cte_integral > 5.0) cte_integral = 5.0;
        if (cte_integral < -5.0) cte_integral = -5.0;
        float derivative = cte - prev_cte;
        prev_cte = cte;
        /*
        std::cout << "tp: " << tp << std::endl;
        std::cout << "ti: " << ti << std::endl;
        std::cout << "td: " << td << std::endl;
        std::cout << "cte: " << cte << std::endl;
        std::cout << "integral: " << cte_integral << std::endl;
        std::cout << "derivative: " << derivative << std::endl;
        */
        std::cout << "right: " << right_distance << std::endl;
        std::cout << "left: " << left_distance << std::endl;
        angVel = -tp * cte - td * derivative - ti * cte_integral;
    }else{
        if(left_distance < right_distance){
            angVel = -1.0;
        }else{
            angVel = 1.0;
        }
    }

}

void Action::manualRobotMotion(MovingDirection direction)
{
    if(direction == FRONT){
        linVel= 0.5; angVel= 0.0;
    }else if(direction == BACK){
        linVel=-0.5; angVel= 0.0;
    }else if(direction == LEFT){
        linVel= 0.0; angVel= 0.5;
    }else if(direction == RIGHT){
        linVel= 0.0; angVel=-0.5;
    }else if(direction == STOP){
        linVel= 0.0; angVel= 0.0;
    }
}

void Action::correctVelocitiesIfInvalid()
{
    float b=0.38;

    float leftVel  = linVel - angVel*b/(2.0);
    float rightVel = linVel + angVel*b/(2.0);

    float VELMAX = 0.5;

    float absLeft = fabs(leftVel);
    float absRight = fabs(rightVel);

    if(absLeft>absRight){
        if(absLeft > VELMAX){
            leftVel *= VELMAX/absLeft;
            rightVel *= VELMAX/absLeft;
        }
    }else{
        if(absRight > VELMAX){
            leftVel *= VELMAX/absRight;
            rightVel *= VELMAX/absRight;
        }
    }
    
    linVel = (leftVel + rightVel)/2.0;
    angVel = (rightVel - leftVel)/b;
}

float Action::getLinearVelocity()
{
    return linVel;
}

float Action::getAngularVelocity()
{
    return angVel;
}

MotionControl Action::handlePressedKey(char key)
{
    MotionControl mc;
    mc.mode=MANUAL;
    mc.direction=STOP;

    if(key=='1'){
        mc.mode=MANUAL;
        mc.direction=STOP;
    }else if(key=='2'){
        mc.mode=WANDER;
        mc.direction=AUTO;
    }else if(key=='3'){
        mc.mode=FARFROMWALLS;
        mc.direction=AUTO;
    }else if(key=='w' or key=='W'){
        mc.mode=MANUAL;
        mc.direction = FRONT;
    }else if(key=='s' or key=='S'){
        mc.mode=MANUAL;
        mc.direction = BACK;
    }else if(key=='a' or key=='A'){
        mc.mode=MANUAL;
        mc.direction = LEFT;
    }else if(key=='d' or key=='D'){
        mc.mode=MANUAL;
        mc.direction = RIGHT;
    }else if(key==' '){
        mc.mode=MANUAL;
        mc.direction = STOP;
    }
    
    return mc;
}
