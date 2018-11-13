#ifndef __HUMANANDBODY_H_
#define __HUMANANDBODY_H_

#include <vector>

enum CocoPart {
    Nose = 0,
    Neck = 1,
    RShoulder = 2,
    RElbow = 3,
    RWrist = 4,
    LShoulder = 5,
    LElbow = 6,
    LWrist = 7,
    RHip = 8,
    RKnee = 9,
    RAnkle = 10,
    LHip = 11,
    LKnee = 12,
    LAnkle = 13,
    REye = 14,
    LEye = 15,
    REar = 16,
    LEar = 17,
    Background = 18
    };

/*
part_idx : part index(eg. 0 for nose)
x, y: coordinate of body part
score : confidence score
*/
class BodyPart
{
public:
    BodyPart()        
        :part_idx(-1),
        x(-1),
        y(-1),
        score(-1){};

    BodyPart(int part_idx, float x, float y, float score)
        :part_idx(part_idx),
        x(x),
        y(y),
        score(score)
    {
    };

    int part_idx;
    float x;
    float y;
    float score;
};

class Human
{
    /*
    body_parts: list of BodyPart
    */
public:
    Human():score(0.0){
        std::vector<BodyPart> tmpv(18, BodyPart());//no Background
        body_parts = tmpv;
    };
public:
    std::vector<BodyPart> body_parts;
    float score;
};
#endif

