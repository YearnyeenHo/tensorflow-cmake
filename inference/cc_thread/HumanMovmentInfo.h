#ifndef __CALMOVEMENT_H_
#define __CALMOVEMENT_H_

#include <algorithm>
#include <vector>
#include <Eigen>
#include "HumanAndBody.h"

#define ROUND_NUM = 3
#define NUM_HUMAN = 4
#define DELAY = 0.01
#define ACTION_BASIC = 0.05

#define STATIC = 0.1
#define BASIC = 0.8
#define EXTREME = 5
#define FAST = BASIC+(EXTREME-BASIC)/3.0
#define ACCELERATE = BASIC+2*(EXTREME-BASIC)/3.0
#define SPEED_INTERVAL = ACCELERATE - BASIC
#define REALTIME_FRAMERATE = 30.0

enum Rotation {Still, Left, Right};
class HumanMovmentInfo:
{
public:
    HumanMovmentInfo(int human_id, int buf_len, int interv):m_id(human_id),
                                                    m_buf_len(buf_len),
                                                    m_interv(interv),
                                                    m_fast(FAST),
                                                    m_size(0),
                                                    m_presize(0),
                                                    m_nod_ratio(0),
                                                    m_pre_nod_ratio(0),
                                                    m_sizeChange(false),
                                                    m_isjump(false),
                                                    m_rotate(Still), 
                                                    m_Nod(false),
                                                    m_Legup(false),
                                                    m_veloc(0)
    { 

        

        //=== buffer =======
        Eigen::Vector3f tmpV3f(-1,-1,-1);
        std::vector<Eigen::Vector3f> tmpV0(m_buf_len, tmpV3f);
        std::vector<std::vector<Eigen::Vector3f>> tmpV1(18, tmpV0);//={} Predecessor bodypart location （x,y,z), z is no use if not run lifting
        m_prebodypart = tmpV1;
        std::vector<float> tmpV2(m_buf_len, -1);//=[]//previous unique veloces of a human, with size of m_buf_len
        m_veloc_buff = tmpV2;
        std::vector<float> tmpV3(18, 0);
        m_part_inst_veloc = tmpV;// = {}// Predecessor instance veloc
        //==== current human veloc info ====
        Eigen::Vector2f tmp(0,0);
        std::vector<Eigen::Vector2f> tmpV4(m_part_list.size(), tmp);
        m_part_inst_veloc_xy = tmpV4;

        bbx<<-1,-1,-1,-1;// = -1
        std::vector<int> tmpV={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};
        m_part_list = tmpV;// = range(18)
    };

    bool checkJump(float dy)
    {
        m_isjump = false;
        if (dy < - ACTION_BASIC) //moving up
            m_isjump = true;
        return m_isjump;
    };

    void getPreviousBodyParts(int frame_id, int part_id, float& x, float& y,float& z, float& xp, float& yp, float& zp)
    {
        Eigen::Vector3f pos;
        pos = m_prebodypart[part_id][(frame_id + m_buf_len - m_interv)%m_buf_len];
        x = pos(1), y=pos(2),z=pos(2);
        pos = m_prebodypart[part_id][(frame_id + m_buf_len - m_interv/2)%m_buf_len];
        xp = pos(1), yp=pos(2),zp=pos(2);
    };

    Rotation cal2dRotation(const Eiegn::Vector2f& c_v, const Eiegn::Vector2f& l_v, float d_x)
    {
        float Lcv = sqrt(c_v.dot(c_v));
        float Llv = sqrt(l_v.dot(l_v));
        if(Lcv-Llv<-ACTION_BASIC)//# rotation
        {
            if(d_x>ACTION_BASIC)//#left
                return Left;
            if(d_x<-ACTION_BASIC)//#righ   
                return Right;
        }
    };

    bool isHasPrePart(int part_id)
    {
        if(m_prebodypart[part_id](0) != -1)
            return true;
        else
            return false;
    };

    void checkRotation(int frame_id, Human& human)
    {
        m_rotate = Still;
        if(human.hasBodyPart(LShoulder) && human.hasBodyPart(RShoulder))
        {   
            if(isHasPrePart(LShoulder) && isHasPrePart(RShoulder))
            {
                float x1,y1,x2,y2,x,y,z,xp1,yp1,zp1,xp2,yp2,zp2;
                getPreviousBodyParts(frame_id, LShoulder, x,y,z, xp1,yp1,zp1);
                getPreviousBodyParts(frame_id, RShoulder, x,y,z, xp2,yp2,zp2); 
                x1 = human.body_parts[LShoulder].x;
                y1 = human.body_parts[LShoulder].y; 
                x2 = human.body_parts[RShoulder].x;
                y2 = human.body_parts[RShoulder].y; 
                Eiegn::Vector2f c_v(x1-x2, y1-y2);
                Eiegn::Vector2f l_v(xp1-xp2, yp1-yp2);
                if(human.hasBodyPart(Nose))
                {
                    Eigen::Vector2f xy = m_part_inst_veloc_xy[Nose];
                    m_rotate = cal2dRotation(c_v, l_v, xy(0));
                }
            }

        }
    };

    float scaleBySize(float val)
    {
        if(m_size>0)
            val/=m_size;
        return val;        
    };

    void calMovement(float c_x, float c_y, float l_x, float l_y, float& dist, float& dx, float& dy)
    {
        float d_x = c_x-l_x;
        float d_y = c_y-l_y;
        float dis = abs(d_x)+ abs(d_y);
        dist = scaleBySize(dis);
        dx = scaleBySize(d_x);
        dy = scaleBySize(d_y);
    };

    void checkNod(int frame_id, Human human)
    {
        m_Nod = False;
        if(human.hasBodyPart(Nose) && human.hasBodyPart(Neck))
        {
            Eigen::Vector2f d_xy1 = m_part_inst_veloc_xy[Nose];
            Eigen::Vector2f d_xy2 = m_part_inst_veloc_xy[Neck];
            float x1,x2,y1,y2;
            x1 = human.body_parts[Nose].x;
            y1 = human.body_parts[Nose].y; 
            x2 = human.body_parts[Neck].x;
            y2 = human.body_parts[Neck].y;
            float dist, dx, dy;
            calMovement(x1,y1,x2,y2, dist, dx, dy);
            m_nod_ratio = dist;
            if(d_y1 > ACTION_BASIC*1.5 && abs(d_y2) < ACTION_BASIC && abs(d_x1) < ACTION_BASIC) 
            {   
                if(m_nod_ratio < m_pre_nod_ratio)
                {
                    m_Nod = true;
                }

            } 
            m_pre_nod_ratio = m_nod_ratio;
        } 
    };

    void checkLegUp(int frame_id, Human human)
    {    
        std::vector<int> leg_parts = {RKnee, RAnkle, LKnee, LAnkle};
        m_Legup = False;
        if (!m_sizeChange)
        {
            for(int i=0; i<leg_parts.size(); ++i)
            {
                part_id = leg_parts[i];
                if(human.hasBodyPart(part_id))
                {
                    Eigen::Vector2f dxy = m_part_inst_veloc_xy[part_id];
                    if(abs(dxy(1)) < ACTION_BASIC*2.0)
                    {
                        m_Legup = true;
                        break;   
                    }
                }
            } 
        }
    }

    void setFinalCurVeloc(float veloc)
    {
        m_veloc = veloc;
    };

    void updateHumanSize(Human human, int keyp1, int keyp2)
    {
        m_bbx = human.bbx;
        m_sizeChange = false;
        float sz = 0;
        if(human.hasBodyPart(keyp1) && human.hasBodyPart(keyp2))
        {
            float x1,x2,y1,y2;
            x1 = human.body_parts[keyp1];
            y1 = human.body_parts[keyp1];
            x2 = human.body_parts[keyp2];
            y2 = human.body_parts[keyp2];
            float dis = float(sqrt(pow(x1-x2,2)+pow(y1-y2,2)));
            if(m_size==-1)
            {
                m_size = dis;
                m_presize = dis;
            }
            else
            {
                m_presize = m_size;
                m_size = (m_size + dis)/2.0;
            }
            if(abs(m_size - m_presize) > 0.1*m_size)
            {
                 m_sizeChange = true;
            }   
               
        }
    };

    template<typename T>
    T maxValOfVec(std::vector<T>& vec)
    {
        std::vector<T> tmp(vec);
        std::sort (tmp.begin(), tmp.end());
        return tmp[tmp.size()-1]; 
    };
    template<typename T>
    T meanVec(std::vector<T>& vec)
    {
        T mean = 0;
        for each(i in vec)
        {
            mean+=i;
        }
        mean/=vec.size();
    };
    template<typename T>
    T argmax(std::vector<T>& vec)
    {
        T maxv;
        int idx=0;
        for(int i=0; i<vec.size(); ++i)
        {
            if(vec[i]>maxv)
            {
                idx = i;
                maxv =vec[i];
            }
        }
        return idx;
    };
    template<typename T>
    void absVec(std::vector<T>& vec, std::vector<T>& des_vec)
    {
        for each(item in vec)
        {
            des_vec.push_back(-item);
        }
    };

    float calUniqueVeloc()
    {
        m_veloc = maxValOfVec<float>(m_part_veloc); 
        if(m_veloc==0)
            m_veloc = -1;

        m_inst_veloc = maxValOfVec<float>(m_part_inst_veloc);
        if(m_inst_veloc==0)
            m_inst_veloc = -1;

        return m_veloc;
    };

    void getArmVeloc(float& v, float& dx, float& dy)
    {
        std::vector<int>arm_list = {RShoulder,RElbow,RWrist,LShoulder,LWrist,LElbow};
        getLimbVeloc(arm_list, v, dx, dy);
    };
    void getLegUpveloc(float& v, float& dx, float& dy)
    {
        std::vector<int>leg_list = {RKnee, RAnkle, LKnee, LAnkle};
        getLimbVeloc(leg_list, v, dx, dy);
    };
    void getLimbVeloc(std::vector<int>&limb_list, float& v, float& dx, float& dy)
    { 
        v=0, dx=0, dy = 0;
        std::vector<float> v_list(maxValOfVec<int>(limb_list)+1), 0);
        for each(auto part_id in limb_list)
        {
            v_list[part_id] = m_part_veloc[part_id];
        }
        std::vector<float> absTmp;
        int part_id = argmax<float>(absVec<float>(v_list, absTmp));
        Eigen::Vector2f dxy = m_part_inst_veloc_xy[part_id]; 
        v = v_list[part_id];
        dx = dxy(0), dy = dxy(1);
    };

    void getRotateVeloc(float& dx)
    {        
        Eigen::Vector2f dxy0 = m_part_inst_veloc_xy[Nose];
        Eigen::Vector2f dxy1(0,0);
        if(m_rotate == Left)
        {
            dxy1 = m_part_inst_veloc_xy[RShoulder];   
        }
        else if(m_rotate == Right)
        
            dxy1 = m_part_inst_veloc_xy[LShoulder];
        }
        if(abs(dxy0(0))>abs(dxy1(0)))
        {
            dx = dxy0(0);
        }
        else
            dx = dxy1(0);
    };

    void clearVelocBuff()
    {
        std::vector<float> tmp(m_buf_len, -1);
        m_veloc_buff = tmp;
    };

    void removePrePart(int part_id)
    {
        Eiegn::Vector3f tmp1(-1,-1, -1);
        std::vector<Eiegn::Vector3f> tmpv(m_buf_len, tmp1);
        m_prebodypart[part_id] = tmpv;
        Eiegn::Vector2f tmp2(0,0);
        m_part_inst_veloc[part_id] = tmp2;
    };

    void updatePrePartLocation(int frame_id, int part_id, Eigen::Vector3f& location)
    {
        if(m_prebodypart[part_id][0](0) == -1)
        {
            for(int i=0; i<m_buf_len; ++i)
                m_prebodypart[part_id][i] = location;
        }
        else
            m_prebodypart[part_id][frame_id%m_buf_len] = location;
    };


    float updateVelocAndGetAvg(int frame_id, float veloc)
    {
        if(m_veloc_buff[0]!=-1)
        {
            veloc_buff[frame_id%m_buf_len] = veloc;
        }
        else
        {
            //#init buffer
            for(int i=0; i<m_buf_len; ++i)
            {
                m_veloc_buff[i] = veloc;
            }
        }
        float avg_veloc = meanVec<float>(m_veloc_buff);
        if(veloc>=FAST)
            avg_veloc = veloc;        
        return avg_veloc;
    };

    void reinit()
    {
        Eigen::Vector2f tmp(0,0);
        std::vector<Eigen::Vector2f> tmpV1(m_part_list.size(), tmp);
        m_part_inst_veloc_xy = tmpV1;

        m_rhythmPoint_list.clear();
        
        m_isjump = false;
        
        std::vector<float> tmpV3(m_part_list.size(), -1);
        m_part_veloc = tmpV3;
    };

    void movementRhythm(int part_id, float inst_veloc)
    {
        if(m_part_inst_veloc[part_id]>BASIC && inst_veloc<BASIC)
            m_rhythmPoint_list.push_back(1);
        m_part_inst_veloc[part_id] = inst_veloc;
    };

    bool calPartMovement(Human human, int frame_id)
    {
        reinit();
        for each(part_id in m_part_list)
        {
            if(human.hasBodyPart(part_id))
            {
                float x,y,z;
                human.getPartLocation(part_id, x,y,z);
                if(isHasPrePart(part_id))
                {
                    float l_x,l_y,l_z,p_x,p_y,p_z;
                    getPreviousBodyParts(frame_id, part_id, l_x,l_y,l_z,p_x,p_y,p_z);
                    float macr_v, macr_dx, macr_dy, local_v, local_dx, local_dy;
                    calMovement(x,y,l_x,l_y, macr_v, macr_dx, macr_dy);
                    calMovement(x,y,p_x,p_y, local_v, local_dx, local_dy);
                    movementRhythm(part_id, local_v);
                    Eigen::Vector2f tmp(local_dx, local_dy);
                    m_part_inst_veloc_xy[part_id] = tmp;
                    dy = 0
                    if(local_v < BASIC || local_v > FAST*1.5)
                    {
                        m_part_veloc[part_id] = local_v;
                        dy = local_dy;
                    }
                    else
                    {
                        m_part_veloc[part_id] = macr_v;
                        dy = macr_dy;
                    }
                    if (!m_sizeChange && (part_id == RShoulder || part_id == LShoulder))
                        m_isjump = checkJump(dy);
                }
                Eigen::Vector3f pos(x,y,z);
                updatePrePartLocation(frame_id, part_id, pos);
            }
            else// no detected part
                removePrePart(part_id);
        }
        return m_isjump;
    };

    bool checkAtRhythm()
    {
        if(m_rhythmPoint_list.size())
            return true;
        else
            return false;
    };

    int m_id;// = human_id
    int m_buf_len;// =m_buf_len
    int m_interv;// = m_interv
    float m_fast;// = FAST
    //=== buffer =======
    std::vector<Eigen::Vector3f> m_prebodypart;//={} Predecessor bodypart location （x,y,z), z is no use if not run lifting
    std::vector<float> m_veloc_buff;//=[]//previous unique veloces of a human, with size of m_buf_len
    std::vector<float> m_part_inst_veloc;// = {}// Predecessor instance veloc
    //==== current human veloc info ====
    std::vector<Eigen::Vector3f> m_part_veloc;// = {} // current veloc of each part
    std::vector<Eigen::Vector2f> m_part_inst_veloc_xy;// = {}  current veloc x and y of each part

    //=== human current states ===
    std::vector<float> m_rhythmPoint_list;// = []
    float m_size;// = 0
    float m_presize;// = 0
    float m_nod_ratio;// = 0
    float m_pre_nod_ratio;// = 0
    Eigen::Vector4f bbx;// = -1
    bool m_sizeChange;// = False
    bool m_isjump;// = False
    Rotation m_rotate;// =  m_Rotate.Still.value
    bool m_Nod;// = False
    bool m_Legup;
    float m_veloc;// = 0
    std::vector<int> m_part_list;// = range(18)
};
#endif
