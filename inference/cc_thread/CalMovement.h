
#define k 2
#define NUM_VELOC 5
#define TIME_INTERVAL 1.5
#define BASIC 0.8
#define ACTION_BASIC 0.05
#include <algorithm>
#include <vector>
#include <Eigen/Core>
#include "HumanAndBody.h"
#include "HumanMovementInfo.h"
#include <map>

enum CtrlSiganlType {Still, Nod, Rotate, ArmMove, LegUp};

class CalMovementForSegPlay
{
public:
    CalMovementForSegPlay(float image_w, float image_h, float area_ratio=0.8)
            m_num_velocs(NUM_VELOC),
            m_part_interv(NUM_VELOC),
            m_part_buff_size(k*NUM_VELOC),
            m_num_frames(k*NUM_VELOC),
            m_last_movetime(0),// = 0
            m_width(image_w),// = image_w
            m_heigh(image_h),// = image_h
            m_left_boder(image_w/2 - image_w*area_ratio/2),// = image_w/2 - image_w*area_ratio/2
            m_right_boder(image_w/2 + image_w*area_ratio/2)// = image_w/2 + image_w*area_ratio/2
    {
    };

    void calSegPlayState(const std::vector<float>& avg_veloc_list, 
                        const std::vector<float>& inst_veloc_list, 
                        const std::vector<int>& humans_trk_id, 
                        std::vector<int>& humanPlayList, 
                        std::vector<int>& inst_humanPlayList, 
                        std::vector<float>& trk_veloc_list)
    {
        if(humans_trk_id.size())
        {
            int maxTrkId = maxValOfVec<int>(humans_trk_id);
            std::vector<int> tmp1(maxTrkId+1,0);
            humanPlayList = tmp1;
            inst_humanPlayList = tmp1;
            std::vector<float> tmp2(maxTrkId+1,0);
            trk_veloc_list = tmp2;
        }
        for(int i=0; i<humans_trk_id.size(); ++i)
        {
            trk_id = humans_trk_id[i];
            if(trk_id != -1)
            {
                trk_veloc_list[trk_id] = avg_veloc_list[i];
                if(avg_veloc_list[i]>BASIC)
                    humanPlayList[trk_id] = 1;
                if(inst_veloc_list[i]>BASIC)
                    inst_humanPlayList[trk_id] = 1;
            }
        }
    };

    void calMovementSpeeds(float time_stamp, std::vector<Human>& humans, std::vector<int>& humans_trk_id, int frame_rate=3,
            std::vector<float>& avg_veloc_list,
            std::map<int std::map<string, float>>& veloc_dict_dict,
            std::vector<float>& rhythm_list,
            std::vector<float>& inst_veloc_list)
    {   /*
            # avg_veloc_list: for seg play
            # veloc_dict_dict: for mario go & jump
            # rhythm_list:(0,1)vector, for free rhythm play
        */
        for(int i=0; i<humans.size(); ++i)
        {
            Human human = humans[i];
            int trk_id = humans_trk_id[i];
            if(m_human_info_objs.find(trk_id) == m_human_info_objs.end())
            {
                m_human_info_objs[trk_id] = HumanMovementInfo(trk_id, m_part_buff_size, m_part_interv); 
            }
            HumanMovementInfo& infoObj = m_human_info_objs[trk_id];

            infoObj.updateHumanSize(human, RShoulder,LShoulder);
            bool isjump = infoObj.calPartMovement(human, m_frame_counter);
            float veloc = infoObj.calUniqueVeloc();
            infoObj.checkNod(m_frame_counter, human);
            infoObj.checkRotation(m_frame_counter, human);
            infoObj.checkLegUp(m_frame_counter, human);
            float avg_veloc = veloc;
            if(veloc > BASIC)  //# valid, add veloc, get avg, else fast stop
            {  
                m_last_movetime = time_stamp;
                avg_veloc = infoObj.updateVelocAndGetAvg(m_frame_counter, veloc);
            }
            else if(veloc >= 0.0)//# invalid, clear
            {     
                if(time_stamp - m_last_movetime > TIME_INTERVAL || veloc < 0.1)//# really stop? 
                {
                    infoObj.clearVelocBuff();
                }    
                else
                {
                    avg_veloc = BASIC+0.1;
                }
            }
            infoObj.setFinalCurVeloc(avg_veloc);
            rhythm_list.push_back(infoObj.checkAtRhythm());
            inst_veloc_list.push_back(infoObj.inst_veloc);
            avg_veloc_list.push_back(avg_veloc); 
            std::map tmpMap;
            tmpMap['go'] = avg_veloc;
            tmpMap['jump'] = isjump;
            veloc_dict_dict[trk_id]= tmpMap;
        }//# humans' veloc ok
        std::vector<int> remove_list;

        for(std::map<int, HumanMovementInfo>::iterator it=m_human_info_objs.begin(); it!=m_human_info_objs.end(); ++it)
        {
            int trk_id = iterator->first;
            if (!isInVec<int>(trk_id, humans_trk_id))
                remove_list.append(trk_id);
        }
        for each(trk_id in remove_list)
        {
            m_human_info_objs.erase(trk_id);
        }
        m_frame_counter = (m_frame_counter+1)%m_num_frames;
    };

    void judgeContrlAndSendPortInfo(const std::map<int, HumanMovementInfo>& human_info_objs, CtrlSiganlType& ctrlSig, float& sx, float&sy)
    {
        std::vector<float> Nod_veloc_list;
        std::vector<float> Rotate_veloc_list;
        std::vector<float> ArmMove_velo_list;
        std::vector<Eigen::Vector2f> ArmMove_velo_xy_list;
        std::vector<float> LegUp_velo_list;
        std::vector<Eigen::Vector2f> LegUp_velo_xy_list;
        std::map<int, HumanMovementInfo>::const_iterator it = human_info_objs.begin();
        for(it; it!=human_info_objs.end(); ++it)
        {
            if (checkInArea((it->second).bbx))
            {
                if ((it->second).veloc > BASIC)
                {
                    float v, dx, dy;
                    (it->second).getArmVeloc(v, dx, dy);
                    if(v > BASIC)
                    {
                        ArmMove_velo_list.push_back(v);
                        Eigen::Vector2f dxy(dx, dy);
                        ArmMove_velo_xy_list.push_back(dxy);
                    }
                    (it->second).getLegUpveloc(v, dx, dy);
                    if(v > BASIC)
                    {
                        LegUp_velo_list.push_back(v);
                        dxy<<dx, dy;
                        LegUp_velo_xy_list.push_back(dxy);
                    }
                    if ((it->second).Nod)
                    {
                        Eigen::Vector2f dxy = (it->second).part_inst_veloc_xy[Nose];
                        if(dxy(1) !=0)
                            Nod_veloc_list.push_back(dxy(1));
                    }
          
                    if((it->second).rotate != Still)
                    {
                        float dx = (it->second).getRotateVeloc();
                        if(dx != 0)
                            Rotate_veloc_list.push_back(dx);
                    }

                }
            }

        }
        getUniqueCtrlSignal(ArmMove_velo_list, 
                            ArmMove_velo_xy_list, 
                            LegUp_velo_list, 
                            LegUp_velo_xy_list, 
                            Nod_veloc_list, 
                            Rotate_veloc_list,
                            ctrlSig, sx, sy);
    };//ok

    void getUniqueCtrlSignal(std::vector<float>& Nod_veloc_list,
                            std::vector<float>& Rotate_veloc_list,
                            std::vector<float>& ArmMove_velo_list,
                            std::vector<Eigen::Vector2f>& ArmMove_velo_xy_list,
                            std::vector<float>& LegUp_velo_list,
                            std::vector<Eigen::Vector2f>& LegUp_velo_xy_list,
                            CtrlSiganlType& ctrlSig, float& sx, float&sy)
    {
        sx = 0,sy = 0;
        ctrlSig = Still;
        if(Nod_veloc_list.size() && Rotate_veloc_list.size())
        {    
            std::vector<float> tmpv1;
            absVec<float>(Nod_veloc_list, tmpv1);
            float m1 = maxValOfVec<float>(tmpv1);
            std::vector<float> tmpv2;
            absVec<float>(Rotate_veloc_list, tmpv2);
            float m2 = maxValOfVec<float>(tmpv2);
            if(m1 > m2)//#nod
            {    
                ctrlSig = Nod;
                getCtrlVeloc(ctrlSig, Nod_veloc_list, sx,sy);
            }
            else//#rotate
            {    
                ctrlSig = Rotate;
                getCtrlVeloc(ctrlSig, Rotate_veloc_list, sx,sy);
            }
        }
        else
        {
            if (Nod_veloc_list.size())//#nod
            {    
                ctrlSig = Nod;
                getCtrlVeloc(ctrlSig, Nod_veloc_list, sx,sy);
            }
            else
            {
                if (Rotate_veloc_list.size())//#rotate
                {
                    ctrlSig = Rotate;
                    getCtrlVeloc(ctrlSig, Rotate_veloc_list, sx,sy);
                }
            }
        }
        if(ctrlSig == Still)
        {
            if (LegUp_velo_list.size())
            {
                ctrlSig = LegUp;
                int idx = argmax<float>(LegUp_velo_list);
                Eigen::Vector2f sxy = LegUp_velo_xy_list[idx];
                velocScale(sxy, 5, 0.9);
                sx = sxy(0);
                sy = sxy(1);
            }
        }
        if(ctrlSig == Still)
        {
            if (ArmMove_velo_list.size())
            {
                ctrlSig = ArmMove;
                int idx = argmax<float>(ArmMove_velo_list);
                Eigen::Vector2f sxy = ArmMove_velo_xy_list[idx];
                velocScale(sxy, 5, 0.9);
                sx = sxy(0);
                sy = sxy(1);
            }
        }
    };

    void getCtrlVeloc(CtrlSiganlType ctrlSig, std::vector<float>& veloc_list, float& sx, float& sy):
    {
        sx=0,sy= 0;

        std::vector<float> tmpv1;
        absVec<float>(veloc_list, tmpv1);
        int idx = argmax<float>(tmpv1);
        float veloc = veloc_list[idx]; 
        if(ctrlSig == Nod) 
        {
            Eigen::Vector2f sxy(0,veloc);
            velocScale(sxy, 10, 0.9);
            sx = sxy(0);
            sy = sxy(1);
        }
        if(ctrlSig == Rotate)
        {
            Eigen::Vector2f sxy(veloc, 0);
            velocScale(sxy, 5, 0.9);
            sx = sxy(0);
            sy = sxy(1);
        }
    }

    void velocScale(Eigen::Vector2f& dxy, float scale1, float scale2)
    {
        float t_x = scale1*dxy(0);
        float y_x = scale1*dxy(1);
        scaleVal(t_x, scale2);
        scaleVal(t_y, scale2);
        dxy(0)=t_x;
        dxy(1)=t_y;
    };
    
    void scaleVal(float& x, float scale=0.9)
    {
        if(abs(x)>1.0)
            x = scale*(x/abs(x));
        x = int(x*127/2 + 127/2);    
    };

    bool checkInArea(Eigen::Vector4f& bbx)
    {
        bool inarea = false;
        x = int(bbx(0) + bbx(2)/2.0);
        y = int(bbx(1) + bbx(3)/2.0);   
        if(x>m_left_boder && x<m_right_boder)
            inarea = true;
        return inarea;
    };

    std::map<int, HumanMovementInfo> m_human_info_objs;
    int m_frame_counter;
    int m_num_velocs; //= 5 #3 frames,2 velocs, =frame_rate
    int m_part_interv;// = self.num_velocs
    int m_part_buff_size;// = self.num_velocs * k
    int m_num_frames;// = self.num_velocs * k

    float m_last_movetime;// = 0

    float m_width;// = image_w
    float m_heigh;// = image_h
    float m_left_boder;// = image_w/2 - image_w*area_ratio/2
    float m_right_boder;// = image_w/2 + image_w*area_ratio/2
};
   

        
   




