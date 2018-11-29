class Rotate(Enum):
    Left = 1
    Right = 2
    Still = 0
class CtrlSiganlType(Enum):
    Still = 0
    Nod = 1
    Rotate = 2
    ArmMove = 3

#fname = "btv.mid"
NUM_HUMAN = 4

STATIC = 0.1
BASIC = 0.6
EXTREME = 5
FAST = BASIC+(EXTREME-BASIC)/3.0
ACCELERATE = BASIC+2*(EXTREME-BASIC)/3.0
SPEED_INTERVAL = ACCELERATE - BASIC
REALTIME_FRAMERATE = 30.0
RShoulder = 2
RElbow = 3
RWrist = 4
LShoulder = 5
LElbow = 6
LWrist = 7
Nose = 0
Neck = 1
class HumanMovmentInfo(object):
    def __init__(self, human_id, buf_len, interv): 
        self.id = human_id
        self.buf_len =buf_len
        self.interv = interv
        self.fast = FAST
        #=== buffer =======
        self.prebodypart={}# Predecessor bodypart location （x,y,z), z is no use if not run lifting
        self.veloc_buff=[]#previous unique veloces of a human, with size of buf_len
        self.part_inst_veloc = {}# Predecessor instance veloc
        #==== current human veloc info ====
        self.part_veloc = {} # current veloc of each part
        self.part_inst_veloc_xy = {} # current veloc x and y of each part

        #=== human current states ===
        self.rhythmPoint_list = []
        self.size = 0
        self.bbx = -1
        self.sizeChange = False
        self.isjump = False
        self.rotate =  Rotate.Still.value
        self.Nod = False
        self.veloc = 0
        self.part_list = range(18)
    
    def checkJump(self, dy):
        isjump = 0
        if dy < - 0.06:# moving up
            isjump = 1
        return isjump
    
    def checkRotation(self, frame_id, human):
        LShoulder, RShoulder
        self.rotate = Rotate.Still.value
        if LShoulder in human.body_parts and RShoulder in human.body_parts:
            x1,y1,z1 = human.body_parts[LShoulder].x, human.body_parts[LShoulder].y, human.body_parts[LShoulder].z
            x2,y2,z2 = human.body_parts[RShoulder].x, human.body_parts[RShoulder].y, human.body_parts[RShoulder].z
            if self.isHasPrePart(LShoulder) and self.isHasPrePart(RShoulder):
                _, _, _, xp1,yp1,zp1 = self.getPreviousBodyParts(frame_id, LShoulder)
                _, _, _,xp2,yp2,zp2 = self.getPreviousBodyParts(frame_id, RShoulder)  
                c_v = np.array([x1-x2, y1-y2, z1-z2])
                l_v = np.array([xp1-xp2, yp1-yp2, zp1-zp2])
                if Nose in human.body_parts and Nose in self.part_inst_veloc_xy:
                    d_x,_ = self.part_inst_veloc_xy[Nose]
                    c_v = np.array([x1-x2, y1-y2])
                    l_v = np.array([xp1-xp2, yp1-yp2])
                    self.rotate = self.cal2dRotation(c_v, l_v, d_x)
                if -1 not in [z1,z2,zp1,zp2]:
                    self.rotate = self.cal3dRotation(c_v, l_v)
                
    def checkNod(self, frame_id, human):
        Nose = 0
        Neck = 1
        self.Nod = False
        if Nose in human.body_parts and Neck in human.body_parts:
            x1,y1,z1 = human.body_parts[Nose].x, human.body_parts[Nose].y, human.body_parts[Nose].z
            x2,y2,z2 = human.body_parts[Neck].x, human.body_parts[Neck].y, human.body_parts[Neck].z
            if Nose in human.body_parts and Nose in self.part_inst_veloc_xy and Neck in human.body_parts and Neck in self.part_inst_veloc_xy:
                _, d_y1 = self.part_inst_veloc_xy[Nose]
                _, d_y2 = self.part_inst_veloc_xy[Nose]
                if  d_y1 < -0.06 and np.abs(d_y2) < 0.06:
                    self.Nod = True
                    print('NODNODNODNODNODNODNODNODNODNODNODNODNOD,NODNODNODNOD')

            self.updatePrePartLocation(frame_id, Nose, (x1, y1, z1))
            self.updatePrePartLocation(frame_id, Neck, (x2, y2, z2))

    def isHasPrePart(self, part_id):
        if part_id in self.prebodypart:
            return True
        else:
            return False
    
    def getArmVeloc(self):
        dx, dy = 0
        arm_list = [RShoulder,RElbow,RWrist,LShoulder,LWrist,LElbow]
        v_list = []
        for part_id in arm_list:
            if part_id in self.part_veloc:
                v_list.append(self.part_veloc[part_id])
        if len(v_list):
            idx = np.argmax(np.abs(v_list))
            part_id = arm_list[idx]
            dx,dy = self.part_inst_veloc_xy[part_id]
        return v_list[idx], dx, dy
    
    def getRotateVeloc(self):
        dx = 0
        if self.rotate == Rotate.Left.value:
            dx0, _ = self.part_inst_veloc_xy[Nose]
            dx1, _ = self.part_inst_veloc_xy[RShoulder]
            dx = dx0 if np.abs(dx0)>np.abs(dx1) else dx1
        else:
            if self.rotate == Rotate.Right.value:
                dx0, _ = self.part_inst_veloc_xy[Nose]
                dx1, _ = self.part_inst_veloc_xy[LShoulder]
                dx = dx0 if np.abs(dx0)>np.abs(dx1) else dx1
        return dx
    
    def getVectorAngle(self, v1,v2):
        cos_val = c_v.dot(l_v)
        rad = np.arccos(cos_val)
        angle = (rad/np.pi)*180
        return angle, cos_val

    def leftOrRight(self, cx_cos, cz_cos, lx_cos, lz_cos):
        if cz_cos>0 and (cx_cos-lx_cos<0):#right
            print("right")
            return Rotate.Right.value
        if cz_cos<0 and (cx_cos-lx_cos>0):#right
            print("right")
            return Rotate.Right.value
        if cz_cos<0 and (cx_cos-lx_cos<0):#left
            print("left")
            return Rotate.Left.value
        if cz_cos>0 and (cx_cos-lx_cos>0):#left
            print("left")
            return Rotate.Left.value

    def cal2dRotation(self, c_v, l_v, d_x):
        Lcv = np.sqrt(c_v.dot(c_v))
        Llv = np.sqrt(l_v.dot(l_v))
        if Lcv-Llv<-0.06:# rotation
            if d_x>0.06:#left
                print("Left", d_x)
                return Rotate.Left.value
            if d_x<-0.06:#righ
                print("Right", d_x)
                return Rotate.Right.value

    def cal3dRotation(self, c_v, l_v):
        rotate = Rotate.Still.value
        gravity = np.array([0,1,0])
        z = np.array([0,0,1])
        x = np.array([1,0,0])

        Lcv = np.sqrt(c_v.dot(c_v))
        Llv = np.sqrt(l_v.dot(l_v))
        c_v /= Lcv
        l_v /= Llv
        angle, _= self.getVectorAngle(n_v, gravity)
        
        n_v = np.cross(c_v, l_v)
        n_v/=np.sqrt(n_v.dot(n_v))
        _, pivot_cos= self.getVectorAngle(n_v, gravity)
        if np.abs(pivot_cos)>cos(np.pi/6) and angle>30:# valid
            _, cz_cos= self.getVectorAngle(c_v, z)
            _, cx_cos= self.getVectorAngle(c_v, x)
            _, lz_cos= self.getVectorAngle(l_v, z)
            _, lx_cos= self.getVectorAngle(l_v, x)
            rotate = self.leftOrRight(cx_cos, cz_cos, lx_cos, lz_cos)
        return rotate

    def calMovement(self, c_x, c_y, l_x, l_y):
        d_x, d_y = c_x-l_x, c_y-l_y
        dis = np.abs(d_x)+np.abs(d_y)
        veloc = float(dis)/1
        veloc = veloc/self.size if self.size>0 else veloc
        d_x = d_x/self.size if self.size>0 else d_x
        d_y = d_y/self.size if self.size>0 else d_y
        return veloc, d_x, d_y 

    def setFinalCurVeloc(self, veloc):
        self.veloc = veloc

    def calUniqueVeloc(self):
        self.veloc = max(self.part_veloc.values()) if len(self.part_veloc) else -1 # detection failure
        self.inst_veloc = max(self.part_inst_veloc.values()) if len(self.part_inst_veloc) else -1 
        return self.veloc

    def getPreviousBodyParts(self, frame_id, part_id):
        x,y,z = self.prebodypart[part_id][(frame_id + self.buf_len - self.interv)%self.buf_len]
        xp,yp,zp = self.prebodypart[part_id][(frame_id + self.buf_len - self.interv//2)%self.buf_len]
        return x,y,z, xp,yp,zp

    def updateHumanSize(self, human, keyp1, keyp2):
        self.bbx = human.bbx
        self.sizeChange = False
        sz = 0
        if keyp1 in human.body_parts and keyp2 in human.body_parts:
            body_part = human.body_parts[keyp1]
            x1,y1=body_part.x, body_part.y 
            body_part = human.body_parts[keyp2]
            x2,y2=body_part.x, body_part.y 
            dis = float(math.sqrt(pow(x1-x2,2)+pow(y1-y2,2)))
            if self.size==-1:
                sz = dis
                self.size = dis
            else:
                sz = (self.size + dis)/2.0
            if np.abs(self.size - sz) > 0.1*self.size:
                self.sizeChange = True
        self.size = sz

    def removePrePart(self, part_id):
        if part_id in self.prebodypart:
            del self.prebodypart[part_id]
        if part_id in self.part_inst_veloc:
            del self.part_inst_veloc[part_id]

    def clearVelocBuff(self):
        self.veloc_buff = []

    def updatePrePartLocation(self, frame_id, part_id, location):
        if not self.isHasPrePart(part_id):
            self.prebodypart[part_id] = []
            for i in range(self.buf_len):
                 self.prebodypart[part_id].append(location)
        else:
            self.prebodypart[part_id][frame_id%self.buf_len] = location

    def updateVelocAndGetAvg(self, frame_id, veloc):
        if len(self.veloc_buff):
            self.veloc_buff[frame_id%self.buf_len] = veloc
        else:
            #init buffer
            for i in range(self.buf_len):
                self.veloc_buff.append(veloc)
        avg_veloc = np.mean(self.veloc_buff)
        if veloc>=self.fast:
            avg_veloc = veloc        
        return avg_veloc
    
    #=========== cal movement speed ==================#
    def calPartMovement(self, human, frame_id):
        self.rhythmPoint_list = []
        self.isjump = False
        self.part_veloc = {}
        for part_id in self.part_list:
            if part_id in human.body_parts:
                body_part = human.body_parts[part_id]
                x,y,z=body_part.x, body_part.y, body_part.z 
                if self.isHasPrePart(part_id):
                    l_x,l_y, _, p_x, p_y, _ = self.getPreviousBodyParts(frame_id, part_id)
                    macr_v, macr_dx, macr_dy = self.calMovement(x, y, l_x, l_y)
                    local_v, local_dx, local_dy = self.calMovement(x, y, p_x, p_y)
                    self.movementRhythm(part_id, local_v)
                    #self.part_inst_veloc[part_id] = local_v/self.size if self.size>0 else local_v
                    self.part_inst_veloc_xy[part_id] = local_dx, local_dy
                    
                    if local_v < BASIC:
                        self.part_veloc[part_id] = local_v
                        dy = local_dy
                    else:
                        if local_v > FAST*(1.5):
                            self.part_veloc[part_id] = local_v
                            dy = local_dy
                        else:
                            self.part_veloc[part_id] = macr_v
                            dy = macr_dy
                    if not self.sizeChange and (part_id == RShoulder or part_id == LShoulder):
                        self.isjump = self.checkJump(dy)
                self.updatePrePartLocation(frame_id, part_id, (x, y, z))
            else:# no detected part
                self.removePrePart(part_id)
        return self.isjump

    #=========== cal movement Rhythm ===================#
    def movementRhythm(self, part_id, inst_veloc):
        if part_id in self.part_inst_veloc:
            if self.part_inst_veloc[part_id]>BASIC and inst_veloc<BASIC:
                self.rhythmPoint_list.append(1)
        self.part_inst_veloc[part_id] = inst_veloc

    def checkAtRhythm(self):
        if np.sum(self.rhythmPoint_list)>0:
            return 1
        else:
            return 0


class CalMovementForSegPlay(object):
    def __init__(self):
        self.human_info_objs = {}
        self.frame_counter = 0
        k=2 # whatever you like
        self.num_velocs = 5 # 3 frames,2 velocs, =frame_rate
        self.part_interv = self.num_velocs
        self.part_buff_size = self.num_velocs * k
        self.num_frames = self.num_velocs * k
        self.basic = BASIC
        self.fast = FAST
        self.last_movetime = 0
        self.maxdx = -1
        self.mindx = 1
    def calSegPlayState(self, avg_veloc_list, inst_veloc_list, humans_trk_id): 
        humanPlayList = []
        inst_humanPlayList = []
        if len(humans_trk_id):
            humanPlayList = np.zeros(max(humans_trk_id)+1)
            inst_humanPlayList = np.zeros(max(humans_trk_id)+1)
        for i in range(len(humans_trk_id)):
            trk_id = humans_trk_id[i]
            if trk_id != -1:
                if avg_veloc_list[i]>BASIC:
                    humanPlayList[trk_id] = 1
                if inst_veloc_list[i]>BASIC:
                    inst_humanPlayList[trk_id] = 1


        return humanPlayList, inst_humanPlayList

    def calMovementSpeeds(self, humans, humans_trk_id, frame_rate=3):
        avg_veloc_list = []  #########################################################
        veloc_dict_dict = {}
        rhythm_list = []
        inst_veloc_list = []
        # greneral case & sp case 1,2
        for i in range(len(humans)):
            human = humans[i]
            trk_id = humans_trk_id[i]
            if trk_id not in self.human_info_objs:
                self.human_info_objs[trk_id] = HumanMovmentInfo(trk_id, self.part_buff_size, self.part_interv) 
            #========== info obj update  ==============
            infoObj = self.human_info_objs[trk_id]
            infoObj.updateHumanSize(human, RShoulder,LShoulder)
            isjump = infoObj.calPartMovement(human, self.frame_counter)
            veloc = infoObj.calUniqueVeloc()
            infoObj.checkNod(self.frame_counter, human)
            infoObj.checkRotation(self.frame_counter, human)
            #========== set velocsity ==============
            #veloc = 1.2
            avg_veloc = veloc
            
            if veloc > self.basic:  # valid, add veloc, get avg, else fast stop
                self.last_movetime = time.time()
                avg_veloc = infoObj.updateVelocAndGetAvg(self.frame_counter, veloc)
            else: # invalid, clear
                if veloc >= 0.0:
                    if time.time() - self.last_movetime > REALTIME_FRAMERATE/frame_rate or veloc < 0.1:# really stop? 
                        if trk_id in self.human_info_objs.keys():
                            self.human_info_objs[trk_id].clearVelocBuff()
                    else:
                        avg_veloc = self.basic+0.1
            infoObj.setFinalCurVeloc(avg_veloc)
            rhythm_list.append(infoObj.checkAtRhythm())
            inst_veloc_list.append(infoObj.inst_veloc)
            self.human_info_objs[trk_id] = infoObj
            avg_veloc_list.append(avg_veloc) ######################################
            veloc_dict_dict[trk_id] = {}
            veloc_dict_dict[trk_id]['go'] = avg_veloc
            veloc_dict_dict[trk_id]['jump'] = isjump
        # humans' veloc ok
        remove_list = []
        for trk_id in self.human_info_objs:
            if trk_id not in humans_trk_id:
                remove_list.append(trk_id)
        for trk_id in remove_list:
            del self.human_info_objs[trk_id]
        self.frame_counter = (self.frame_counter+1)%self.num_frames
        # avg_veloc_list: for seg play
        # veloc_dict_dict: for mario go & jump
        # rhythm_list:(0,1)vector, for free rhythm play
        return avg_veloc_list, veloc_dict_dict, rhythm_list, inst_veloc_list
