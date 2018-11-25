import numpy as np
import fluidsynth
import mido
import time

NUM_HUMAN = 4

def trace(frame, event, arg):
    print ("%s, %s:%d" % (event, frame.f_code.co_filename, frame.f_lineno))
    return trace

def initMidiSegPlay(fname):
    mseg = MidiSegmentation_multitrack(fname)
    msp = MidiSegPlay(mseg)
    return msp

def playNext(msp, time_stamp):
    #import sys
    #sys.settrace(trace)
    play_states = [1,1,1,1]
    msp.playSegContinue(play_states, time_stamp)
    
class MidiSegPlay(object):
    def __init__(self, mseg):
        self.part_instrs = mseg.part_instrs
        self.seg_result = mseg.seg_result
        self.seg_timeDelta = mseg.seg_timeDelta
        self.track_dict = {}
        self.finger_notes = {}
        self.finger_down_times = {}
        self.playSegBuff = []

        self.cur_seg_id = 0
        self.last_playtime = 0
        self.last_validSpeedTime = 0
        self.last_basicSpeedTime = 0
        self.segEventPlayInit()

    ###################################
    def playspeedMechanism(self, basic, veloc_list, humans_trk_id):
        humanPlayList = []
        #self.changeSoundFont(self.sfid1)
        for i in range(len(veloc_list)):
            h_trk_id = -1
            if veloc_list[i]>basic:
                h_trk_id = humans_trk_id[i]
            humanPlayList.append(h_trk_id)
        veloc = max(veloc_list)
        return veloc, humanPlayList
    ###################################
    def checkDeltaTime(self, dura_factor):
        curIdx = self.cur_seg_id + len(self.playSegBuff)
        deltaFromLast = dura_factor * self.seg_timeDelta[(curIdx)%len(self.seg_result)]
        return deltaFromLast

    def playPlan(self, num_notes, dura_factor):
        basic_sleeps = []
        curidx = self.cur_seg_id + len(self.playSegBuff)
        time=0
        play_time = 0.33#1/frame_rate    
        while True:
            nextDelay = dura_factor*self.seg_timeDelta[(curidx+num_notes)%len(self.seg_result)]
            time += nextDelay
            if time<play_time:
                basic_sleeps.append(nextDelay)
                num_notes+=1
            else:
                break
        return basic_sleeps, num_notes

    def playNextSegBuffering(self, veloc_list, humans_trk_id):
        if len(veloc_list)==0:
            if time.time() - self.last_validSpeedTime > 1:
                self.playSegBuff=[]
            return
        # static 0.02 
        basic, extreme = BASIC, EXTREME
        fast, accelerate = FAST, ACCELERATE 
        num_notes = 0
        dura_factor = 1.0
        delta_factor = 1.0
        basic_sleeps = []
        deltaFromLast = 0
        # only when whlie loop total sleep not larger than 0.15s, can it stop right after user pause
        veloc, humanPlayList  = self.playspeedMechanism(basic, veloc_list, humans_trk_id) ######################
        if veloc<=basic and veloc>= 0.0:
            if time.time() - self.last_basicSpeedTime > 0.5 or veloc<0.1 :
                self.playSegBuff=[]
        else:
            if veloc > 0.0:
                num_notes = 1
                self.last_validSpeedTime = time.time()
                self.last_basicSpeedTime = time.time()
            else:# detection failure
                if time.time() - self.last_validSpeedTime > 2:
                    self.playSegBuff=[]
    
        if veloc> basic:# baisc movement
            deltaFromLast = self.checkDeltaTime(delta_factor)
            basic_sleeps, num_notes = self.playPlan(num_notes, delta_factor)
                    
        if num_notes:
            cache = deltaFromLast, dura_factor, humanPlayList # next note
            self.playSegBuff.append(cache)
            # add more note if time allow 
            for deltaFromLast in basic_sleeps:
                cache = deltaFromLast, dura_factor, humanPlayList
                self.playSegBuff.append(cache)
    
    def playBuffer(self):
        num=0
        for cache in self.playSegBuff:
            deltaFromLast, _, _  = cache
            _, _, humanPlayList = self.playSegBuff[-1]
            if deltaFromLast <= (time.time() - self.last_playtime):
                self.finger_down(self.seg_result[self.cur_seg_id], humanPlayList)
                self.cur_seg_id=(self.cur_seg_id + 1)%len(self.seg_result)
                self.last_playtime = time.time()
                num+=1
            else:
                break
        self.playSegBuff = self.playSegBuff[num:]

    def finger_down(self, seg, humanPlayList):
        for n in seg:
            track_id, chanl, pitch, onset, dura, veloc = n   
            for h_trk_id in humanPlayList:
                if h_trk_id != -1:# valid speed:
                    trk = self.getTrackHuman(track_id, chanl, h_trk_id%NUM_HUMAN)
                    self.fsyn.noteon(trk, pitch, veloc)
                    key = (trk, pitch, onset)
                    self.finger_notes[key]=n
                    self.finger_down_times[key]=time.time() 
    
    def finger_up(self):
        keys = self.finger_notes.keys()
        off_keys = []
        for k in keys:
            track_id, chanl, pitch, onset, dura, veloc = self.finger_notes[k]
            trk, pitch, onset = k
            if dura <= (time.time() - self.finger_down_times[k]):
                self.fsyn.noteoff(trk, pitch)
                off_keys.append(k)
        for k in off_keys:
            self.finger_notes.pop(k)
            self.finger_down_times.pop(k)
    
    def segEventPlayInit(self):
        self.fsyn = fluidsynth.Synth()
        self.fsyn.start(driver="coreaudio", midi_driver="coremidi")
        self.sfid1 = self.fsyn.sfload("FluidR3_GM.sf2")#"Pixels_8-bit_soundfont.sf2")
        self.sfid2 = self.fsyn.sfload("FluidR3_GM.sf2")#"VintageDreamsWaves-v2.sf2")
        self.changeSoundFont(self.sfid1 )

    def programSelect(self, trk, chanl, sfid, h_id, program=0):
        band = 0
        if chanl == 9:
            band = 128
        else:
            program = h_id*20
        self.fsyn.program_select(trk, sfid, band, program)

    def changeSoundFont(self, sfid):
        for k in self.part_instrs:
            track_id, chanl = k
            program = self.part_instrs[k]
            #self.fsyn.program_select(track, sfid, 0, program)
            for h_id in range(NUM_HUMAN):
                track = self.getTrackHuman(track_id, chanl, h_id)
                self.programSelect(track, chanl, sfid, h_id, program)

    def segEventPlayFinished(self):
        self.fsyn.delete()

    def getTrackHuman(self, track_id, chanl, human_id):
        k = (track_id, chanl, human_id)
        track = 0
        if k in self.track_dict:
            trackNum = self.track_dict[k]
        else:
            trackNum = len(self.track_dict)
            self.track_dict[k] = trackNum
        return trackNum
    
    #===================== play continue ===================#
    def playSegContinue(self, play_states, time_stamp):
        if self.seg_timeDelta[self.cur_seg_id] <= time_stamp-self.last_playtime:
            seg = self.seg_result[self.cur_seg_id]
            self.finger_down_continue(seg, play_states, time_stamp)
            self.cur_seg_id= (self.cur_seg_id + 1)%len(self.seg_result)
            self.last_playtime=time_stamp
        self.finger_up_continue(time_stamp) 

    def finger_down_continue(self, seg, play_states, time_stamp):
        for n in seg:
            track_id, chanl, pitch, onset, dura, veloc = n   
            for h_trk_id in range(len(play_states)):
                state = play_states[h_trk_id]
                if state != 0:# valid speed:
                    trk = self.getTrackHuman(track_id, chanl, h_trk_id%NUM_HUMAN)
                    self.fsyn.noteon(trk, pitch, veloc)
                    key = (trk, pitch, onset)
                    self.finger_notes[key]=n
                    self.finger_down_times[key]=time_stamp   

    def finger_up_continue(self, time_stamp):
        keys = self.finger_notes.keys()
        off_keys = []
        for k in keys:
            track_id, chanl, pitch, onset, dura, veloc = self.finger_notes[k]
            trk, pitch, onset = k
            if dura < (time_stamp - self.finger_down_times[k]):
                self.fsyn.noteoff(trk, pitch)
                off_keys.append(k)
        for k in off_keys:
            self.finger_notes.pop(k)
            self.finger_down_times.pop(k)

class MidiSegmentation_multitrack(object):
    def __init__(self, fname): 
        self.parseMidiFile(fname)
        self.midiSegMultitrack()

    def init(self):
        self.pitch_buff=[]
        self.seg_timeDelta=[0]
        self.seg_result=[]
        self.track_dict = {}
        self.last_time=0
        
    def parseMidiFile(self, fname):
        self.midobj = MidiObj(fname)
        self.midobj.flatTracks()
        self.parts = self.midobj.parts #key = (track_id, chanl) val=(pitch, onset, dura, veloc)
        self.part_instrs = self.midobj.getInstrs() # key = (track_id, chanl) val=program
   
    def midiSegMultitrack(self):
        self.init()
        print("midiSegMultitrack")
        self.last_time = 0.0
        self.last_time_candis = []
        counters = np.zeros(len(self.parts), np.int32)
        num_elem = np.sum([len(self.parts[k]) for k in self.parts])
        while np.sum(counters)<num_elem:
            # find next pitches set. look into each part score
            for id, key in enumerate(self.parts):
                part_list = self.parts[key] 
                track_id, chanl = key
                i = counters[id]
                # continue passing a score
                while i <len(part_list):
                    elem = part_list[i] #offsetMap is list
                    pitch, onset, dura, veloc = elem
                    if onset>self.last_time:
                        self.last_time_candis.append(onset)
                        break
                    self.addPitchToBuffer(track_id, chanl, pitch, onset, dura, veloc)
                    i+=1
                # one score part finish
                counters[id] = i
            # time stamp finish
            ofsd = 0
            if len(self.last_time_candis):
                ofsd = np.min(self.last_time_candis) - self.last_time
                self.last_time = np.min(self.last_time_candis)
            self.seg_timeDelta.append(ofsd)
            self.bufferFlush()
            self.last_time_candis=[]
        print("midiSegMultitrack OK")
        print(len(self.seg_result))
        
        
        self.seg_timeDelta = self.seg_timeDelta[:-1]
        if len(self.seg_timeDelta)>len(self.seg_result):
            self.seg_timeDelta = self.seg_timeDelta[1:]
        #self.seg_timeDelta = self.seg_timeDelta[0]+self.seg_timeDelta
        print(len(self.seg_timeDelta), len(self.seg_result))

    def bufferFlush(self):
        seg=[]
        for n in self.pitch_buff: 
            seg.append(n) 
        if len(seg):
            self.seg_result.append(seg)
        self.pitch_buff = []
       
    def addPitchToBuffer(self, track_id, chanl, pitch, onset, dura, veloc):
        cache = (track_id, chanl, pitch, onset, dura, veloc)
        self.pitch_buff.append(cache)
    
class MidiObj(object):
    def __init__(self, fname):
        self.mid = mido.MidiFile(fname)
        self.tempo = 0
    '''
            s0 s0 s40 e0 e0 e40
    acctime 0  0   40 40 40  80
    '''
    def tick2second(self, tick, ticks_per_beat, tempo):
        """Convert absolute time in ticks to seconds.

        Returns absolute time in seconds for a chosen MIDI file time
        resolution (ticks per beat, also called PPQN or pulses per quarter
        note) and tempo (microseconds per beat).
        """
        scale = tempo * 1e-6 / ticks_per_beat
        sec = tick * scale
        #print("sec",sec)
        return sec
    
    def processTrack(self, track):
        uncomplete_note = {}
        partdict = {}
        acc_time = 0
        onnum, offnum=0,0
        for msg in track:
            msgdict = msg.dict()
            time = msgdict['time']
            acc_time += time
            if msgdict['type']=='note_on' and msgdict['velocity']!=0:
                onnum+=1
                key = (msgdict['note'], msgdict['channel'])
                cache = (msgdict['note'], acc_time, msgdict['velocity'], msgdict['channel'])
                uncomplete_note[key]=cache
                
            if msgdict['type']=='note_off' or (msgdict['type']=='note_on' and msgdict['velocity']==0):
                offnum+=1
                key = (msgdict['note'], msgdict['channel'])
                if key in uncomplete_note:
                    pitch, onset, veloc, chanl = uncomplete_note[key]
                    dura = acc_time-onset
                    new_key = (onset, pitch, chanl)
                    cache = (pitch, onset, dura, veloc, chanl)
                    partdict[new_key]=cache
                    del uncomplete_note[key]
                else:
                    print('Ooops, single note_off!!!???', msgdict, uncomplete_note)
                    
        print('partdict:',len(partdict), 'uncomplete_note',len(uncomplete_note),'onnum,offnum=',onnum, offnum)
        #sort resulting dict
        keys = list(partdict.keys())
        keys.sort() 
        itera = map(partdict.get, keys)
        flat = []
        for n in itera:
            flat.append(n) 
        return flat
    
    def seperateChannels(self, track_id, flat):
        parts = {}
        ticks_per_beat = self.mid.ticks_per_beat
        for trk in self.mid.tracks:
            if self.tempo:
                break
            for msg in trk:
                msgdict = msg.dict()
                if msgdict['type'] == 'set_tempo':
                    self.tempo = msgdict['tempo']
                    break
        for n in flat:
            pitch, onset, dura, veloc, chanl = n
            onset = self.tick2second(onset, ticks_per_beat, self.tempo)
            dura = self.tick2second(dura, ticks_per_beat, self.tempo)
            key = (track_id, chanl)
            val = (pitch, onset, dura, veloc)
            if key in parts:
                parts[key].append(val)
            else:
                parts[key]=[]
                parts[key].append(val)
        return parts
    
    def getInstrs(self):
        self.part_insrts={}
        val = 0 ###
        for track_id, track in enumerate(self.mid.tracks):
            for msg in track:
                msgdict = msg.dict()
                if msgdict['type']=='program_change':
                    key = (track_id, msgdict['channel'])
                    val = msgdict['program']
                    self.part_insrts[key]=val
            if len(self.part_insrts)<track_id+1:
                self.part_insrts[(track_id,0)] = val ###
        print('self.part_insrts',self.part_insrts)
        return self.part_insrts
        # ok
            
    def flatTracks(self):
        self.parts = {} #key = (track_id, chanl) val=(pitch, onset, dura, veloc)
        for t_i, track in enumerate(self.mid.tracks):
            print('track',t_i,'======================')
            flat = self.processTrack(track)
            if len(flat):
                parts =  self.seperateChannels(t_i, flat)
                self.parts.update(parts)
        for k in self.parts:
            print(k, len(self.parts[k])) 

if __name__ == '__main__':
    mseg = MidiSegmentation_multitrack('kmhm.mid')
    msp =  MidiSegPlay(mseg)
    while True:
        time_stamp = time.time()
        play_states = [1]
        msp.playSegContinue(play_states, time_stamp)


